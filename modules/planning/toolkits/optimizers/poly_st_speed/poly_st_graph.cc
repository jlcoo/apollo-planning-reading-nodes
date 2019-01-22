/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/toolkits/optimizers/poly_st_speed/poly_st_graph.h"

#include <algorithm>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/curve1d/quartic_polynomial_curve1d.h"
#include "modules/planning/toolkits/optimizers/poly_st_speed/speed_profile_cost.h"

namespace apollo {
namespace planning {
namespace {
constexpr double kEpsilon = 1e-6;                                                           // 匿名空间, 只有在当前文件中其作用
}

using apollo::common::ErrorCode;                                                            // 错误的代码号
using apollo::common::Status;                                                               // 模块的状态

PolyStGraph::PolyStGraph(const PolyStSpeedConfig &config,
                         const ReferenceLineInfo *reference_line_info,
                         const SpeedLimit &speed_limit)
    : config_(config),
      reference_line_info_(reference_line_info),                                            // 利用初始化列表进行构造函数
      reference_line_(reference_line_info->reference_line()),
      speed_limit_(speed_limit) {}

bool PolyStGraph::FindStTunnel(                                                             // 找到图中的一个通道
    const common::TrajectoryPoint &init_point,
    const std::vector<const PathObstacle *> &obstacles,                                     // path上的障碍物
    SpeedData *const speed_data) {
  CHECK_NOTNULL(speed_data);                                                                // 容错, 检查指针不为空

  // set init point
  init_point_ = init_point;                                                                 // 设置起点
  unit_s_ = std::fmax(0.1, init_point_.v() / 4.0);                                          // 进行细化的s刻度

  // sample end points
  std::vector<std::vector<STPoint>> points;                                                 // 采样的点
  if (!SampleStPoints(&points)) {                                                           // 采样的点放到一个二维数组中
    AERROR << "Fail to sample st points.";
    return false;
  }

  PolyStGraphNode min_cost_node;                                                            // 最小代价的点
  if (!GenerateMinCostSpeedProfile(points, obstacles, &min_cost_node)) {                    // 产生一个最小的代价点
    AERROR << "Fail to search min cost speed profile.";
    return false;
  }
  ADEBUG << "min_cost_node s = " << min_cost_node.st_point.s()                              // debug一下最小代价点的s(路程)和时间点(t)
         << ", t = " << min_cost_node.st_point.t();
  speed_data->Clear();                                                                      // 速度的数据全部清空
  constexpr double delta_t = 0.1;  // output resolution, in seconds
  const auto curve = min_cost_node.speed_profile;                                           // 最小节点的速度曲线
  for (double t = 0.0; t < planning_time_; t += delta_t) {                                  // 时间的分辨率设置为0.1s
    const double s = curve.Evaluate(0, t);
    const double v = curve.Evaluate(1, t);
    const double a = curve.Evaluate(2, t);
    const double da = curve.Evaluate(3, t);                                                 // 通过5次多项式计算一些值(速度， 路程， 加速度， 加速度的微分)
    speed_data->AppendSpeedPoint(s, t, v, a, da);                                           // 新增一个速度的点
  }
  return true;
}

bool PolyStGraph::GenerateMinCostSpeedProfile(                                              // 生成一个最小代价函数值对应的速度的曲线
    const std::vector<std::vector<STPoint>> &points,                                        // 二维的数组
    const std::vector<const PathObstacle *> &obstacles,                                     // 障碍物的数组
    PolyStGraphNode *const min_cost_node) {                                                 // 最小代价对应的结点
  CHECK_NOTNULL(min_cost_node);                                                             // 检查坐标
  PolyStGraphNode start_node = {STPoint(0.0, 0.0), init_point_.v(),                         // 起点
                                init_point_.a()};
  SpeedProfileCost cost(config_, obstacles, speed_limit_, init_point_);                     // 速度曲线的代价配置
  double min_cost = std::numeric_limits<double>::max();
  for (const auto &level : points) {                                                        // 看看有多少个level
    for (const auto &st_point : level) {                                                    // 迭代一个level上的点
      const double speed_limit = speed_limit_.GetSpeedLimitByS(st_point.s());               // 通过s点获得对应的限速信息
      constexpr int num_speed = 10;                                                         // 将两个点再细分10个点
      for (double v = 0; v < speed_limit + kEpsilon;
           v += speed_limit / num_speed) {
        PolyStGraphNode node = {st_point, v, 0.0};                                          // 构建一个多项式的图中的点
        node.speed_profile = QuarticPolynomialCurve1d(                                      // 新建一个5次多项式
            0.0, start_node.speed, start_node.accel, node.st_point.s(),
            node.speed, node.st_point.t());
        const double c =
            cost.Calculate(node.speed_profile, st_point.t(), min_cost);                     // 计算对应点的代价函数
        if (c < min_cost) {                                                                 // 取出最小的代价函数值和对应的图中的节点
          *min_cost_node = node;
          min_cost = c;
        }
      }
    }
  }
  return true;
}

bool PolyStGraph::SampleStPoints(                                                           // 采样点
    std::vector<std::vector<STPoint>> *const points) {                                      // 一个二维的数组
  CHECK_NOTNULL(points);
  constexpr double start_t = 6.0;                                                           // 开始的时间为6秒
  constexpr double start_s = 0.0;
  for (double t = start_t; t <= planning_time_; t += unit_t_) {                             // 从第6秒开始采样
    std::vector<STPoint> level_points;
    for (double s = start_s; s < planning_distance_ + kEpsilon; s += unit_s_) {
      level_points.emplace_back(s, t);                                                      // 构建新的采样点
    }
    points->push_back(std::move(level_points));
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
