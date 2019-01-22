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

#ifndef MODULES_PLANNING_TASKS_POLY_ST_SPEED_POLY_ST_GRAPH_H_
#define MODULES_PLANNING_TASKS_POLY_ST_SPEED_POLY_ST_GRAPH_H_

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/poly_st_speed_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/math/curve1d/quartic_polynomial_curve1d.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class PolyStGraph {                                                                // ST 坐标系下的多项式的图
 public:
  explicit PolyStGraph(const PolyStSpeedConfig &config,                            // 多项式st坐标系下的配置项
                       const ReferenceLineInfo *reference_line_info,               // 中心参考线的各种信息
                       const SpeedLimit &speed_limit);                             // 速度的限制

  ~PolyStGraph() = default;                                                        // 析构函数

  bool FindStTunnel(const common::TrajectoryPoint &init_point,                     // 轨迹的起点
                    const std::vector<const PathObstacle *> &obstacles,            // path上的障碍物
                    SpeedData *const speed_data);                                  // speed相关的数据(data)

 private:
  struct PolyStGraphNode {                                                         // 多项式的点(st多项式图中的一个节点)
   public:
    PolyStGraphNode() = default;                                                   // 默认的构造函数

    PolyStGraphNode(const STPoint &point_st, const double speed,                   // 通过一个st的点, 速度， 加速度进行构造
                    const double accel)
        : st_point(point_st), speed(speed), accel(accel) {}

    STPoint st_point;
    double speed = 0.0;
    double accel = 0.0;
    QuarticPolynomialCurve1d speed_profile;                                        // 还有一个5项多项式的曲线
  };

  bool GenerateMinCostSpeedProfile(                                                // 产生最小代价函数值的速度曲线
      const std::vector<std::vector<STPoint>> &points,                             // 二维的点
      const std::vector<const PathObstacle *> &obstacles,                          // 障碍物的集合
      PolyStGraphNode *const min_cost_node);                                       // 最小代价的点

  bool SampleStPoints(std::vector<std::vector<STPoint>> *const points);            // 采用点的二维数组

 private:
  PolyStSpeedConfig config_;                                                       // st多项式优化器的配置项
  common::TrajectoryPoint init_point_;                                             // 轨迹的起点
  const ReferenceLineInfo *reference_line_info_ = nullptr;                         // 中心参考线的信息
  const ReferenceLine &reference_line_;                                            // 中心参考线
  const SpeedLimit &speed_limit_;                                                  // 速度的限制信息

  double unit_t_ = 1.0;                                                            // 时间轴上的单位为1
  double unit_s_ = 5.0;                                                            // s(距离)的单位
  double planning_distance_ = 100.0;                                               // 做planning的距离为100米
  double planning_time_ = 6.0;                                                     // 做planning的时间为6秒左右
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_POLY_ST_SPEED_POLY_ST_GRAPH_H_
