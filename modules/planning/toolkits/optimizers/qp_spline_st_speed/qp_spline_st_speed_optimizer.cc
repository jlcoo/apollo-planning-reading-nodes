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

#include "modules/planning/toolkits/optimizers/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/file.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/toolkits/optimizers/qp_spline_st_speed/qp_piecewise_st_graph.h"
#include "modules/planning/toolkits/optimizers/qp_spline_st_speed/qp_spline_st_graph.h"
#include "modules/planning/toolkits/optimizers/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;                                                     // 错误码
using apollo::common::Status;                                                        // 状态码
using apollo::common::TrajectoryPoint;                                               // 轨迹点
using apollo::planning_internal::STGraphDebug;                                       // st图中的debug信息

QpSplineStSpeedOptimizer::QpSplineStSpeedOptimizer()                                 // 构造函数
    : SpeedOptimizer("QpSplineStSpeedOptimizer") {}

bool QpSplineStSpeedOptimizer::Init(const PlanningConfig& config) {                  // 二次规划的初始化
  qp_st_speed_config_ =
      config.lane_follow_scenario_config().qp_st_speed_config();
  st_boundary_config_ = qp_st_speed_config_.st_boundary_config();
  std::vector<double> init_knots;                                                    // 初始化节点的个数
  spline_generator_.reset(new Spline1dGenerator(init_knots, 5));                     // 生成5个节点的一维spline
  is_init_ = true;
  return true;
}

Status QpSplineStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,          // QP 优化器的处理过程
                                         const PathData& path_data,
                                         const TrajectoryPoint& init_point,
                                         const ReferenceLine& reference_line,
                                         const SpeedData& reference_speed_data,
                                         PathDecision* const path_decision,
                                         SpeedData* const speed_data) {
  if (reference_line_info_->ReachedDestination()) {                                  // 检查参考线信息, 看看有没有到达目的地
    return Status::OK();
  }
  if (!is_init_) {                                                                   // 没有初始化的话应该是不行的
    AERROR << "Please call Init() before Process.";
    return Status(ErrorCode::PLANNING_ERROR, "Not init.");
  }

  if (path_data.discretized_path().NumOfPoints() == 0) {                             // path data中国的离散path不存在也是不行的
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  StBoundaryMapper boundary_mapper(                                                 // 将障碍物的boundary和decision绑定在一起
      adc_sl_boundary, st_boundary_config_, reference_line, path_data,
      qp_st_speed_config_.total_path_length(), qp_st_speed_config_.total_time(),
      reference_line_info_->IsChangeLanePath());

  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {       // 迭代所有的path上的障碍物
    DCHECK(path_obstacle->HasLongitudinalDecision());                               // 看看是否有纵向的策略
  }
  // step 1 get boundaries
  path_decision->EraseStBoundaries();                                               // 将path上的决策对应的边框(boundary)全部清空
  if (boundary_mapper.CreateStBoundary(path_decision).code() ==
      ErrorCode::PLANNING_ERROR) {                                                  // 做一个操作记得进行错误检查
    return Status(ErrorCode::PLANNING_ERROR,
                  "Mapping obstacle for qp st speed optimizer failed!");
  }

  std::vector<const StBoundary*> boundaries;                                        // 迭代所有的障碍物
  for (auto* obstacle : path_decision->path_obstacles().Items()) {
    auto id = obstacle->Id();                                                       // 障碍物的id值
    if (!obstacle->st_boundary().IsEmpty()) {                                       // 如果障碍物的st boundary还存在的话, 就会设置为堵塞的障碍物
      path_decision->Find(id)->SetBlockingObstacle(true);
      boundaries.push_back(&obstacle->st_boundary());                               // 直接增加到boundary的数组中
    } else if (FLAGS_enable_side_vehicle_st_boundary &&                             // FLAGS_enable_side_vehicle_st_boundary设置为false
               (adc_sl_boundary.start_l() > 2.0 ||                                  // 自动驾驶的宽度太宽
                adc_sl_boundary.end_l() < -2.0)) {
      if (obstacle->obstacle()->IsVirtual()) {                                      // 虚拟障碍物的话就会直接跳过， 进行下一个障碍物的循环
        continue;
      }
      if (path_decision->Find(id)->reference_line_st_boundary().IsEmpty()) {        // 中心参考线的boundary是空的话， 也会进行下一个障碍物的迭代
        continue;
      }
      auto st_boundary_copy =
          path_decision->Find(id)->reference_line_st_boundary();                    // 获得中心参考线的副本
      auto st_boundary = st_boundary_copy.CutOffByT(3.5);                           // 设置截止时间为3.5秒
      if (!st_boundary.IsEmpty()) {                                                 // st坐标系下的boundary不为空
        auto decision = obstacle->LongitudinalDecision();                           // 取出纵向的策略
        if (decision.has_yield()) {                                                 // 根据决策的策略进而社则st boundary的策略(类型)
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::YIELD);
        } else if (decision.has_overtake()) {
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::OVERTAKE);
        } else if (decision.has_follow()) {
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::FOLLOW);
        } else if (decision.has_stop()) {
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::STOP);
        } else if (decision.has_ignore()) {
          continue;
        } else {
          AWARN << "Obstacle " << id << " has unhandled decision type: "
                << decision.ShortDebugString();
        }
        st_boundary.SetId(st_boundary_copy.id());                                  // 设置st boundary的id值
        st_boundary.SetCharacteristicLength(
            st_boundary_copy.characteristic_length());                             // 设置特征的长度

        path_decision->SetStBoundary(id, st_boundary);
        boundaries.push_back(&obstacle->st_boundary());                            // 设置决策对应的st boundary和保存到boundary的数组中
      }
    }
  }

  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, st_boundary_config_,      // 速度限制的决策者
                                        reference_line, path_data);
  SpeedLimit speed_limits;                                                         // 限速信息
  if (speed_limit_decider.GetSpeedLimits(path_decision->path_obstacles(),
                                         &speed_limits) != Status::OK()) {         // 通过SpeedLimitDecider获取对应的限速信息
    return Status(ErrorCode::PLANNING_ERROR,
                  "GetSpeedLimits for qp st speed optimizer failed!");             // 出错就返回planning的错误
  }

  // step 2 perform graph search
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();                    // 获取车辆的具体参数
  QpSplineStGraph st_graph(spline_generator_.get(), qp_st_speed_config_,
                           veh_param, reference_line_info_->IsChangeLanePath());   // 构造一个st的图

  StGraphData st_graph_data(boundaries, init_point, speed_limits,
                            path_data.discretized_path().Length());                // 构造一个st图中需要的数据

  STGraphDebug* st_graph_debug = reference_line_info_->mutable_debug()
                                     ->mutable_planning_data()
                                     ->add_st_graph();                             // 设置debug的接口

  std::pair<double, double> accel_bound = {
      qp_st_speed_config_.preferred_min_deceleration(),
      qp_st_speed_config_.preferred_max_acceleration()};                           // 加速度的boundary
  st_graph.SetDebugLogger(st_graph_debug);
  auto ret = st_graph.Search(st_graph_data, accel_bound, reference_speed_data,
                             speed_data);                                          // 搜索对应的值
  if (ret != Status::OK()) {                                                       // 如果失败的话就会报错
    AERROR << "Failed to solve with ideal acceleration conditions. Use "
              "secondary choice instead.";

    accel_bound.first = qp_st_speed_config_.min_deceleration();                    // 并且重新设置加速度的边界
    accel_bound.second = qp_st_speed_config_.max_acceleration();
    ret = st_graph.Search(st_graph_data, accel_bound, reference_speed_data,
                          speed_data);                                             // 重新进搜索

    // backup plan: use piecewise_st_graph
    if (ret != Status::OK()) {                                                     // 如果还是不能
      AERROR << "Spline QP speed solver Failed. "
             << "Using finite difference method.";
      QpPiecewiseStGraph piecewise_st_graph(qp_st_speed_config_);                  // qp 进行分段的st图
      ret = piecewise_st_graph.Search(st_graph_data, speed_data, accel_bound);     // 进行搜索

      if (ret != Status::OK()) {                                                   // 出错记录debug信息
        std::string msg = common::util::StrCat(
            Name(), ": Failed to search graph with quadratic programming!");
        AERROR << msg;
        RecordSTGraphDebug(st_graph_data, st_graph_debug);
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
    }
  }

  // record debug info
  RecordSTGraphDebug(st_graph_data, st_graph_debug);                               // 没有错误也要记录debug的信息
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
