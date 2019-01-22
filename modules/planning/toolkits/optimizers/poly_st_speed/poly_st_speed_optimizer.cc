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

#include "modules/planning/toolkits/optimizers/poly_st_speed/poly_st_speed_optimizer.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/file.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/toolkits/optimizers/poly_st_speed/poly_st_graph.h"
#include "modules/planning/toolkits/optimizers/st_graph/speed_limit_decider.h"
#include "modules/planning/toolkits/optimizers/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;                                                              // 错误码
using apollo::common::Status;                                                                 // 模块的状态
using apollo::common::TrajectoryPoint;                                                        // 轨迹点
using apollo::planning_internal::STGraphDebug;                                                // st图中的debug信息

PolyStSpeedOptimizer::PolyStSpeedOptimizer()
    : SpeedOptimizer("PolyStSpeedOptimizer") {}                                               // 构造函数

bool PolyStSpeedOptimizer::Init(const PlanningConfig& config) {                               // 初始化函数
  if (is_init_) {                                                                             // 已经初始化后, 不能重复初始化
    AERROR << "Duplicated Init.";
    return false;
  }
  poly_st_speed_config_ =
      config.lane_follow_scenario_config().poly_st_speed_config();                            // 获得配置参数
  st_boundary_config_ = poly_st_speed_config_.st_boundary_config();
  is_init_ = true;
  return true;
}

Status PolyStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,
                                     const PathData& path_data,
                                     const TrajectoryPoint& init_point,
                                     const ReferenceLine& reference_line,
                                     const SpeedData& reference_speed_data,
                                     PathDecision* const path_decision,
                                     SpeedData* const speed_data) {                           // 进行处理(做优化器的处理)
  if (reference_line_info_->ReachedDestination()) {
    return Status::OK();
  }
  if (!is_init_) {
    AERROR << "Please call Init() before Process.";
    return Status(ErrorCode::PLANNING_ERROR, "Not init.");
  }

  if (path_data.discretized_path().NumOfPoints() == 0) {                                      // 离散的路径的没有点
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  StBoundaryMapper boundary_mapper(adc_sl_boundary, st_boundary_config_,                      // 做障碍物的boundary和decision进行绑定
                                   reference_line, path_data,
                                   poly_st_speed_config_.total_path_length(),
                                   poly_st_speed_config_.total_time(),
                                   reference_line_info_->IsChangeLanePath());

  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    DCHECK(path_obstacle->HasLongitudinalDecision());                                         // 检查是否与横向的策略
  }
  // step 1 get boundaries
  path_decision->EraseStBoundaries();                                                         // 获取boundary的边框
  if (boundary_mapper.CreateStBoundary(path_decision).code() ==
      ErrorCode::PLANNING_ERROR) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Mapping obstacle for qp st speed optimizer failed!");
  }

  for (const auto* obstacle : path_decision->path_obstacles().Items()) {                      // 迭代path障碍物上的所有条目
    auto id = obstacle->Id();
    auto* mutable_obstacle = path_decision->Find(id);

    if (!obstacle->st_boundary().IsEmpty()) {
      mutable_obstacle->SetBlockingObstacle(true);
    } else {
      path_decision->SetStBoundary(
          id, path_decision->Find(id)->reference_line_st_boundary());
    }
  }

  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, st_boundary_config_,                // 限速的决策者
                                        reference_line, path_data);
  SpeedLimit speed_limits;
  if (speed_limit_decider.GetSpeedLimits(path_decision->path_obstacles(),
                                         &speed_limits) != Status::OK()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "GetSpeedLimits for qp st speed optimizer failed!");
  }

  // step 2 perform graph search
  // make a poly_st_graph and perform search here.
  PolyStGraph poly_st_graph(poly_st_speed_config_, reference_line_info_,                     // 多项式的图, 在这个图中进行搜索
                            speed_limits);
  auto ret = poly_st_graph.FindStTunnel(                                                     // 在st空间中的图中获取一个通路
      init_point,
      reference_line_info_->path_decision()->path_obstacles().Items(),
      speed_data);
  if (!ret) {                                                                                // 没有的通路的话就报错
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to find st tunnel in PolyStGraph.");
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
