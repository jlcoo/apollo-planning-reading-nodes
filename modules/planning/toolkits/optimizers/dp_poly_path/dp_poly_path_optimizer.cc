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

#include "modules/planning/toolkits/optimizers/dp_poly_path/dp_poly_path_optimizer.h"

#include <string>
#include <utility>
#include <vector>

#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/toolkits/optimizers/dp_poly_path/dp_road_graph.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

DpPolyPathOptimizer::DpPolyPathOptimizer()
    : PathOptimizer("DpPolyPathOptimizer") {}    // 构造的时候就是给个名字

bool DpPolyPathOptimizer::Init(const PlanningConfig &config) {
  config_ = config.lane_follow_scenario_config().dp_poly_path_config();   // config是从planning config的文件中的lane 跟随场景的配置中获取的
  is_init_ = true;
  return true;
}

Status DpPolyPathOptimizer::Process(const SpeedData &speed_data,                    // 速度相关的数据, SpeedData是一个类
                                    const ReferenceLine &,                          // 车道中心线, ReferenceLine也是一个类
                                    const common::TrajectoryPoint &init_point,      // 轨迹点, 里面有很多的信息, TrajectoryPoint是在pnc_point.proto文件中定义
                                    PathData *const path_data) {                    // 路径数据, 应该是初略解的输出, PathData也是一个类
  if (!is_init_) {
    AERROR << "Please call Init() before Process().";
    return Status(ErrorCode::PLANNING_ERROR, "Not inited.");
  }
  CHECK_NOTNULL(path_data);
  DPRoadGraph dp_road_graph(config_, *reference_line_info_, speed_data);           // 生成路网图, DPRoadGraph是一个类对象
  dp_road_graph.SetDebugLogger(reference_line_info_->mutable_debug());

  if (!dp_road_graph.FindPathTunnel(
          init_point,
          reference_line_info_->path_decision()->path_obstacles().Items(),
          path_data)) {
    AERROR << "Failed to find tunnel in road graph";
    return Status(ErrorCode::PLANNING_ERROR, "dp_road_graph path generation");
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
