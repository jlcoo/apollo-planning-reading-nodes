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

#ifndef MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_STITCHER_H_
#define MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_STITCHER_H_

#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/reference_line/reference_line.h"

#include "modules/planning/common/trajectory/publishable_trajectory.h"

namespace apollo {
namespace planning {
// 拼接trajectory
class TrajectoryStitcher {                                                             // 轨迹的合并类
 public:
  TrajectoryStitcher() = delete;                                                       // 简单的封装接口, 不用构造函数, 直接用static的函数

  static void TransformLastPublishedTrajectory(const double x_diff,                    // 从上次最新发布的轨迹中转换
      const double y_diff, const double theta_diff,
      PublishableTrajectory* prev_trajectory);                                         // 转换为上一个轨迹

  static std::vector<common::TrajectoryPoint> ComputeStitchingTrajectory(              // 计算有没有可以合并的轨迹
      const common::VehicleState& vehicle_state, const double current_timestamp,       // 根据车辆的状态, 当前的时间戳， 做一个planning所需的时间， 上次发布的轨迹
      const double planning_cycle_time,
      const PublishableTrajectory* prev_trajectory);

 private:     // 计算位置投影
  static std::pair<double, double> ComputePositionProjection(const double x,           // 计算轨迹点的投影
      const double y, const common::TrajectoryPoint& matched_trajectory_point);        // x,y的坐标， 两条轨迹合并时的轨迹点
              // 重新初始化?
  static std::vector<common::TrajectoryPoint> ComputeReinitStitchingTrajectory(        // TrajectoryPoint中是有x,y,z的信息呀
      const common::VehicleState& vehicle_state);                                      // 车辆的状态
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_STITCHER_H_
