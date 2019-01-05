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
 * @file publishable_trajectory.h
 **/

#ifndef MODULES_PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H_
#define MODULES_PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H_

#include <vector>

#include "modules/planning/proto/planning.pb.h"

#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {
// 可发布的轨迹, 从离散轨迹中继承而来
class PublishableTrajectory : public DiscretizedTrajectory {
 public:
  PublishableTrajectory() = default;   // 默认构造函数

  PublishableTrajectory(const double header_time,
                        const DiscretizedTrajectory& discretized_trajectory);
  /**
   * Create a publishable trajectory based on a trajectory protobuf
   */     // ADCTrajectory 来自于 protobuf
  explicit PublishableTrajectory(const ADCTrajectory& trajectory_pb);

  double header_time() const;
  // 填充轨迹
  void PopulateTrajectoryProtobuf(ADCTrajectory* trajectory_pb) const;

 private:
  double header_time_ = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H_
