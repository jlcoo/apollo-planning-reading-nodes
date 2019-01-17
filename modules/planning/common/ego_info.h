/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file ego_info.h
 **/

#ifndef MODULES_PLANNING_COMMON_EGO_INFO_H_
#define MODULES_PLANNING_COMMON_EGO_INFO_H_

#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"

#include "modules/common/macro.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class EgoInfo {                                                                  // ego的信息
 public:
  ~EgoInfo() = default;                                                          // 默认的析构函数

  bool Update(const common::TrajectoryPoint& start_point,                        // 轨迹点
              const common::VehicleState& vehicle_state,                         // 车辆的状态
              const std::vector<const Obstacle*>& obstacles);                    // 障碍物的数量
  void Clear();                                                                  // 清空ego的信息

  common::TrajectoryPoint start_point() const { return start_point_; }           // 轨迹的起点

  common::VehicleState vehicle_state() const { return vehicle_state_; }          // 车辆的状态

  double front_clear_distance() const { return front_clear_distance_; }          // 前方的距离

 private:
  FRIEND_TEST(EgoInfoTest, EgoInfoSimpleTest);                                   // 做简单的测试

  void set_vehicle_state(const common::VehicleState& vehicle_state) {            // 设置车辆的状态
    vehicle_state_ = vehicle_state;                                              // 直接赋值
  }

  void set_start_point(const common::TrajectoryPoint& start_point) {             // 设置轨迹的起点
    start_point_ = start_point;                                                  // 直接赋值
  }

  void CalculateFrontObstacleClearDistance(                                      // 计算前方障碍物到车辆的距离
      const std::vector<const Obstacle*>& obstacles);

  // stitched point (at stitching mode)  拼接模式
  // or real vehicle point (at non-stitching mode)
  common::TrajectoryPoint start_point_;                                          // 起点

  // ego vehicle state
  common::VehicleState vehicle_state_;                                           // 车辆的状态

  double front_clear_distance_ = std::numeric_limits<double>::max();             // 初始值为最大

  common::VehicleConfig ego_vehicle_config_;                                     // 车辆的配置项

  DECLARE_SINGLETON(EgoInfo);                                                    // 单例模式
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_EGO_INFO_H_
