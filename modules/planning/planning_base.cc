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

#include "modules/planning/planning_base.h"

#include <algorithm>
#include <list>
#include <vector>

#include "google/protobuf/repeated_field.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/trajectory_stitcher.h"
#include "modules/planning/planner/em/em_planner.h"
#include "modules/planning/planner/lattice/lattice_planner.h"
#include "modules/planning/planner/navi/navi_planner.h"
#include "modules/planning/planner/open_space/open_space_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#include "modules/planning/toolkits/deciders/traffic_decider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;                                                        // 错误码
using apollo::common::Status;                                                           // 状态, Status是一个类
using apollo::common::TrajectoryPoint;                                                  // 轨迹点
using apollo::common::VehicleState;                                                     // 车的状态
using apollo::common::VehicleStateProvider;                                             // 车状态的提供者
using apollo::common::adapter::AdapterManager;                                          // 适配器管理者
using apollo::common::time::Clock;                                                      // 时间的时钟脉冲, Clock是一个类, 在time.h文件中定义
using apollo::hdmap::HDMapUtil;                                                         // 高精地图的使用者, HDMapUtil也是一个类

PlanningBase::~PlanningBase() {}

void PlanningBase::CheckPlanningConfig() {                                              // 检查planning的配置文件对不对
  if (config_.has_lane_follow_scenario_config() &&                                      // 是否有follow 场景的配置项
      config_.lane_follow_scenario_config().has_dp_st_speed_config()) {                 // 是否有dp的配置项
    const auto& dp_st_speed_config =
        config_.lane_follow_scenario_config().dp_st_speed_config();
    CHECK(dp_st_speed_config.has_matrix_dimension_s());        // 150
    CHECK_GT(dp_st_speed_config.matrix_dimension_s(), 3);      // 150
    CHECK_LT(dp_st_speed_config.matrix_dimension_s(), 10000);
    CHECK(dp_st_speed_config.has_matrix_dimension_t());        // 8
    CHECK_GT(dp_st_speed_config.matrix_dimension_t(), 3);      // 大于3
    CHECK_LT(dp_st_speed_config.matrix_dimension_t(), 10000);  // 小于10000
  }
  // TODO(All): check other config params
}

bool PlanningBase::IsVehicleStateValid(const VehicleState& vehicle_state) {             // 检查车辆的状态是否合法
  if (std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||                 // 所有的值都不能为空nan
      std::isnan(vehicle_state.z()) || std::isnan(vehicle_state.heading()) ||
      std::isnan(vehicle_state.kappa()) ||
      std::isnan(vehicle_state.linear_velocity()) ||
      std::isnan(vehicle_state.linear_acceleration())) {
    return false;
  }
  return true;
}

void PlanningBase::PublishPlanningPb(ADCTrajectory* trajectory_pb,                      // 发布轨迹
                                     double timestamp) {                                // 时间戳
  trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);                        // 设置轨迹的时间戳?
  if (AdapterManager::GetPrediction() &&                                                // 从适配器的管理者中拿出一个预测对象
      !AdapterManager::GetPrediction()->Empty()) {                                      // 容错检查
    const auto& prediction =
        AdapterManager::GetPrediction()->GetLatestObserved();                           // 获得最新的观察值
    trajectory_pb->mutable_header()->set_lidar_timestamp(
        prediction.header().lidar_timestamp());                                         // 设置雷达的时间戳
    trajectory_pb->mutable_header()->set_camera_timestamp(
        prediction.header().camera_timestamp());                                        // 设置相机的时间戳
    trajectory_pb->mutable_header()->set_radar_timestamp(
        prediction.header().radar_timestamp());                                         // 设置激光雷达的时间戳
  }

  // TODO(all): integrate reverse gear
  trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);                                 // 从canbus获取档位信息
  if (AdapterManager::GetRoutingResponse() &&                                           // 获取routing的初始值
      !AdapterManager::GetRoutingResponse()->Empty()) {                                 // 容错检查
    trajectory_pb->mutable_routing_header()->CopyFrom(                                  // 将routing的头直接复制给轨迹(trajectory)中
        AdapterManager::GetRoutingResponse()->GetLatestObserved().header());
  }

  if (FLAGS_use_planning_fallback &&
      trajectory_pb->trajectory_point_size() == 0) {
    SetFallbackTrajectory(trajectory_pb);                                               // 设置自动驾驶轨迹的后备轨迹
  }

  // NOTICE:
  // Since we are using the time at each cycle beginning as timestamp, the
  // relative time of each trajectory point should be modified so that we can
  // use the current timestamp in header.

  // auto* trajectory_points = trajectory_pb.mutable_trajectory_point();
  if (!FLAGS_planning_test_mode) {                                                      // FLAGS_planning_test_mode为false, 就是进行下面的语句
    const double dt = timestamp - Clock::NowInSeconds();                                // 修改时间戳
    for (auto& p : *trajectory_pb->mutable_trajectory_point()) {                        // 迭代轨迹中的每个轨迹点
      p.set_relative_time(p.relative_time() + dt);                                      // 相对时间加上一个dt
    }
  }
  Publish(trajectory_pb);                                                               // 把计算出来的轨迹(通过protocol buf)发布出去
}

void PlanningBase::SetFallbackTrajectory(ADCTrajectory* trajectory_pb) {                // 设置自动驾驶车辆的轨迹(trajectory)
  CHECK_NOTNULL(trajectory_pb);
  // use planning trajecotry from last cycle
  auto* last_planning = AdapterManager::GetPlanning();                                  // 使用上一个周期的规划出来的轨迹, (调用适配器管理者获取planning模块)
  if (last_planning != nullptr && !last_planning->Empty()) {                            // 容错检查
    const auto& traj = last_planning->GetLatestObserved();                              // 获得最新的轨迹

    const double current_time_stamp = trajectory_pb->header().timestamp_sec();          // 当前的时间戳
    const double pre_time_stamp = traj.header().timestamp_sec();                        // 先前的时间戳   

    for (int i = 0; i < traj.trajectory_point_size(); ++i) {                            // 迭代上个轨迹的每个点
      const double t = traj.trajectory_point(i).relative_time() +                       // 重新计算时间戳
                       pre_time_stamp - current_time_stamp;
      auto* p = trajectory_pb->add_trajectory_point();                                  // 将上次的轨迹复制为trajectory_pb, 只是改变了时间戳的信息
      p->CopyFrom(traj.trajectory_point(i));
      p->set_relative_time(t);
    }
  }
}

}  // namespace planning
}  // namespace apollo
