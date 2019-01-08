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

#ifndef MODULES_PLANNING_PLANNING_BASE_H_
#define MODULES_PLANNING_PLANNING_BASE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ctpl/ctpl_stl.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/traffic_rule_config.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/apollo_app.h"
#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/planner/planner_dispatcher.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief PlanningBase module main class.
 */
class PlanningBase : public apollo::common::ApolloApp {                           // planning的基类
 public:
  PlanningBase() = default;                                                       // 默认的基类
  virtual ~PlanningBase();                                                        // 虚的析构函数

  virtual void RunOnce() = 0;                                                     // 接口函数, 主要的planning的逻辑在这个函数中实现

  // Watch dog timer
  virtual void OnTimer(const ros::TimerEvent&) = 0;                               // 看门狗定时器

  /**
   * @brief Plan the trajectory given current vehicle state
   */
  virtual apollo::common::Status Plan(                                            // 接口函数, 规划出响应的轨迹
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,           // TrajectoryPoint是在pnc_point.proto文件中定义的, 具有一个路径点, 速度, 加速度, 相对时间
      ADCTrajectory* trajectory) = 0;                                             // ADCTrajectory是在planning.proto文件中定义, 主要包括了path的总长度, 总时间, 轨迹点, 是否紧急刹车, 道路点 等等

 protected:
  void PublishPlanningPb(ADCTrajectory* trajectory_pb, double timestamp);        // ADCTrajectory是在planning.proto文件中定义, 是轨迹的一些字节段

  /**
   * @brief Fill the header and publish the planning message.
   */
  void Publish(planning::ADCTrajectory* trajectory) {                            // 填充planning 消息段的头部信息
    using apollo::common::adapter::AdapterManager;                               // 适配器管理者
    AdapterManager::FillPlanningHeader(Name(), trajectory);                      // 填充planning的头部
    AdapterManager::PublishPlanning(*trajectory);                                // 发布规划出来的轨迹
  }

  bool IsVehicleStateValid(const common::VehicleState& vehicle_state);           // 当前车辆的状态是否合法
  virtual void SetFallbackTrajectory(ADCTrajectory* cruise_trajectory);          // 设置自动驾驶轨迹的后备轨迹
  void CheckPlanningConfig();                                                    // 检查planning的配置文件

  double start_time_ = 0.0;                                                      // 初始化开始时间为0.0
  PlanningConfig config_;                                                        // planning的配置
  TrafficRuleConfigs traffic_rule_configs_;                                      // 交通规则
  const hdmap::HDMap* hdmap_ = nullptr;                                          // hdmap_是一个空指针
  std::unique_ptr<Planner> planner_;                                             // 独立拥有，  提供不同的planning策略
  std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;           // 可以发布的轨迹
  ros::Timer timer_;                                                             // ros的定时器
  std::unique_ptr<PlannerDispatcher> planner_dispatcher_;                        // 在planning_base.h中定义, dispatcher是调度员的意思
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNING_BASE_H_
