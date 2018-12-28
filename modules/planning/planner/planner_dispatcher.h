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

#ifndef MODULES_PLANNING_PLANNER_PLANNER_DISPATCHER_H_
#define MODULES_PLANNING_PLANNER_PLANNER_DISPATCHER_H_

#include <memory>
#include <string>

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/planner/planner.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief PlannerDispatcher module main class.
 */   // planner的调度员
class PlannerDispatcher {
 public:
  PlannerDispatcher() = default;
  virtual ~PlannerDispatcher() = default;

  virtual common::Status Init() {
    RegisterPlanners();   // 注册planner
    return common::Status::OK(); //初始化完成
  }

  virtual std::unique_ptr<Planner> DispatchPlanner() = 0;

 protected:
  void RegisterPlanners();  // 连成员变量一起继承
  // PlanningConfig_PlannerType即PlanningConfig::PlannerType是一个枚举值， Planner的基类
// enum PlanningConfig_PlannerType {
//   PlanningConfig_PlannerType_RTK = 0,
//   PlanningConfig_PlannerType_EM = 1,
//   PlanningConfig_PlannerType_LATTICE = 2,
//   PlanningConfig_PlannerType_NAVI = 3,
//   PlanningConfig_PlannerType_OPENSPACE = 4
// };   // PlanningConfig::PlannerType是EM
  common::util::Factory<PlanningConfig::PlannerType, Planner> planner_factory_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNER_PLANNER_DISPATCHER_H_
