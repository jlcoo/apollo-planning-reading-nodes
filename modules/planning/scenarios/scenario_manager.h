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

#ifndef MODULES_PLANNING_SCENARIOS_SCENARIO_MANAGER_H_
#define MODULES_PLANNING_SCENARIOS_SCENARIO_MANAGER_H_

#include <memory>

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {
// 在prediction中也有一个ScenarioManager的类
class ScenarioManager final {   // final修饰的类不能被派生
 public:
  ScenarioManager() = default;                                                       // 默认的构造函数
  // 初始化 --> 注册 --> 更新
  bool Init();                                                                       // 初始化

  Scenario* mutable_scenario() { return scenario_.get(); }                           // 获取可以修改的场景

  void Update(const common::TrajectoryPoint& ego_point, const Frame& frame);         // 通过ego的轨迹和一帧数据进行更新场景

 private:
  void RegisterScenarios();                                                          // 私有的函数, 进行场景的注册

  ScenarioConfig::ScenarioType DecideCurrentScenario(                                // 根据当前ego的位置(轨迹点)和一帧数据决定应该执行那种场景
      const common::TrajectoryPoint& ego_point, const Frame& frame);

  common::util::Factory<ScenarioConfig::ScenarioType, Scenario>                      // 一个场景的类型对应着一个场景
      scenario_factory_;                  // 场景的工厂模式

  std::unique_ptr<Scenario> scenario_;                                               // 有一个唯一的Scenario类
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_SCENARIOS_SCENARIO_MANAGER_H_
