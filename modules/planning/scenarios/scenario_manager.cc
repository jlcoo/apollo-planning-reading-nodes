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

#include "modules/planning/scenarios/scenario_manager.h"

#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"

namespace apollo {
namespace planning {

bool ScenarioManager::Init() {                                                          // 场景管理者的初始化函数
  RegisterScenarios();                                                                  // 先注册情景，再创建情景
  scenario_ = scenario_factory_.CreateObject(ScenarioConfig::LANE_FOLLOW);              // 跟车的场景
  return true;
}
// 注册一个follow的
void ScenarioManager::RegisterScenarios() {                                             // 向管理者中注册一个跟车的场景， LaneFollowScenari是一个类
  scenario_factory_.Register(ScenarioConfig::LANE_FOLLOW, []() -> Scenario* {
    return new LaneFollowScenario();   // 直接创建一个laneFollow的情景
  });
}

void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,                  // 更新场景， 通过一个当前的轨迹点和一帧数据
                             const Frame& frame) {                                      // 数据帧
  const auto new_scenario_type = DecideCurrentScenario(ego_point, frame);               // 决定当前使用那个场景
  if (new_scenario_type != scenario_->scenario_type()) {                                // 容错判断
    scenario_ = scenario_factory_.CreateObject(new_scenario_type);                      // 创建一个新的对象
  }
}

ScenarioConfig::ScenarioType ScenarioManager::DecideCurrentScenario(
    const common::TrajectoryPoint& ego_point, const Frame& frame) {                     // 通过场景的Transfer进行场景切换, 输入为场景的类型和当前的ego点还有就是一帧数据
  return scenario_->Transfer(scenario_->scenario_type(), ego_point, frame);
}

}  // namespace planning
}  // namespace apollo
