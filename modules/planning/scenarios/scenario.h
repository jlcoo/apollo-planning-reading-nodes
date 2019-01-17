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
 * @file
 **/

#ifndef MODULES_PLANNING_SCENARIOS_SCENARIO_H_
#define MODULES_PLANNING_SCENARIOS_SCENARIO_H_

#include <string>

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {
// Scenario是场景的意思, 通过ScenarioConfig的配置文件, 构造场景, 然后定义一些接口
class Scenario {                                                                          // 场景抽象的基类
 public:
  Scenario() = default;                                                                   // 默认的构造函数

  explicit Scenario(const ScenarioConfig::ScenarioType& scenario_type)                    // 通过场景的类型进行构造, 但是禁止隐式转换
      : scenario_type_(scenario_type) {}                                                  // 9种不同的场景, 在modules/planning/proto/planning_config.proto中定义

  virtual ~Scenario() = default;                                                          // 默认的虚的析构函数

  virtual ScenarioConfig::ScenarioType scenario_type() const;                             // 返回场景的信息

  virtual bool Init(const PlanningConfig& config) = 0;                                    // 初始化
 
  virtual common::Status Process(                                                         // 通过轨迹初始点和一帧数据进行场景的处理
      const common::TrajectoryPoint& planning_init_point, Frame* frame) = 0;        

  virtual ScenarioConfig::ScenarioType Transfer(                                          // 切换当前的场景
      const ScenarioConfig::ScenarioType& current_scenario,                               // 应该在派生类中调用, 这样就形成了多态
      const common::TrajectoryPoint& ego_point, const Frame& frame) const = 0;
  // 初始化， 处理和转换
 protected:
  bool is_init_ = false;                                                                  // 默认是没有被初始化
  const ScenarioConfig::ScenarioType scenario_type_;                                      // 场景的类型
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_SCENARIOS_SCENARIO_H_
