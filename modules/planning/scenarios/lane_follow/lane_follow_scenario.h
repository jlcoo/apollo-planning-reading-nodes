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

#ifndef MODULES_PLANNING_SCENARIOS_LANE_FOLLLOW_SCENARIO_H_
#define MODULES_PLANNING_SCENARIOS_LANE_FOLLLOW_SCENARIO_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/planning/scenarios/scenario.h"
#include "modules/planning/toolkits/optimizers/task.h"

namespace apollo {
namespace planning {

class LaneFollowScenario : public Scenario {                                                       // 跟车的场景
 public:
  LaneFollowScenario() : Scenario(ScenarioConfig::LANE_FOLLOW) {}                                  // 构造函数, 场景的类型默认是跟车
  virtual ~LaneFollowScenario() = default;                                                         // 析构函数

  bool Init(const PlanningConfig& config) override;                                                // 初始化跟车的场景

  common::Status Process(const common::TrajectoryPoint& planning_init_point,                       // 处理跟车的场景
                         Frame* frame) override;

  ScenarioConfig::ScenarioType Transfer(                                                           // 进行场景的切换
      const ScenarioConfig::ScenarioType& current_scenario,                                        // 当前场景的类型
      const common::TrajectoryPoint& ego_point,                                                    // 一个轨迹点
      const Frame& frame) const override;                                                          // 一帧数据

 private:
  void RegisterTasks();                                                                            // 注册任务进去

  common::Status PlanOnReferenceLine(                                                              // 在中心参考线上进行plan(规划)
      const common::TrajectoryPoint& planning_start_point, Frame* frame,                           // 做planning的起点, 一帧数据
      ReferenceLineInfo* reference_line_info);                                                     // 中心参考线的info(信息)

  std::vector<common::SpeedPoint> DummyHotStart(                                                   // 虚拟的热启动, 返回的是速度点的数组
      const common::TrajectoryPoint& planning_init_point);                                         // planning的初始点

  void GenerateFallbackPathProfile(const ReferenceLineInfo* reference_line_info,                   // c产生path(路程反馈)的文件， 输入是参考线的信息， 输出为path_data
                                   PathData* path_data);

  common::SLPoint GetStopSL(const ObjectStop& stop_decision,                                       // 获取停止点在中心参考线中的位置
                            const ReferenceLine& reference_line) const;

  void RecordObstacleDebugInfo(ReferenceLineInfo* reference_line_info);                            // 记录下障碍物debug的信息, 输入是中心参考线的信息

  void RecordDebugInfo(ReferenceLineInfo* reference_line_info,                                     // 记录下debug的信息， 中心参考线， 名字， 和时间差
                       const std::string& name, const double time_diff_ms);

  apollo::common::util::Factory<TaskType, Task> task_factory_;                                     // 任务工厂(任务的类型和具体的任务)
                                                                                                   // 任务的数组
  std::vector<std::unique_ptr<Task>> tasks_;                                                       // tasks_是std::unique_ptr<Task>指针的共享指针。

  SpeedProfileGenerator speed_profile_generator_;                                                  // 产生速度反馈的文件
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_SCENARIOS_LANE_FOLLLOW_SCENARIO_H_
