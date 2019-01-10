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

#ifndef MODULES_PLANNING_PLANNER_PLANNER_H_
#define MODULES_PLANNING_PLANNER_PLANNER_H_

#include <string>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/scenarios/scenario.h"
#include "modules/planning/scenarios/scenario_manager.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class Planner
 * @brief Planner is a base class for specific planners.
 *        It contains a pure virtual function Plan which must be implemented in
 * derived class.
 */
// 抽象出来的基类
class Planner {
 public:
  /**
   * @brief Constructor
   */
  Planner() = default;                                                                           // 默认的构造函数

  /**
   * @brief Destructor
   */
  virtual ~Planner() = default;                                                                  // 默认的析构函数

  virtual std::string Name() = 0;                                                                // 初始化planner(规划器), 纯虚函数
  virtual apollo::common::Status Init(const PlanningConfig& config) = 0;

  /**
   * @brief Compute trajectories for execution.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  virtual apollo::common::Status Plan( // 通过起点和当前planning的帧,规划出完整的轨迹
      const common::TrajectoryPoint& planning_init_point, Frame* frame) = 0;                     // 计算轨迹点(planner的输出就是轨迹trajectories)

 protected:                            // planning基类中使用到了聚合的思想
  PlanningConfig config_;              // planning配置项, 就是planning_config.pb.txt这个配置文件
  ScenarioManager scenario_manager_;   // 场景管理项, 来自Preception
  Scenario* scenario_;                 // 场景, from planning/scenario, 情景
};
// 用到了三重继承, 继承有点点深吧？， 继承于Planner, planner提供一些接口和数据成员
class PlannerWithReferenceLine : public Planner {   // 继承该类
 public:                                            // 带车道中心参考线的规划器
  /**
   * @brief Constructor
   */
  PlannerWithReferenceLine() = default;                                      // 默认的构造函数, 

  /**
   * @brief Destructor
   */
  virtual ~PlannerWithReferenceLine() = default;                             // 默认的析构函数

  /**
   * @brief Compute a trajectory for execution.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */   // PlanOnReferenceLine这个函数会计算执行轨迹
  virtual apollo::common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,   // TrajectoryPoint(轨迹的点)， Frame(是一个)
      ReferenceLineInfo* reference_line_info) {                           // ReferenceLineInfo(是) 计算得到的车道中心线的信息
    CHECK_NOTNULL(frame);
    return apollo::common::Status::OK();
  }
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNER_PLANNER_H_
