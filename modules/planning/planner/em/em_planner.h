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

#ifndef MODULES_PLANNING_PLANNER_EM_EM_PLANNER_H_
#define MODULES_PLANNING_PLANNER_EM_EM_PLANNER_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/planning/toolkits/optimizers/task.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class EMPlanner
 * @brief EMPlanner is an expectation maximization planner.
 */
// EMPlanner类继承于PlannerWithReferenceLine这个类
class EMPlanner : public PlannerWithReferenceLine {                                     // EM规划器, 最大期望的planner
 public:
  /**
   * @brief Constructor
   */
  EMPlanner() = default;                                                                // 默认的构造函数

  /**
   * @brief Destructor
   */
  virtual ~EMPlanner() = default;                                                       // 默认的析构函数

  std::string Name() override { return "EM"; }                                          // 规划器的名字

  common::Status Init(const PlanningConfig& config) override;                           // 通过planning的配置选项， 进行初始化

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  apollo::common::Status Plan(                                                          // EM planner进行规划的代码
      const common::TrajectoryPoint& planning_init_point,                               // 轨迹的初始化点
      Frame* frame) override;                                                           // 数据框(一帧的数据)
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNER_EM_EM_PLANNER_H_
