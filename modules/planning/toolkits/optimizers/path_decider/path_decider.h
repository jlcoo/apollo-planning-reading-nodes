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

/**
 * @file
 **/

#ifndef MODULES_PLANNING_TASKS_PATH_DECIDER_PATH_DECIDER_H_
#define MODULES_PLANNING_TASKS_PATH_DECIDER_PATH_DECIDER_H_

#include <limits>
#include <string>

#include "modules/planning/toolkits/optimizers/task.h"

namespace apollo {
namespace planning {

class PathDecider : public Task {                                                // 路径决策者直接从task中继承而来
 public:
  PathDecider();                                                                 // 构造函数
  ~PathDecider() = default;                                                      // 默认的析构函数

  apollo::common::Status Execute(                                                // 执行path的决策者
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  apollo::common::Status Process(const PathData &path_data,                      // 处理path中的data(数据)
                                 PathDecision *const path_decision);             // 添加上path上的决策(decision)

  bool MakeObjectDecision(const PathData &path_data,                             // 做对象的决策
                          PathDecision *const path_decision);

  bool MakeStaticObstacleDecision(const PathData &path_data,                     // 做静态障碍物的决策
                                  PathDecision *const path_decision);

  ObjectStop GenerateObjectStopDecision(                                         // 做停止障碍物的决策
      const PathObstacle &path_obstacle) const;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_PATH_DECIDER_PATH_DECIDER_H_
