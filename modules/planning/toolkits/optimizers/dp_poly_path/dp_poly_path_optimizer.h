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
 * @file dp_poly_path_optimizer.h
 **/

#ifndef MODULES_PLANNING_TASKS_DP_POLY_PATH_OPTIMIZER_H_
#define MODULES_PLANNING_TASKS_DP_POLY_PATH_OPTIMIZER_H_

#include <string>

#include "modules/planning/proto/dp_poly_path_config.pb.h"

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/toolkits/optimizers/path_optimizer.h"

namespace apollo {
namespace planning {

/**
 * @class DpPolyPathOptimizer
 * @brief DpPolyPathOptimizer does path planning with dynamic programming
 * algorithm.
 */  // 路径的动态规划算法
class DpPolyPathOptimizer : public PathOptimizer {                                       // 动态规划的多项式的path优化器
 public:
  DpPolyPathOptimizer();                                                                 // 默认的构造函数

  bool Init(const PlanningConfig &config) override;                                      // 通过配置参数进行初始化

 private:
  apollo::common::Status Process(const SpeedData &speed_data,                            // 优化器的处理过程, 速度的数据
                                 const ReferenceLine &reference_line,                    // 道路中心参考线
                                 const common::TrajectoryPoint &init_point,              // 轨迹点
                                 PathData *const path_data) override;                    // 路径的数据(path data)

 private:
  DpPolyPathConfig config_;                                                              // PolyPathConfig是一个配置文件， 配置文件为modules/planning/proto/dp_poly_path_config.proto
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_DP_POLY_PATH_OPTIMIZER_H_
