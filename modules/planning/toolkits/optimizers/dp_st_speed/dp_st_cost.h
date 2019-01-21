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

#ifndef MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_COST_H_
#define MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_COST_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/dp_st_speed_config.pb.h"

#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/toolkits/optimizers/dp_st_speed/st_graph_point.h"

namespace apollo {
namespace planning {

class DpStCost {                                                                               // st图中计算代价函数值
 public:
  explicit DpStCost(const DpStSpeedConfig& dp_st_speed_config,                                 // dp图中的动态规划的配置项
                    const std::vector<const PathObstacle*>& obstacles,                         // 保存障碍物的数组
                    const common::TrajectoryPoint& init_point);                                // 轨迹的起点

  float GetObstacleCost(const StGraphPoint& point);                                            // 获得障碍物的代价函数值

  float GetReferenceCost(const STPoint& point,                                                 // 参考线的代价函数值
                          const STPoint& reference_point) const;

  float GetSpeedCost(const STPoint& first, const STPoint& second,                              // 速度的代价函数值
                      const float speed_limit) const;

  float GetAccelCostByTwoPoints(const float pre_speed, const STPoint& first,                   // 加速度的代价函数值
                                 const STPoint& second);
  float GetAccelCostByThreePoints(const STPoint& first, const STPoint& second,
                                   const STPoint& third);

  float GetJerkCostByTwoPoints(const float pre_speed, const float pre_acc,
                                const STPoint& pre_point,
                                const STPoint& curr_point);
  float GetJerkCostByThreePoints(const float first_speed,
                                  const STPoint& first_point,
                                  const STPoint& second_point,
                                  const STPoint& third_point);

  float GetJerkCostByFourPoints(const STPoint& first, const STPoint& second,
                                 const STPoint& third, const STPoint& fourth);

 private:
  float GetAccelCost(const float accel);                                                      // 给定一个浮点数获得代价函数值
  float JerkCost(const float jerk);

  void AddToKeepClearRange(const std::vector<const PathObstacle*>& obstacles);                // 保持keep clear的范围
  static void SortAndMergeRange(
      std::vector<std::pair<float, float>>* keep_clear_range_);                               // 排序并合并范围
  bool InKeepClearRange(float s) const;                                                       // 是否在keep clear这个范围内

  const DpStSpeedConfig& config_;                                                             // 配置参数项
  const std::vector<const PathObstacle*>& obstacles_;                                         // 障碍物的数组
  const common::TrajectoryPoint& init_point_;                                                 // 轨迹的起点

  float unit_t_ = 0.0;                                                                        // 时间的分辨率

  std::unordered_map<std::string, int> boundary_map_;                                         // 用一个hash表做边界的映射
  std::vector<std::vector<std::pair<float, float>>> boundary_cost_;                           // boundary的代价值

  std::vector<std::pair<float, float>> keep_clear_range_;                                     // keep clear的点对

  std::array<float, 200> accel_cost_;                                                         // 200个浮点型的数组(用于保存加速度)
  std::array<float, 400> jerk_cost_;                                                          // 400个浮点型的数组(用于保存加速度的微分)
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_COST_H_
