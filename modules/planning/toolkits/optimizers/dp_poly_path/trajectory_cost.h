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

#ifndef MODULES_PLANNING_TASKS_DP_POLY_PATH_TRAJECTORY_COST_H_
#define MODULES_PLANNING_TASKS_DP_POLY_PATH_TRAJECTORY_COST_H_

#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/dp_poly_path_config.pb.h"

#include "modules/common/math/box2d.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/toolkits/optimizers/dp_poly_path/comparable_cost.h"

namespace apollo {
namespace planning {
                                                                                      // 将轨迹的代价封装成一个类
class TrajectoryCost {
 public:
  TrajectoryCost() = default;                                                         // 默认的构造函数
  explicit TrajectoryCost(const DpPolyPathConfig &config,                             // dp路径的配置项
                          const ReferenceLine &reference_line,                        // 道路中心线, 参考线
                          const bool is_change_lane_path,                             // 是否变道
                          const std::vector<const PathObstacle *> &obstacles,         // path道路上的所有障碍物
                          const common::VehicleParam &vehicle_param,                  // 车辆状态的参数
                          const SpeedData &heuristic_speed_data,                      // 启发式的速度参数, 速度相关的数据
                          const common::SLPoint &init_sl_point);                      // 初始位置
  ComparableCost Calculate(const QuinticPolynomialCurve1d &curve,                     // 通过一个一维的5次曲线
                           const float start_s, const float end_s,                    // s方向的起点和终点
                           const uint32_t curr_level,                                 // 当前的水平(level)
                           const uint32_t total_level);                               // 总共的level

 private:
  ComparableCost CalculatePathCost(const QuinticPolynomialCurve1d &curve,             // 计算path的代价 
                                   const float start_s, const float end_s,
                                   const uint32_t curr_level,
                                   const uint32_t total_level);
  ComparableCost CalculateStaticObstacleCost(                                         // 计算静态障碍物的代价
      const QuinticPolynomialCurve1d &curve, const float start_s,
      const float end_s);
  ComparableCost CalculateDynamicObstacleCost(                                        // 计算动态障碍物的代价
      const QuinticPolynomialCurve1d &curve, const float start_s,
      const float end_s) const;
  ComparableCost GetCostBetweenObsBoxes(                                              // 在两个障碍物之间获得代价
      const common::math::Box2d &ego_box,
      const common::math::Box2d &obstacle_box) const;

  FRIEND_TEST(AllTrajectoryTests, GetCostFromObsSL);                                  // 测试
  ComparableCost GetCostFromObsSL(const float adc_s, const float adc_l,               // 获取自动驾驶车辆离sl边框的代价
                                  const SLBoundary &obs_sl_boundary);

  common::math::Box2d GetBoxFromSLPoint(const common::SLPoint &sl,                    // 从一个sl坐标系下的点获取一个二维的box
                                        const float dl) const;

  const DpPolyPathConfig config_;                                                     // dp的配置参数
  const ReferenceLine *reference_line_ = nullptr;                                     // 中心参考线的指针
  bool is_change_lane_path_ = false;                                                  // 是否已经改变车道
  const common::VehicleParam vehicle_param_;                                          // 车辆的参数
  SpeedData heuristic_speed_data_;                                                    // 启发式的速度点
  const common::SLPoint init_sl_point_;                                               // 初始点在sl坐标系下的坐标
  uint32_t num_of_time_stamps_ = 0;                                                   // 时间戳的个数
  std::vector<std::vector<common::math::Box2d>> dynamic_obstacle_boxes_;              // 动态障碍物的box
  std::vector<float> obstacle_probabilities_;                                         // 是障碍物的概率

  std::vector<SLBoundary> static_obstacle_sl_boundaries_;                             // 静态障碍物的边界
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_DP_POLY_PATH_TRAJECTORY_COST_H_
