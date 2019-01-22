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

#ifndef MODULES_PLANNING_TASKS_POLY_ST_SPEED_SPEED_PROFILE_COST_H_
#define MODULES_PLANNING_TASKS_POLY_ST_SPEED_SPEED_PROFILE_COST_H_

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/poly_st_speed_config.pb.h"

#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/math/curve1d/quartic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

class SpeedProfileCost {                                                                  // 速度曲线的代价函数类
 public:
  explicit SpeedProfileCost(const PolyStSpeedConfig &config,                              // 禁止隐式转换的构造函数
                            const std::vector<const PathObstacle *> &obstacles,
                            const SpeedLimit &speed_limit,
                            const common::TrajectoryPoint &init_point);

  double Calculate(const QuarticPolynomialCurve1d &curve, const double end_time,          // speed 曲线计算cost的函数, 5次多项式曲线
                   const double curr_min_cost) const;

 private:
  double CalculatePointCost(const QuarticPolynomialCurve1d &curve,                        // 计算一个点的代价函数值
                            const double t) const;

  const PolyStSpeedConfig config_;                                                        // st多项式的配置项的配置参数
  const std::vector<const PathObstacle *> &obstacles_;                                    // path上的障碍物
  const SpeedLimit &speed_limit_;                                                         // 速度的限制
  const common::TrajectoryPoint &init_point_;                                             // 轨迹的起点
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_POLY_ST_SPEED_SPEED_PROFILE_COST_H_
