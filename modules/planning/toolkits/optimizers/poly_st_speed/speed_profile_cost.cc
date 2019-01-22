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

#include "modules/planning/toolkits/optimizers/poly_st_speed/speed_profile_cost.h"

#include <limits>

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace {
constexpr auto kInfCost = std::numeric_limits<double>::infinity();                              // 匿名的命名空间仅作用于当前的文件
constexpr double kEpsilon = 1e-6;
}  // namespace

using apollo::common::TrajectoryPoint;                                                          // 使用的是轨迹点

SpeedProfileCost::SpeedProfileCost(                                                             // 构造函数
    const PolyStSpeedConfig &config,
    const std::vector<const PathObstacle *> &obstacles,
    const SpeedLimit &speed_limit, const common::TrajectoryPoint &init_point)
    : config_(config),
      obstacles_(obstacles),
      speed_limit_(speed_limit),
      init_point_(init_point) {}                                                                // 初始化列表的构造函数

double SpeedProfileCost::Calculate(const QuarticPolynomialCurve1d &curve,
                                   const double end_time,
                                   const double curr_min_cost) const {                          // 计算代价函数的值
  double cost = 0.0;                                                                            // cost的临时函数值
  constexpr double kDeltaT = 0.5;                                                               // 时间上的间隔点
  for (double t = kDeltaT; t < end_time + kEpsilon; t += kDeltaT) {
    if (cost > curr_min_cost) {
      return cost;
    }
    cost += CalculatePointCost(curve, t);                                                       // 计算出一个点对应的代价函数值
  }
  return cost;
}

double SpeedProfileCost::CalculatePointCost(
    const QuarticPolynomialCurve1d &curve, const double t) const {                              // 计算一个时间点的代价值
  const double s = curve.Evaluate(0, t);
  const double v = curve.Evaluate(1, t);
  const double a = curve.Evaluate(2, t);
  const double da = curve.Evaluate(3, t);                                                       // 通过一个5次曲线进行计算

  if (s < 0.0) {                                                                                // s(路程)太小， 直接返回无穷大
    return kInfCost;
  }

  const double speed_limit = speed_limit_.GetSpeedLimitByS(s);                                  // 通过s获得对应点上的速度限制
  if (v < 0.0 || v > speed_limit * (1.0 + config_.speed_limit_buffer())) {                      // 速度超出范围
    return kInfCost;
  }
  if (a > config_.preferred_accel() || a < config_.preferred_decel()) {                         // 加速度超出范围
    return kInfCost;
  }

  double cost = 0.0;
  for (const auto *obstacle : obstacles_) {                                                     // 迭代所有的障碍物的
    auto boundary = obstacle->st_boundary();                                                    // 先获得障碍物的边框(boundary)
    const double kIgnoreDistance = 100.0;
    if (boundary.min_s() > kIgnoreDistance) {                                                   // 障碍物的下界在可以忽略的范围内, 就继续考虑其他障碍物
      continue;
    }
    if (t < boundary.min_t() || t > boundary.max_t()) {                                         // 超出了时间的范围
      continue;
    }
    if (obstacle->IsBlockingObstacle() &&
        boundary.IsPointInBoundary(STPoint(s, t))) {                                            // 堵塞的障碍物
      return kInfCost;
    }
    double s_upper = 0.0;
    double s_lower = 0.0;
    boundary.GetBoundarySRange(t, &s_upper, &s_lower);                                          // 求解t时刻的上下界
    if (s < s_lower) {                                                                          // 小于下界
      const double len = v * FLAGS_follow_time_buffer;
      if (s + len < s_lower) {                                                                  // 太小
        continue;
      } else {
        cost += config_.obstacle_weight() * std::pow((len - s_lower + s), 2);                   // 具体求法， 障碍物的权重乘以一个值的平方
      }
    } else if (s > s_upper) {
      const double kSafeDistance = 15.0;  // or calculated from velocity
      if (s > s_upper + kSafeDistance) {                                                        // 障碍物在安全范围内
        continue;
      } else {
        cost += config_.obstacle_weight() *
                std::pow((kSafeDistance + s_upper - s), 2);                                     // 计算障碍物的代价函数值
      }
    } else {
      if (!obstacle->IsBlockingObstacle()) {
        cost += config_.unblocking_obstacle_cost();                                             // 固定的代价值
      }
    }
  }
  cost += config_.speed_weight() * std::pow((v - speed_limit), 2);                              // 速度的权重
  cost += config_.jerk_weight() * std::pow(da, 2);                                              // 加速度的微分的权重
  ADEBUG << "t = " << t << ", s = " << s << ", v = " << v << ", a = " << a
         << ", da = " << da << ", cost = " << cost;                                       // 时间， 位移, 速度， 加速度， 加速度的一次微分， cost(代价函数值)

  return cost;
}

}  // namespace planning
}  // namespace apollo
