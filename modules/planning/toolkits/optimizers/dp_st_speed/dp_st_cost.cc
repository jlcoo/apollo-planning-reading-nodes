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

#include "modules/planning/toolkits/optimizers/dp_st_speed/dp_st_cost.h"

#include <algorithm>
#include <limits>

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/st_point.h"

namespace apollo {
namespace planning {
namespace {
constexpr float kInf = std::numeric_limits<float>::infinity();                            // 无穷大
}

DpStCost::DpStCost(const DpStSpeedConfig& config,
                   const std::vector<const PathObstacle*>& obstacles,
                   const common::TrajectoryPoint& init_point)
    : config_(config), obstacles_(obstacles), init_point_(init_point) {                   // 初始化列表
  int index = 0;
  for (auto& obstacle : obstacles) {
    boundary_map_[obstacle->st_boundary().id()] = index++;                                // 索引值加1
  }
  unit_t_ = config_.total_time() / config_.matrix_dimension_t();                          // 总时间是7秒

  AddToKeepClearRange(obstacles);                                                         // 将障碍物加到keep clear区域中

  boundary_cost_.resize(obstacles_.size());                                               // 重新计算
  for (auto& vec : boundary_cost_) {
    vec.resize(config_.matrix_dimension_t(), std::make_pair(-1.0, -1.0));
  }
  accel_cost_.fill(-1.0);
  jerk_cost_.fill(-1.0);
}

void DpStCost::AddToKeepClearRange(
    const std::vector<const PathObstacle*>& obstacles) {
  for (const auto& obstacle : obstacles) {                                                // 迭代所欲的障碍物
    if (obstacle->st_boundary().IsEmpty()) {
      continue;
    }
    if (obstacle->st_boundary().boundary_type() !=
        StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    float start_s = obstacle->st_boundary().min_s();                                      // 获得每个障碍物的起点和终点
    float end_s = obstacle->st_boundary().max_s();
    keep_clear_range_.emplace_back(start_s, end_s);
  }
  SortAndMergeRange(&keep_clear_range_);                                                  // 排序并合并范围
}

void DpStCost::SortAndMergeRange(
    std::vector<std::pair<float, float>>* keep_clear_range) {                             // 点对的数组
  if (!keep_clear_range || keep_clear_range->empty()) {
    return;
  }                                                                                       // 默认是从第一个元素从小到大进行排序的
  std::sort(keep_clear_range->begin(), keep_clear_range->end());                          // 点对的排序, 按起点进行排序
  std::size_t i = 0;
  std::size_t j = i + 1;
  while (j < keep_clear_range->size()) {                                                  // 不断迭代
    if (keep_clear_range->at(i).second < keep_clear_range->at(j).first) {
      ++i;
      ++j;
    } else {
      keep_clear_range->at(i).second = std::max(keep_clear_range->at(i).second,           // 障碍物的终点最大的点
                                                keep_clear_range->at(j).second);
      ++j;
    }
  }
  keep_clear_range->resize(i + 1);                                                        // 只保存前面几个障碍物
}

bool DpStCost::InKeepClearRange(float s) const {                                          // 判断是否在keep clear的范围内
  if (keep_clear_range_.empty()) {
    return false;
  }
  for (const auto& p : keep_clear_range_) {                                               // 全部进行迭代
    if (p.first <= s && p.second >= s) {
      return true;
    }
  }
  return false;
}

float DpStCost::GetObstacleCost(const StGraphPoint& st_graph_point) {                     // 获得障碍物的代价函数值
  const float s = st_graph_point.point().s();
  const float t = st_graph_point.point().t();

  float cost = 0.0;
  for (const auto* obstacle : obstacles_) {
    if (!obstacle->IsBlockingObstacle()) {
      continue;
    }

    auto boundary = obstacle->st_boundary();
    const float kIgnoreDistance = 200.0;                                                  // 200米开外的的障碍物不用考虑
    if (boundary.min_s() > kIgnoreDistance) {
      continue;
    }
    if (t < boundary.min_t() || t > boundary.max_t()) {                                   // 超出boundary的范围
      continue;
    }
    if (obstacle->IsBlockingObstacle() &&                                                 // 障碍物是堵塞的障碍物或者是在起点就是返回无穷大
        boundary.IsPointInBoundary(STPoint(s, t))) {
      return kInf;
    }
    double s_upper = 0.0;
    double s_lower = 0.0;

    int boundary_index = boundary_map_[boundary.id()];                                    // 获得索引值
    if (boundary_cost_[boundary_index][st_graph_point.index_t()].first < 0.0) {           // 上界太小
      boundary.GetBoundarySRange(t, &s_upper, &s_lower);
      boundary_cost_[boundary_index][st_graph_point.index_t()] =
          std::make_pair(s_upper, s_lower);
    } else {
      s_upper = boundary_cost_[boundary_index][st_graph_point.index_t()].first;           // 获取上界和下界
      s_lower = boundary_cost_[boundary_index][st_graph_point.index_t()].second;
    }
    if (s < s_lower) {                                                                    // st图中的s比下界还要小
      constexpr float kSafeTimeBuffer = 3.0;                                              // 安全时间为3秒
      const float len = obstacle->obstacle()->Speed() * kSafeTimeBuffer;                  // 障碍物的速度乘以安全的时间
      if (s + len < s_lower) {                                                            // 安全距离能够通过, 就继续其他的障碍物
        continue;
      } else {                                                                            // 否则计算障碍物的代价函数值
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                std::pow((len - s_lower + s), 2);
      }
    } else if (s > s_upper) {                                                             // 如果障碍物已经在自动驾驶车辆的后面
      const float kSafeDistance = 20.0;  // or calculated from velocity                   // 保持20米的行车距离
      if (s > s_upper + kSafeDistance) {                                                  // 如果安全的话就继续行驶
        continue;
      } else {
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *             // 如果太近的话就计算代价函数值
                std::pow((kSafeDistance + s_upper - s), 2);
      }
    }
  }
  return cost * unit_t_;                                                                  // 获得单位时间对应的代价值
}

float DpStCost::GetReferenceCost(const STPoint& point,
                                 const STPoint& reference_point) const {                  // 参考线的代价值
  return config_.reference_weight() * (point.s() - reference_point.s()) *                 // 配置项中都会有权重
         (point.s() - reference_point.s()) * unit_t_;
}

float DpStCost::GetSpeedCost(const STPoint& first, const STPoint& second,
                             const float speed_limit) const {                             // 速度的代价函数值
  float cost = 0.0;
  const float speed = (second.s() - first.s()) / unit_t_;                                 // 通过两个点的距离，估计一个速度出来
  if (speed < 0) {
    return kInf;
  }

  if (speed < FLAGS_max_stop_speed && InKeepClearRange(second.s())) {                     // 最大的停止速度FLAGS_max_stop_speed设置为0.2
    // first.s in range
    cost += config_.keep_clear_low_speed_penalty() * unit_t_ *                            // 低速的惩罚项keep_clear_low_speed_penalty默认是10
            config_.default_speed_cost();
  }

  float det_speed = (speed - speed_limit) / speed_limit;                                  // 速度的增量
  if (det_speed > 0) {                                                                    // 超速了， 比限速的还要快
    cost += config_.exceed_speed_penalty() * config_.default_speed_cost() *
            fabs(speed * speed) * unit_t_;
  } else if (det_speed < 0) {                                                             // 正常行驶
    cost += config_.low_speed_penalty() * config_.default_speed_cost() *
            -det_speed * unit_t_;
  }
  return cost;
}

float DpStCost::GetAccelCost(const float accel) {                                         // 计算加速度的代价
  float cost = 0.0;
  constexpr float kEpsilon = 0.1;                                                         // 无穷小量
  constexpr size_t kShift = 100;                                                          // 100个速度点？
  const size_t accel_key = static_cast<size_t>(accel / kEpsilon + 0.5 + kShift);          // 加速度中的点, 这是要移窗么?
  DCHECK_LT(accel_key, accel_cost_.size());
  if (accel_key >= accel_cost_.size()) {                                                  // 移窗过大
    return kInf;
  }

  if (accel_cost_.at(accel_key) < 0.0) {                                                  // 代价值小于0
    const float accel_sq = accel * accel;
    float max_acc = config_.max_acceleration();
    float max_dec = config_.max_deceleration();
    float accel_penalty = config_.accel_penalty();                                          
    float decel_penalty = config_.decel_penalty();

    if (accel > 0.0) {                                                                    // 加速的cost
      cost = accel_penalty * accel_sq;
    } else {
      cost = decel_penalty * accel_sq;                                                    // 减速的cost
    }
    cost += accel_sq * decel_penalty * decel_penalty /                                    // 总代价函数值
                (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty /
                (1 + std::exp(-1.0 * (accel - max_acc)));
    accel_cost_.at(accel_key) = cost;
  } else {
    cost = accel_cost_.at(accel_key);                                                     // 取出shift后的加速度代价函数值
  }
  return cost * unit_t_;
}

float DpStCost::GetAccelCostByThreePoints(const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third) {
  float accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);          // 三个点计算出一个加速度, 再获取对应的代价值
  return GetAccelCost(accel);
}

float DpStCost::GetAccelCostByTwoPoints(const float pre_speed,
                                        const STPoint& pre_point,
                                        const STPoint& curr_point) {                     // 获得两个点之间的加速度的代价函数值
  float current_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  float accel = (current_speed - pre_speed) / unit_t_;
  return GetAccelCost(accel);
}

float DpStCost::JerkCost(const float jerk) {                                             // jerk的代价函数
  float cost = 0.0;
  constexpr float kEpsilon = 0.1;
  constexpr size_t kShift = 200;
  const size_t jerk_key = static_cast<size_t>(jerk / kEpsilon + 0.5 + kShift);
  if (jerk_key >= jerk_cost_.size()) {
    return kInf;
  }

  if (jerk_cost_.at(jerk_key) < 0.0) {
    float jerk_sq = jerk * jerk;
    if (jerk > 0) {
      cost = config_.positive_jerk_coeff() * jerk_sq * unit_t_;                         // 设置参数里面有一个正的jerk的参数
    } else {                                                                            // 这个else有点多余吧
      cost = config_.negative_jerk_coeff() * jerk_sq * unit_t_;
    }
    jerk_cost_.at(jerk_key) = cost;
  } else {
    cost = jerk_cost_.at(jerk_key);
  }

  // TODO(All): normalize to unit_t_
  return cost;
}

float DpStCost::GetJerkCostByFourPoints(const STPoint& first,
                                        const STPoint& second,
                                        const STPoint& third,
                                        const STPoint& fourth) {                       // 通过四个点计算
  float jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
               (unit_t_ * unit_t_ * unit_t_);
  return JerkCost(jerk);
}

float DpStCost::GetJerkCostByTwoPoints(const float pre_speed,
                                       const float pre_acc,
                                       const STPoint& pre_point,
                                       const STPoint& curr_point) {                    // 通过两个点进行计算
  const float curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  const float curr_accel = (curr_speed - pre_speed) / unit_t_;
  const float jerk = (curr_accel - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

float DpStCost::GetJerkCostByThreePoints(const float first_speed,
                                         const STPoint& first,
                                         const STPoint& second,
                                         const STPoint& third) {                       // 通过三个点进行计算
  const float pre_speed = (second.s() - first.s()) / unit_t_;
  const float pre_acc = (pre_speed - first_speed) / unit_t_;
  const float curr_speed = (third.s() - second.s()) / unit_t_;
  const float curr_acc = (curr_speed - pre_speed) / unit_t_;
  const float jerk = (curr_acc - pre_acc) / unit_t_;
  return JerkCost(jerk);                                                               // 都是先计算一个jerk函数值
}

}  // namespace planning
}  // namespace apollo
