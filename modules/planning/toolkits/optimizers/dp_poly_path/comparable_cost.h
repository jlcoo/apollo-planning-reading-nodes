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

#ifndef MODULES_PLANNING_TASKS_DP_POLY_PATH_COMPARABLE_COST_H_
#define MODULES_PLANNING_TASKS_DP_POLY_PATH_COMPARABLE_COST_H_

#include <cmath>
#include <cstdlib>

#include <array>
#include <vector>

namespace apollo {
namespace planning {
// 代价函数封装成一个类
class ComparableCost {                                                          // 可比较的代价函数, 就是重载了很多比较操作符
 public:
  ComparableCost() = default;                                                   // 默认的构造函数
  ComparableCost(const bool has_collision, const bool out_of_boundary,          // 是否有碰撞, 是否超出了边界
                 const bool out_of_lane, const float safety_cost_,              // 是否超出了lane(车道), 安全代价的值
                 const float smoothness_cost_)                                  // 平滑代价的值                       
      : safety_cost(safety_cost_), smoothness_cost(smoothness_cost_) {          // 初始化列表一部分数据
    cost_items[HAS_COLLISION] = has_collision;                                  // 3种不同的cost类型, 碰撞类型
    cost_items[OUT_OF_BOUNDARY] = out_of_boundary;                              // 超出边框类型
    cost_items[OUT_OF_LANE] = out_of_lane;                                      // 超出车道的类型
  }
  ComparableCost(const ComparableCost &) = default;                             // copy构造函数  
  // 和另一个可比较的代价函数进行比较
  int CompareTo(const ComparableCost &other) const {                            // 深度拷贝
    for (size_t i = 0; i < cost_items.size(); ++i) {                            // 迭代三种类型, cost_items是一个大小为3的bool类型的数组
      if (cost_items[i]) {                                                      // 如果考虑了该类型的cost函数值
        if (other.cost_items[i]) {                                              // 另个可比较的代价函数都考虑了某种类型的代价函数值
          continue;                                                             // 就继续考虑下一种类型
        } else {                                                                // 如果该ComparableCost比其他的先考虑了这种值, 就返回1
          return 1;
        }
      } else {                                                                  // 如果其他的cost值考虑得多的话，就返回-1
        if (other.cost_items[i]) {
          return -1;
        } else {                                                                // 两种都没有考虑的话, 继续考虑下一个cost的值
          continue;
        }
      }
    }

    constexpr float kEpsilon = 1e-12;                                           // 无限小量
    const float diff = safety_cost + smoothness_cost - other.safety_cost -      // 当前ComparableCost的安全代价和平滑代价之和减去其他cost对象的安全和平滑代价之和
                       other.smoothness_cost;
    if (std::fabs(diff) < kEpsilon) {                                           // 如果两个的差特别接近， 就返回0
      return 0;
    } else if (diff > 0) {                                                      // 如果当前的cost对象的值更大就返回1
      return 1;
    } else {                                                                    // 如果更小就返回-1
      return -1;
    }
  }                                                                             // 重载各种操作符(加+， 加等于+=， 大于>, 大于等于>=, 小于<, 小于等于<=)
  ComparableCost operator+(const ComparableCost &other) {                       // 都是和另一个对象进行比较
    ComparableCost lhs = *this;
    lhs += other;
    return lhs;
  }
  ComparableCost &operator+=(const ComparableCost &other) {
    for (size_t i = 0; i < cost_items.size(); ++i) {                    
      cost_items[i] = (cost_items[i] || other.cost_items[i]);
    }
    safety_cost += other.safety_cost;
    smoothness_cost += other.smoothness_cost;
    return *this;
  }
  bool operator>(const ComparableCost &other) const {
    return this->CompareTo(other) > 0;
  }
  bool operator>=(const ComparableCost &other) const {
    return this->CompareTo(other) >= 0;
  }
  bool operator<(const ComparableCost &other) const {
    return this->CompareTo(other) < 0;
  }
  bool operator<=(const ComparableCost &other) const {
    return this->CompareTo(other) <= 0;
  }
  /*
   * cost_items represents an array of factors that affect the cost,
   * The level is from most critical to less critical.
   * It includes:
   * (0) has_collision or out_of_boundary
   * (1) out_of_lane
   *
   * NOTICE: Items could have same critical levels
   */
  static const size_t HAS_COLLISION = 0;                                      // 静态的const全局变量， 其实用枚举值是不是更容易接受
  static const size_t OUT_OF_BOUNDARY = 1;                                    // 超出边界
  static const size_t OUT_OF_LANE = 2;                                        // 超出车道
  std::array<bool, 3> cost_items = {{false, false, false}};                   // 代价函数的条目, 目前有三个代价函数(是否碰撞, 是否超出了边界, 是否超出了车道)
  // 在dp_path中有两种不同的代价
  // cost from distance to obstacles or boundaries
  float safety_cost = 0.0f;                                                   // 到障碍物或者是边界的距离 成本代价
  // cost from deviation from lane center, path curvature etc
  float smoothness_cost = 0.0f;                                               // 偏离车道中心线或者是车道曲率的 成本代价
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_DP_POLY_PATH_COMPARABLE_COST_H_
