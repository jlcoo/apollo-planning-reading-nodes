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
 * @file path.cc
 **/

#include "modules/planning/common/path/discretized_path.h"

#include <algorithm>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {

DiscretizedPath::DiscretizedPath(
    const std::vector<common::PathPoint> &path_points) {
  path_points_ = path_points;                                                           // 这里面会进行深度拷贝, 会把数组中的内容一一拷贝过去
}

void DiscretizedPath::set_path_points(
    const std::vector<common::PathPoint> &path_points) {
  path_points_ = path_points;
}

double DiscretizedPath::Length() const {
  if (path_points_.empty()) {     // path_points_是一个vector
    return 0.0;
  }                                                                                     // 最前面的一个点减去最后面的一个点
  return path_points_.back().s() - path_points_.front().s();  // 离散点作差
}

common::PathPoint DiscretizedPath::Evaluate(const double path_s) const {
  CHECK(!path_points_.empty());
  auto it_lower = QueryLowerBound(path_s);                                              // 找到s点的下界
  if (it_lower == path_points_.begin()) {  // 在vector的两边?
    return path_points_.front();
  }
  if (it_lower == path_points_.end()) {    
    return path_points_.back();
  }                                        // 进行线性逼近插值
  return common::math::InterpolateUsingLinearApproximation(*(it_lower - 1),             // 然后进行线性插值
                                                           *it_lower, path_s);
}

const std::vector<common::PathPoint> &DiscretizedPath::path_points() const {            // 直接返回离散的路径点
  return path_points_;
}

std::uint32_t DiscretizedPath::NumOfPoints() const {                                    // 返回数组的大小
  return path_points_.size();
}
// 离散的意思就是记录了一个pathPoint结构体封装
const common::PathPoint &DiscretizedPath::StartPoint() const {                          // 数组的起点
  CHECK(!path_points_.empty());                                                         // 容错处理
  return path_points_.front();                                                          // 返回数组的头
}

const common::PathPoint &DiscretizedPath::EndPoint() const {
  CHECK(!path_points_.empty());
  return path_points_.back();                                                           // 返回数组的尾
}

void DiscretizedPath::Clear() { path_points_.clear(); }                                 // 清空数组
// 查询第一个path_s的迭代器
std::vector<common::PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
    const double path_s) const {
  auto func = [](const common::PathPoint &tp, const double path_s) {                    // 定义一个函数对象, 用的是lamda表达式
    return tp.s() < path_s;
  }; // lambda函数对象
  // lower_bound返回一个迭代器，指向键值>= key的第一个元素。
  return std::lower_bound(path_points_.begin(), path_points_.end(), path_s,
                          func);
}

}  // namespace planning
}  // namespace apollo
