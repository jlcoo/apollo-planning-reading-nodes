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
 * @file frenet_frame_path.cc Path
 **/
#include "modules/planning/common/path/frenet_frame_path.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/math/linear_interpolation.h"

namespace apollo {
namespace planning {

using apollo::common::FrenetFramePoint;                                             // sl坐标系中的一个点, sl坐标, l的一次和二次微分

FrenetFramePath::FrenetFramePath(
    const std::vector<FrenetFramePoint>& sl_points) {                               // 直接复制
  points_ = sl_points;
}

void FrenetFramePath::set_points(const std::vector<FrenetFramePoint>& points) {
  points_ = points;                                                                 // 深度拷贝
}

const std::vector<FrenetFramePoint>& FrenetFramePath::points() const {
  return points_;                                                                   // 返回frenet坐标系中的所有点集
}

double FrenetFramePath::Length() const {
  if (points_.empty()) {                                                            // 如果点集为空, 直接返回0
    return 0.0;
  }                                                                                 // 否则返回起点和终点的s方向的距离差值
  return points_.back().s() - points_.front().s();                                  // 放到的是一个vector
}

std::uint32_t FrenetFramePath::NumOfPoints() const { return points_.size(); }       // 返回数组中元素的个数

const FrenetFramePoint& FrenetFramePath::PointAt(
    const std::uint32_t index) const {                                              // 直接索引(index)这个点
  CHECK_LT(index, points_.size());                                                  // 做容错检测
  return points_[index];                                                            // 直接用[](中括号)进行索引
}

FrenetFramePoint FrenetFramePath::GetNearestPoint(const SLBoundary& sl) const {     // 通过一个sl的边框, 看是否能在frenet坐标系中找一个离它最近的点
  auto it_lower = std::lower_bound(points_.begin(), points_.end(), sl.start_s(),    // 小于sl边框s方向起点的下界
                                   LowerBoundComparator);                           // 长度在范围内
  if (it_lower == points_.end()) {                                                  // 如果这个sl boundary在sl path的正前方
    return points_.back();                                                          // 那离boundary最近的点就是最后的那个点
  }
  auto it_upper = std::upper_bound(it_lower, points_.end(), sl.end_s(),             // 获取sl边框s方向终点的上界
                                   UpperBoundComparator);  // 上界
  double min_dist = std::numeric_limits<double>::max();                             // 临时变量, 用来保存最小的距离
  auto min_it = it_upper;                                                           // 保存最小距离的迭代器
  for (auto it = it_lower; it != it_upper; ++it) {                                  // 轮寻sl boundary的s方向的所有点
    if (it->l() >= sl.start_l() && it->l() <= sl.end_l()) {                         // 如果path中的一个点的l恰好在这个boundary的内部, 那么这个点就是最近的点
      return *it;   // 获得最近的点
    } else if (it->l() > sl.end_l()) {                                              // 超出范围, it这个点在boundary的右侧
      double diff = it->l() - sl.end_l();
      if (diff < min_dist) {
        min_dist = diff;
        min_it = it;    // 找一个里end_l最近的点
      }
    } else {                                                                        // it 这个点在boundary的左侧
      double diff = sl.start_l() - it->l();    // 找离start_最近的点
      if (diff < min_dist) {
        min_dist = diff;
        min_it = it;
      }
    }
  }
  return *min_it;
}

FrenetFramePoint FrenetFramePath::EvaluateByS(const double s) const {              // 通过s点进行插值
  CHECK_GT(points_.size(), 1);                                                     // 进行容错检查
  auto it_lower =
      std::lower_bound(points_.begin(), points_.end(), s, LowerBoundComparator);   // 获得s点的下界
  if (it_lower == points_.begin()) {
    return points_.front();                                                        // 如果直接相等的话, 就不用进行插值了
  } else if (it_lower == points_.end()) {
    return points_.back();
  }
  const auto& p0 = *(it_lower - 1);
  const auto s0 = p0.s();
  const auto& p1 = *it_lower;
  const auto s1 = p1.s();

  FrenetFramePoint p;
  p.set_s(s);
  p.set_l(common::math::lerp(p0.l(), s0, p1.l(), s1, s));                          // 否则进行线性插值处理
  p.set_dl(common::math::lerp(p0.dl(), s0, p1.dl(), s1, s));
  p.set_ddl(common::math::lerp(p0.ddl(), s0, p1.ddl(), s1, s));
  return p;
}

void FrenetFramePath::Clear() { points_.clear(); }                                 // 清空数组中的所有的点

}  // namespace planning
}  // namespace apollo
