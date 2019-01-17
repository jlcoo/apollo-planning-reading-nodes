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
 * @file discretized_trajectory.cc
 **/

#include "modules/planning/common/trajectory/discretized_trajectory.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {

using common::TrajectoryPoint;  // 轨迹的点, 包含了速度, 加速度,曲率, 三维坐标等信息

DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<TrajectoryPoint>& trajectory_points) {
  CHECK(!trajectory_points.empty())
      << "trajectory_points should NOT be empty()";
  trajectory_points_ = trajectory_points;
}

DiscretizedTrajectory::DiscretizedTrajectory(const ADCTrajectory& trajectory) {
  trajectory_points_.assign(trajectory.trajectory_point().begin(),  // assign赋值不会产生临时变量
                            trajectory.trajectory_point().end());
}

TrajectoryPoint DiscretizedTrajectory::Evaluate(                                    // 通过一个相对的时间进行评估一个
    const double relative_time) const {                                             // 找到小于该时间的第一个点
  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };    // 比较函数

  auto it_lower =
      std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(),
                       relative_time, comp);  // 小于当前时间的第一项

  if (it_lower == trajectory_points_.begin()) {
    return trajectory_points_.front();
  } else if (it_lower == trajectory_points_.end()) {
    AWARN << "When evaluate trajectory, relative_time(" << relative_time
          << ") is too large";
    return trajectory_points_.back();   // 时间戳太长
  }
  return common::math::InterpolateUsingLinearApproximation(                         // 在相对时间的点上生成一个新的点
      *(it_lower - 1), *it_lower, relative_time);
}

std::uint32_t DiscretizedTrajectory::QueryLowerBoundPoint(                          // 通过一个时间点找到该时间点的下界
    const double relative_time) const {
  CHECK(!trajectory_points_.empty());                                               // 检查离散的轨迹点
  // 时间间隔比我还要长
  if (relative_time >= trajectory_points_.back().relative_time()) {                 // 要索引的时间点特别长， 直接返回当前这段轨迹的最后面的点
    return trajectory_points_.size() - 1;
  }
  auto func = [](const TrajectoryPoint& tp, const double relative_time) {           // 轨迹点的时间比相对时间点还小
    return tp.relative_time() < relative_time;                                      // lamda的函数对象
  };
  auto it_lower =                                                                   // std::lower_bound会返回第一个大于或等于目标值的迭代器
      std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(),
                       relative_time, func);
  return std::distance(trajectory_points_.begin(), it_lower);                      // 返回迭代器离起点的距离， 第几个跌带器(即索引值)
}

std::uint32_t DiscretizedTrajectory::QueryNearestPoint(
    const common::math::Vec2d& position) const {                                   // 二维向量的一个点
  double dist_sqr_min = std::numeric_limits<double>::max();                        // 最大值
  std::uint32_t index_min = 0;                                                     // 最小的索引值, 32位的无符号值
  for (std::uint32_t i = 0; i < trajectory_points_.size(); ++i) {                  // 迭代所有的trajectory_points点
    const common::math::Vec2d curr_point(
        trajectory_points_[i].path_point().x(),
        trajectory_points_[i].path_point().y());                                   // 迭代一次就会生成一个一个二维的点

    const double dist_sqr = curr_point.DistanceSquareTo(position);                 // 得到两点距离的平方, 给定的点到当前点的距离
    if (dist_sqr < dist_sqr_min) {                                                 // 迭代的过程中找到最小的距离的平方和
      dist_sqr_min = dist_sqr;   // 保存一个最近的点
      index_min = i;                                                               // 保存对应的索引值
    }
  }      // index_min对应的就是最近轨迹点的索引
  return index_min;                                                                // 返回轨迹点中离给定点 距离最小的索引值
}

void DiscretizedTrajectory::AppendTrajectoryPoint(
    const TrajectoryPoint& trajectory_point) {                                     // 向轨迹点集中添加一个点
  if (!trajectory_points_.empty()) {
    CHECK_GT(trajectory_point.relative_time(),                                     // 先做安全性检查
             trajectory_points_.back().relative_time());
  }                                                                                // 再将这个轨迹点放到数组的最后面
  trajectory_points_.push_back(trajectory_point);  // 向轨迹数组中增加一个点
}

const TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(
    const std::uint32_t index) const {
  CHECK_LT(index, NumOfPoints());
  return trajectory_points_[index];  // 直接索引这个点
}

TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  CHECK(!trajectory_points_.empty());
  return trajectory_points_.front();   // 返回起点
}

double DiscretizedTrajectory::GetTemporalLength() const {
  CHECK(!trajectory_points_.empty());  // 返回时间的长度, 其实是相对时间
  return trajectory_points_.back().relative_time() -
         trajectory_points_.front().relative_time();
}

double DiscretizedTrajectory::GetSpatialLength() const {
  CHECK(!trajectory_points_.empty());  // 返回空间上的长度, 其实是path的长度
  return trajectory_points_.back().path_point().s() -
         trajectory_points_.front().path_point().s();
}

}  // namespace planning
}  // namespace apollo
