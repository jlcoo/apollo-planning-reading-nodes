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

using common::TrajectoryPoint;  // 轨迹的点

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

TrajectoryPoint DiscretizedTrajectory::Evaluate(
    const double relative_time) const {
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
  return common::math::InterpolateUsingLinearApproximation(
      *(it_lower - 1), *it_lower, relative_time);
}

std::uint32_t DiscretizedTrajectory::QueryLowerBoundPoint(
    const double relative_time) const {
  CHECK(!trajectory_points_.empty());
  // 时间间隔比我还要长
  if (relative_time >= trajectory_points_.back().relative_time()) {
    return trajectory_points_.size() - 1;
  }
  auto func = [](const TrajectoryPoint& tp, const double relative_time) {
    return tp.relative_time() < relative_time;
  };
  auto it_lower =
      std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(),
                       relative_time, func);
  return std::distance(trajectory_points_.begin(), it_lower);  // 迭代器的距离
}

std::uint32_t DiscretizedTrajectory::QueryNearestPoint(
    const common::math::Vec2d& position) const {
  double dist_sqr_min = std::numeric_limits<double>::max();  // 最大值
  std::uint32_t index_min = 0;
  for (std::uint32_t i = 0; i < trajectory_points_.size(); ++i) {  // 迭代所有的trajectory_points点
    const common::math::Vec2d curr_point(
        trajectory_points_[i].path_point().x(),
        trajectory_points_[i].path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position); // 得到两点距离的平方
    if (dist_sqr < dist_sqr_min) {
      dist_sqr_min = dist_sqr;   // 保存一个最近的点
      index_min = i;
    }
  }
  return index_min;
}

void DiscretizedTrajectory::AppendTrajectoryPoint(
    const TrajectoryPoint& trajectory_point) {
  if (!trajectory_points_.empty()) {
    CHECK_GT(trajectory_point.relative_time(),
             trajectory_points_.back().relative_time());
  }
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
