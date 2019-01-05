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

#include "modules/planning/common/trajectory/trajectory_stitcher.h"

#include <algorithm>
#include <list>
#include <utility>

#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;     // 包括了path等信息
using apollo::common::VehicleState;        // 车辆状态
using apollo::common::math::Vec2d;         // 二维向量
using apollo::common::util::DistanceXY;    // 两点之间的距离, 用的模板实现, 模板要有x(), y()的成员函数

std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const VehicleState& vehicle_state) {
  TrajectoryPoint init_point;
  init_point.mutable_path_point()->set_s(0.0);
  init_point.mutable_path_point()->set_x(vehicle_state.x());
  init_point.mutable_path_point()->set_y(vehicle_state.y());
  init_point.mutable_path_point()->set_z(vehicle_state.z());
  init_point.mutable_path_point()->set_theta(vehicle_state.heading());
  init_point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  init_point.set_v(vehicle_state.linear_velocity());
  init_point.set_a(vehicle_state.linear_acceleration());
  init_point.set_relative_time(0.0);

  return std::vector<TrajectoryPoint>(1, init_point);  // 重新初始化, 轨迹
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* prev_trajectory) {    // 在巡航模式中使用
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);
  // 迭代其中的每个元素都执行 一个旋转矩阵的操作
  std::for_each(prev_trajectory->trajectory_points().begin(),
                prev_trajectory->trajectory_points().end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](common::TrajectoryPoint& p) {
                  auto x = p.path_point().x();
                  auto y = p.path_point().y();
                  auto theta = p.path_point().theta();

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new = common::math::WrapAngle(theta - theta_diff);

                  p.mutable_path_point()->set_x(x_new);
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}

// Planning from current vehicle state: enable_trajectory_stitcher
// if 1. the auto-driving mode is off or
//    2. we don't have the trajectory from last planning cycle or
//    3. the position deviation from actual and target is too high
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time,
    const PublishableTrajectory* prev_trajectory) {
  if (!FLAGS_enable_trajectory_stitcher) {   // 没有开启自动驾驶就不停的重新初始化
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  if (!prev_trajectory) {   // 上一个trajectory不存在, 重新初始化
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  // 不是自动驾驶模式
  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  // 之前有多少点
  std::size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();  // 时间戳

  std::size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time); // 小于当前时间戳的第一个点
  // 没有匹配的点
  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  if (time_matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  auto time_matched_point =
      prev_trajectory->TrajectoryPointAt(time_matched_index); // 取出对应的点

  if (!time_matched_point.has_path_point()) {  // 在prev中不存在, 这个检查感觉有点多余吧?
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  std::size_t position_matched_index = prev_trajectory->QueryNearestPoint(
      {vehicle_state.x(), vehicle_state.y()});  // 与车位置最相近的点

  auto frenet_sd = ComputePositionProjection(   // 计算车的位置投影, 投影到frenet坐标系
      vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(position_matched_index));

  auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first; // 纵向误差
  auto lat_diff = frenet_sd.second;   // 横向误差

  ADEBUG << "Control lateral diff: " << lat_diff     // 横纵误差值为5
         << ", longitudinal diff: " << lon_diff;

  if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold ||   
      std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {   // 在允许误差范围内 
    AERROR << "the distance between matched point and actual position is too "
              "large. Replan is triggered. lat_diff = "
           << lat_diff << ", lon_diff = " << lon_diff;
    return ComputeReinitStitchingTrajectory(vehicle_state); // 超过误差重新初始化
  }

  double forward_rel_time =
      prev_trajectory->TrajectoryPointAt(time_matched_index).relative_time() +
      planning_cycle_time;    // planning循环时间

  std::size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);   // 找到该时间间隔最近的元素

  ADEBUG << "Position matched index: " << position_matched_index;
  ADEBUG << "Time matched index: " << time_matched_index;

  auto matched_index = std::min(time_matched_index, position_matched_index); // 取两者最小的, 按时间最佳匹配和空间上最佳匹配
  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->trajectory_points().begin() +
          std::max(0, static_cast<int>(matched_index - 1)),   // 必须要强制转换
      prev_trajectory->trajectory_points().begin() + forward_time_index + 1);  // 至少会得到有两个元素的数组

  const double zero_s = stitching_trajectory.back().path_point().s(); // 初始值长度
  for (auto& tp : stitching_trajectory) {   // trajectory里面必须要有path
    if (!tp.has_path_point()) {
      return ComputeReinitStitchingTrajectory(vehicle_state);
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);  // 已经走过的路径
  }
  return stitching_trajectory;
}
// 将trajectory进行坐标投影
std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const TrajectoryPoint& p) {
  Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  Vec2d n(common::math::cos(
              common::math::Angle16::from_rad(p.path_point().theta())),
          common::math::sin(
              common::math::Angle16::from_rad(p.path_point().theta())));

  std::pair<double, double> frenet_sd;   // 初始化一个空的点对
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();
  frenet_sd.second = v.CrossProd(n);
  return frenet_sd;
}

}  // namespace planning
}  // namespace apollo
