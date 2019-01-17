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
TrajectoryStitcher::ComputeReinitStitchingTrajectory(                                 // 通过合并后的轨迹进行重新初始化， 输入是车辆当前的状态
    const VehicleState& vehicle_state) {
  TrajectoryPoint init_point;                                                         // 新建一个初始化的轨迹点
  init_point.mutable_path_point()->set_s(0.0);                                        // 将积累的路程s设置为0
  init_point.mutable_path_point()->set_x(vehicle_state.x());                          // 设置初始点的x,y,z的坐标
  init_point.mutable_path_point()->set_y(vehicle_state.y());
  init_point.mutable_path_point()->set_z(vehicle_state.z());
  init_point.mutable_path_point()->set_theta(vehicle_state.heading());                // 航向角就是theta
  init_point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  init_point.set_v(vehicle_state.linear_velocity());                                  // 车辆的线性速度
  init_point.set_a(vehicle_state.linear_acceleration());                              // 车辆的线性加速度
  init_point.set_relative_time(0.0);                                                  // 设置相对时间(每次做合并后，路程和时间就会重新计算)

  return std::vector<TrajectoryPoint>(1, init_point);                                 // 重新初始化, 轨迹，(返回只有一个点的数组， 数组中的元素是TrajectoryPoint(轨迹点))
}

// only used in navigation mode                                                       // 只在巡航模式下才会使用
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,                // 输入是x,y坐标的偏差值, 航向角(theta), 上一次发布的轨迹
    PublishableTrajectory* prev_trajectory) {    // 在巡航模式中使用
  if (!prev_trajectory) {                                                             // 容错处理
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);                                            // cos的值
  double sin_theta = -std::sin(theta_diff);                                           // sin的值

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);                               // 旋转矩阵
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);
  // 迭代其中的每个元素都执行 一个旋转矩阵的操作
  std::for_each(prev_trajectory->trajectory_points().begin(),                         // 迭代上次轨迹中的每一个点
                prev_trajectory->trajectory_points().end(),
                [&cos_theta, &sin_theta, &tx, &ty,                                    // 每个点执行的lambda函数
                 &theta_diff](common::TrajectoryPoint& p) {
                  auto x = p.path_point().x();                                        // 获取每个点的坐标
                  auto y = p.path_point().y();
                  auto theta = p.path_point().theta();

                  auto x_new = cos_theta * x - sin_theta * y + tx;                    // 通过旋转矩阵获得新的坐标
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new = common::math::WrapAngle(theta - theta_diff);       // theta的角度不能超过2pi

                  p.mutable_path_point()->set_x(x_new);                               // 通过旋转矩阵后获得新的坐标和航向角
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}

// Planning from current vehicle state: enable_trajectory_stitcher
// if 1. the auto-driving mode is off or
//    2. we don't have the trajectory from last planning cycle or
//    3. the position deviation from actual and target is too high
// 从车辆当前的状态进行planning(规划), 要使能轨迹点的合并器
// 自动驾驶模式被关掉, 从上一次规划时间中没有获得相应的轨迹, 实际位置的偏差和目标值的偏差实在太大了
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(         // 上面三种情况下， 会使能合并轨迹
    const VehicleState& vehicle_state, const double current_timestamp,               // 车辆的状态， 当前的时间戳， 做一次planning的时间(10ms)
    const double planning_cycle_time,
    const PublishableTrajectory* prev_trajectory) {                                  // 上一次发布的轨迹(trajectory)
  if (!FLAGS_enable_trajectory_stitcher) {                                           // 没有开启自动驾驶就不停的重新初始化, FLAGS_enable_trajectory_stitcher默认是true
    return ComputeReinitStitchingTrajectory(vehicle_state);                          // 如果没有使能轨迹合并的话， 就会重新初始化轨迹点
  }
  if (!prev_trajectory) {                                                            // 上一个发布trajectory不存在, 重新初始化(刚开机的时候就不存在)
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  // 不是自动驾驶模式
  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {        // 不是自动驾驶模式, (手动的开车)也会进行重新初始化轨迹的起点(s(路程)和时间t都会进行重新计算)
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  // 之前有多少点
  std::size_t prev_trajectory_size = prev_trajectory->NumOfPoints();                 // 算出上一次轨迹中有多少个点

  if (prev_trajectory_size == 0) {                                                   // 上次发布的轨迹的点数为0, 那么还是会把轨迹点重新初始化
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  const double veh_rel_time =                                                        // 当前时间和上一次发布轨迹的时间差
      current_timestamp - prev_trajectory->header_time();                            // 时间戳

  std::size_t time_matched_index =                                                   // 通过时间进行匹配的索引值的下界
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);                           // 小于当前时间戳的第一个点
  // 没有匹配的点
  if (time_matched_index == 0 &&                                                     // 当前的时间比上一次发布点的第一个点还要小, 
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    return ComputeReinitStitchingTrajectory(vehicle_state);                          // 那么就重新初始化轨迹点
  }
  if (time_matched_index + 1 >= prev_trajectory_size) {                              // 如果当前的时间点比最后的一个时间点还要大，也要进行重新初始化
    AWARN << "current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
                                                                                     // 否则在上次发布的轨迹中寻找到了对应时间点的索引值
  auto time_matched_point =
      prev_trajectory->TrajectoryPointAt(time_matched_index);                        // 取出对应的那个路径点

  if (!time_matched_point.has_path_point()) {                                        // 在prev中不存在, 这个检查感觉有点多余吧?
    return ComputeReinitStitchingTrajectory(vehicle_state);                          // 不存在对应轨迹点， 就重新初始化轨迹点
  }

  std::size_t position_matched_index = prev_trajectory->QueryNearestPoint(           // 获取车辆最近的一个轨迹点(通过车辆的位置获得一个索引值)
      {vehicle_state.x(), vehicle_state.y()});                                       // 与车位置最相近的点

  auto frenet_sd = ComputePositionProjection(                                        // 计算车的位置投影, 投影到frenet坐标系
      vehicle_state.x(), vehicle_state.y(),                                          // frenet_sd是一个点对, 第一个是内积的结果， 第二个是叉积的结果
      prev_trajectory->TrajectoryPointAt(position_matched_index));                   // s是路程, 但d是什么意思呢? 下面的意思又是s代表的路程， d代表的就是横向的误差(dl的值?)

  auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;             // 纵向误差
  auto lat_diff = frenet_sd.second;                                                  // 横向误差

  ADEBUG << "Control lateral diff: " << lat_diff                                     // 横纵误差值为5
         << ", longitudinal diff: " << lon_diff;                                     // debug出相应的信息

  if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold ||               // 横向误差阈值为5.0，就是超过5.0的话就需要重新进行planning
      std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {          // 在允许误差范围内, 纵向的误差也是5.0米, 就是超过5.0的话也会进行重新做planning(规划)
    AERROR << "the distance between matched point and actual position is too "
              "large. Replan is triggered. lat_diff = "
           << lat_diff << ", lon_diff = " << lon_diff;
    return ComputeReinitStitchingTrajectory(vehicle_state);                          // 超过误差重新初始化， 横向或纵向一个方向的误差太大就会进行重新初始化轨迹的点(起点)
  }

  double forward_rel_time =                                                          // 相对于前面参考线的相对的时间戳
      prev_trajectory->TrajectoryPointAt(time_matched_index).relative_time() +       // 前向的轨迹点的时间， 加上做一次planning的时间
      planning_cycle_time;                                                           // 做一次planning的时间间隔(10hz, 100ms)

  std::size_t forward_time_index =                                                   // 通过这个时间戳获取一个索引值(这个索引值对应的点的时间大于或等于forward_time_index)
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);                       // 找到该时间间隔最近的元素

  ADEBUG << "Position matched index: " << position_matched_index;                    // debug 出对应的值
  ADEBUG << "Time matched index: " << time_matched_index;

  auto matched_index = std::min(time_matched_index, position_matched_index);         // 取两者最小的, 按时间最佳匹配和空间上最佳匹配
  std::vector<TrajectoryPoint> stitching_trajectory(                                 // 构造一个合并的轨迹
      prev_trajectory->trajectory_points().begin() +
          std::max(0, static_cast<int>(matched_index - 1)),                          // 必须要强制转换
      prev_trajectory->trajectory_points().begin() + forward_time_index + 1);        // 至少会得到有两个元素的数组

  const double zero_s = stitching_trajectory.back().path_point().s();                // 初始值长度
  for (auto& tp : stitching_trajectory) {                                            // trajectory里面必须要有path
    if (!tp.has_path_point()) {                                                      // 迭代合并轨迹中的所有的点
      return ComputeReinitStitchingTrajectory(vehicle_state);                        // 如果有一点不存在，就重新初始化起点
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -       // 设置合并轨迹的时间, 相对时间+头部时间-当前的时间戳
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);                    // 已经走过的路径， path point点中的s再减去(-)零点的s
  }
  return stitching_trajectory;
}
// 将trajectory进行坐标投影
std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(             // 计算位置投影
    const double x, const double y, const TrajectoryPoint& p) {                      // 输入是车的位置坐标， 轨迹中的一点
  Vec2d v(x - p.path_point().x(), y - p.path_point().y());                           // 构造车到轨迹点的一个向量v
  Vec2d n(common::math::cos(                                                         // n为航向角对应的单位向量
              common::math::Angle16::from_rad(p.path_point().theta())),
          common::math::sin(
              common::math::Angle16::from_rad(p.path_point().theta())));

  std::pair<double, double> frenet_sd;                                               // 初始化一个空的点对， 这是指frenet坐标系中的sl坐标么?
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();                             // 第一个点放内积的值
  frenet_sd.second = v.CrossProd(n);                                                 // 第二个点放叉积的值
  return frenet_sd;                                                                  // 最后返回的是该frenet_sd的坐标
}

}  // namespace planning
}  // namespace apollo
