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

#include "modules/planning/toolkits/optimizers/speed_decider/speed_decider.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/proto/decision.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;                                                        // 错误的代码
using apollo::common::math::Vec2d;                                                      // 二维的向量
using apollo::common::Status;                                                           // 模块的状态值
using apollo::perception::PerceptionObstacle;                                           // 预测的障碍物

SpeedDecider::SpeedDecider() : Task("SpeedDecider") {}                                  // 构造函数进行名字的构造对象

bool SpeedDecider::Init(const PlanningConfig& config) {                                 // 通过配置文件进行速度决策者的初始化
  dp_st_speed_config_ =
      config.lane_follow_scenario_config().dp_st_speed_config();                        // 拷贝速度配置项
  st_boundary_config_ = dp_st_speed_config_.st_boundary_config();                       // 拷贝边框配置项(boundary)
  return true;
}

apollo::common::Status SpeedDecider::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {                             // 执行speed decider的执行代码
  Task::Execute(frame, reference_line_info);                                            // 调用基类 的执行函数(就是复制frame中的数据和reference line中的数据)
  init_point_ = frame_->PlanningStartPoint();                                           // 轨迹的起点
  adc_sl_boundary_ = reference_line_info_->AdcSlBoundary();                             // 自动驾驶车辆的boundary
  reference_line_ = &reference_line_info_->reference_line();                            // 获得中心参考线
  if (!MakeObjectDecision(reference_line_info->speed_data(),
                          reference_line_info->path_decision())                         // 获得speed曲线中的障碍物决策
           .ok()) {
    const std::string msg = "Get object decision by speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);                                      // 出错了就返回错误
  }
  return Status::OK();
}

SpeedDecider::StPosition SpeedDecider::GetStPosition(                                   // 获取一个st boundary在st坐标系下的位置
    const SpeedData& speed_profile, const StBoundary& st_boundary) const {
  StPosition st_position = BELOW;                                                       // StPosition是一个枚举类型, 在什么之下
  if (st_boundary.IsEmpty()) {                                                          // st的边框是空的
    return st_position;
  }

  bool st_position_set = false;                                                         // st 坐标没有被设置
  const double start_t = st_boundary.min_t();                                           // boundary开始的时间
  const double end_t = st_boundary.max_t();                                             // boundary结束的时间
  for (size_t i = 0; i + 1 < speed_profile.speed_vector().size(); ++i) {                // 迭代所有的速度点
    const STPoint curr_st(speed_profile.speed_vector()[i].s(),
                          speed_profile.speed_vector()[i].t());                         // 当前的一个点
    const STPoint next_st(speed_profile.speed_vector()[i + 1].s(),                      // 下一个点
                          speed_profile.speed_vector()[i + 1].t());
    if (curr_st.t() < start_t && next_st.t() < start_t) {                               // 这两个点的时间t没有发生碰撞
      continue;
    }
    if (curr_st.t() > end_t) {                                                          // 当前点已经过了boundary的值
      break;
    }

    common::math::LineSegment2d speed_line(curr_st, next_st);                           // boundary上的点和当前点和下一个点组成的线段有交点
    if (st_boundary.HasOverlap(speed_line)) {                                           // 按理说到这里的话就是有重叠的
      ADEBUG << "speed profile cross st_boundaries.";
      st_position = CROSS;                                                              // 将这个枚举类型设置为交叉(cross)

      // check if KEEP_CLEAR obstacle is "crossable"
      if (st_boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {        // 检查keep clear的障碍物是否在cross的时间段中
        const auto& last_speed_point = speed_profile.speed_vector().back();             // 上一个速度点
        double last_speed_point_v = 0.0;                                                // 上个速度点的速度
        if (last_speed_point.has_v()) {                                                 // 如果存在速度就保存这个速度
          last_speed_point_v = last_speed_point.v();
        } else {
          const size_t len = speed_profile.speed_vector().size();                       // 没有速度就需要重新计算 
          if (len > 1) {
            const auto& last_2nd_speed_point =
                speed_profile.speed_vector()[len - 2];
            last_speed_point_v =
                (last_speed_point.s() - last_2nd_speed_point.s()) /                     // 速度的变化率
                (last_speed_point.t() - last_2nd_speed_point.t());
          }
        }
        constexpr double kKeepClearSlowSpeed = 4.0;  // m/s                             // keep clear的最低速度为4m/s
        if (last_speed_point.s() <= st_boundary.max_s() &&                              // 如果速度太慢就设置为blow
            last_speed_point_v < kKeepClearSlowSpeed) {
          st_position = BELOW;
        }
      }
      break;                                                                            // 如果有重叠的话就跳出for循环
    }

    // note: st_position can be calculated by checking two st points once
    //       but we need iterate all st points to make sure there is no CROSS
    if (!st_position_set) {                                                             // 通过两个st的点来计算st position
      if (start_t < next_st.t() && curr_st.t() < end_t) {
        STPoint bd_point_front = st_boundary.upper_points().front();                    // 上界点
        double side = common::math::CrossProd(bd_point_front, curr_st, next_st);        // 交叉点
        st_position = side < 0.0 ? ABOVE : BELOW;                                       // 通过side这个值确定
        st_position_set = true;
      }
    }
  }
  return st_position;
}

bool SpeedDecider::IsFollowTooClose(const PathObstacle& path_obstacle) const {          // 太近是否进行跟车
  if (!path_obstacle.IsBlockingObstacle()) {                                            // 是堵塞的障碍物是不会进行跟车
    return false;
  }

  if (path_obstacle.st_boundary().min_t() > 0.0) {                                      // 障碍物的最小时间大于0, 不会跟车, 这个意思是什么?
    return false;
  }
  const double obs_speed = path_obstacle.obstacle()->Speed();                           // 障碍物的速度
  const double ego_speed = init_point_.v();                                             // 自动驾驶车辆的速度(从起点中获取)
  if (obs_speed > ego_speed) {                                                          // 障碍物的速度比自动驾驶车辆的速度更大的话就不会进行跟车
    return false;
  }
  const double distance =
      path_obstacle.st_boundary().min_s() - FLAGS_min_stop_distance_obstacle;           // FLAGS_min_stop_distance_obstacle被设置为6m/s 
  constexpr double decel = 1.0;                                                         // 减速度为1.0m/s
  return distance < std::pow((ego_speed - obs_speed), 2) * 0.5 / decel;                 // 是否能减速到最小的距离
}

Status SpeedDecider::MakeObjectDecision(
    const SpeedData& speed_profile, PathDecision* const path_decision) const {          // 设置对象的决策者
  if (speed_profile.speed_vector().size() < 2) {                                        // 速度曲线不存在
    const std::string msg = "dp_st_graph failed to get speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);                                      // 返回错误状态信息
  }
  for (const auto* obstacle : path_decision->path_obstacles().Items()) {                // 迭代path中的障碍物
    auto* path_obstacle = path_decision->Find(obstacle->Id());                          // 找到对应的障碍物
    const auto& boundary = path_obstacle->st_boundary();                                // 获取对应障碍物的st的边界

    if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||                                 // 如果障碍物超出范围，就添加一个忽略决策
        boundary.max_t() < 0.0 ||
        boundary.min_t() > dp_st_speed_config_.total_time()) {
      AppendIgnoreDecision(path_obstacle);                                              // 添加一个忽略的决策在障碍物中
      continue;
    }

    if (path_obstacle->HasLongitudinalDecision()) {                                     // 纵向方向上没有对应的决策的话
      AppendIgnoreDecision(path_obstacle);                                              // 也会添加一个忽略的决策给这个障碍物
      continue;
    }

    auto position = GetStPosition(speed_profile, boundary);                             // 获取障碍物对应的boundary在速度曲线中的位置
    switch (position) {                                                                 // 通过不同的位置进行不同的策略
      case BELOW:
        if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {         // 在自动驾驶车辆的后面
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*path_obstacle, &stop_decision,
                                 -FLAGS_stop_line_stop_distance)) {                     // 离得太近就会停车
            path_obstacle->AddLongitudinalDecision("dp_st_graph/keep_clear",
                                                   stop_decision);
          }
        } else if (CheckIsFollowByT(boundary) &&                                        // 停止低速减速
                   (boundary.max_t() - boundary.min_t() >
                    FLAGS_follow_min_time_sec)) {
          // stop for low_speed decelerating
          if (IsFollowTooClose(*path_obstacle)) {                                       // 障碍物在低速的跟车????
            ObjectDecisionType stop_decision;
            if (CreateStopDecision(*path_obstacle, &stop_decision,
                                   -FLAGS_min_stop_distance_obstacle)) {                // 太近也会进行停车
              path_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                     stop_decision);
            }
          } else {  // high speed or low speed accelerating
            // FOLLOW decision
            ObjectDecisionType follow_decision;                                         // 障碍物在高速或者低速(但是在加速中)
            if (CreateFollowDecision(*path_obstacle, &follow_decision)) {
              path_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                     follow_decision);
            }
          }
        } else {                                                                        // 否则进行必然行为
          // YIELD decision
          ObjectDecisionType yield_decision;
          if (CreateYieldDecision(*path_obstacle, &yield_decision)) {
            path_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                   yield_decision);
          }
        }
        break;
      case ABOVE:
        if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {         // above的情况
          ObjectDecisionType ignore;
          ignore.mutable_ignore();
          path_obstacle->AddLongitudinalDecision("dp_st_graph", ignore);
        } else {
          // OVERTAKE decision
          ObjectDecisionType overtake_decision;
          if (CreateOvertakeDecision(*path_obstacle, &overtake_decision)) {
            path_obstacle->AddLongitudinalDecision("dp_st_graph/overtake",
                                                   overtake_decision);
          }
        }
        break;
      case CROSS:                                                                       // cross的情况
        if (obstacle->IsBlockingObstacle()) {                                           // 是堵塞的障碍物
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*path_obstacle, &stop_decision,
                                 -FLAGS_min_stop_distance_obstacle)) {
            path_obstacle->AddLongitudinalDecision("dp_st_graph/cross",
                                                   stop_decision);
          }
          const std::string msg =
              "Failed to find a solution for crossing obstacle:" +
              obstacle->Id();
          AERROR << msg;
          return Status(ErrorCode::PLANNING_ERROR, msg);
        }
        break;
      default:
        AERROR << "Unknown position:" << position;
    }
    AppendIgnoreDecision(path_obstacle);
  }
  return Status::OK();
}

void SpeedDecider::AppendIgnoreDecision(PathObstacle* path_obstacle) const {      // 给path上的障碍物增加一个忽略的障碍物
  ObjectDecisionType ignore_decision;
  ignore_decision.mutable_ignore();
  if (!path_obstacle->HasLongitudinalDecision()) {                                // 针对障碍物没有纵向的决策
    path_obstacle->AddLongitudinalDecision("dp_st_graph", ignore_decision);
  }
  if (!path_obstacle->HasLateralDecision()) {                                     // 没有横向的决策
    path_obstacle->AddLateralDecision("dp_st_graph", ignore_decision);
  }
}

bool SpeedDecider::CreateStopDecision(const PathObstacle& path_obstacle,          // 创建一个停止的决策
                                      ObjectDecisionType* const stop_decision,
                                      double stop_distance) const {
  DCHECK_NOTNULL(stop_decision);                                                  // 有效性检查

  const auto& boundary = path_obstacle.st_boundary();                             // 获取障碍物在st坐标下的边界
  const double fence_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + stop_distance;                // 栅栏的距离s
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();             // 主要的停止的s
  if (main_stop_s < fence_s) {
    ADEBUG << "Stop fence is further away, ignore.";                              // 栅栏放到停车点之前了, 就会报错
    return false;
  }

  const auto fence_point = reference_line_->GetReferencePoint(fence_s);           // 获得栅栏处在中心参考线上的距离

  // set STOP decision
  auto* stop = stop_decision->mutable_stop();                                     // 设置停止点
  stop->set_distance_s(stop_distance);                                            // 设置停止点的s值
  auto* stop_point = stop->mutable_stop_point();
  stop_point->set_x(fence_point.x());                                             // 设置停止点的坐标
  stop_point->set_y(fence_point.y());
  stop_point->set_z(0.0);
  stop->set_stop_heading(fence_point.heading());                                  // 设置停止点的航向角

  if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {         // 设置keep clear 停止的原因
    stop->set_reason_code(StopReasonCode::STOP_REASON_CLEAR_ZONE);
  }

  PerceptionObstacle::Type obstacle_type =
      path_obstacle.obstacle()->Perception().type();                              // 获取boundary的类型
  ADEBUG << "STOP: obstacle_id[" << path_obstacle.obstacle()->Id()
         << "] obstacle_type[" << PerceptionObstacle_Type_Name(obstacle_type)     // 输出debug的信息
         << "]";

  return true;
}

bool SpeedDecider::CreateFollowDecision(                                          // 创建跟车的决策(follow的决策)
    const PathObstacle& path_obstacle,
    ObjectDecisionType* const follow_decision) const {
  DCHECK_NOTNULL(follow_decision);                                                // 容错检查

  const double follow_speed = init_point_.v();                                    // 起点的速度
  const double follow_distance_s = -std::fmax(
      follow_speed * FLAGS_follow_time_buffer, FLAGS_follow_min_distance);        // 跟车的距离, FLAGS_follow_time_buffer设置为2.5米
                                                                                  // FLAGS_follow_min_distance设置为3.0米
  const auto& boundary = path_obstacle.st_boundary();                             // 障碍物中st的边框
  const double reference_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + follow_distance_s;            // 参考线的s
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();             // 停止点在参考中心线上的s
  if (main_stop_s < reference_s) {
    ADEBUG << "Follow reference_s is further away, ignore.";                      // 要跟随的的s实在太远了
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_s);               // 获得在参考线上的一个点

  // set FOLLOW decision
  auto* follow = follow_decision->mutable_follow();                               // 设置栅栏点的跟车决策
  follow->set_distance_s(follow_distance_s);
  auto* fence_point = follow->mutable_fence_point();
  fence_point->set_x(ref_point.x());
  fence_point->set_y(ref_point.y());
  fence_point->set_z(0.0);
  follow->set_fence_heading(ref_point.heading());

  PerceptionObstacle::Type obstacle_type =                                        // 感知到的类型
      path_obstacle.obstacle()->Perception().type();
  ADEBUG << "FOLLOW: obstacle_id[" << path_obstacle.obstacle()->Id()
         << "] obstacle_type[" << PerceptionObstacle_Type_Name(obstacle_type)
         << "]";

  return true;
}

bool SpeedDecider::CreateYieldDecision(                                           // 创建一个避让的策略
    const PathObstacle& path_obstacle,
    ObjectDecisionType* const yield_decision) const {
  DCHECK_NOTNULL(yield_decision);                                                 // 容错处理

  PerceptionObstacle::Type obstacle_type =
      path_obstacle.obstacle()->Perception().type();
  double yield_distance = FLAGS_yield_distance;                                   // FLAGS_yield_distance设置为3.0米
  switch (obstacle_type) {                                                        // 障碍物的类型
    case PerceptionObstacle::PEDESTRIAN:
    case PerceptionObstacle::BICYCLE:
      yield_distance = FLAGS_yield_distance_pedestrian_bycicle;                   // 5米内进行避让
      break;
    default:
      yield_distance = FLAGS_yield_distance;                                      // 默认是3米进行避让
      break;
  }

  const auto& obstacle_boundary = path_obstacle.st_boundary();
  const double yield_distance_s =
      std::max(-obstacle_boundary.min_s(), -yield_distance);

  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + obstacle_boundary.min_s() + yield_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_line_fence_s) {                                     // 参考点的距离太远
    ADEBUG << "Yield reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);    // 获得参考线上的一点

  // set YIELD decision
  auto* yield = yield_decision->mutable_yield();                                  // 设置避让的决策
  yield->set_distance_s(yield_distance_s);
  yield->mutable_fence_point()->set_x(ref_point.x());
  yield->mutable_fence_point()->set_y(ref_point.y());
  yield->mutable_fence_point()->set_z(0.0);
  yield->set_fence_heading(ref_point.heading());

  ADEBUG << "YIELD: obstacle_id[" << path_obstacle.obstacle()->Id()
         << "] obstacle_type[" << PerceptionObstacle_Type_Name(obstacle_type)
         << "]";

  return true;
}

bool SpeedDecider::CreateOvertakeDecision(                                        // 创建超车的策略
    const PathObstacle& path_obstacle,                                            // path上的障碍物
    ObjectDecisionType* const overtake_decision) const {                          // path上的超车决策
  DCHECK_NOTNULL(overtake_decision);                                              // 检查是否为空指针

  constexpr double kOvertakeTimeBuffer = 3.0;    // in seconds                    // 超车之前的时间至少为3秒
  constexpr double kMinOvertakeDistance = 10.0;  // in meters                     // 超车的最短距离至少为10米

  const auto& velocity = path_obstacle.obstacle()->Perception().velocity();       // 获得障碍物的预测速度
  const double obstacle_speed =
      common::math::Vec2d::CreateUnitVec2d(init_point_.path_point().theta())      // 障碍物的速度向量?
          .InnerProd(Vec2d(velocity.x(), velocity.y()));

  const double overtake_distance_s = std::fmax(
      std::fmax(init_point_.v(), obstacle_speed) * kOvertakeTimeBuffer,
      kMinOvertakeDistance);

  const auto& boundary = path_obstacle.st_boundary();
  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + overtake_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_line_fence_s) {
    ADEBUG << "Overtake reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);    // 在那个点上进行超车

  // set OVERTAKE decision
  auto* overtake = overtake_decision->mutable_overtake();
  overtake->set_distance_s(overtake_distance_s);
  overtake->mutable_fence_point()->set_x(ref_point.x());
  overtake->mutable_fence_point()->set_y(ref_point.y());
  overtake->mutable_fence_point()->set_z(0.0);
  overtake->set_fence_heading(ref_point.heading());

  PerceptionObstacle::Type obstacle_type =
      path_obstacle.obstacle()->Perception().type();
  ADEBUG << "OVERTAKE: obstacle_id[" << path_obstacle.obstacle()->Id()
         << "] obstacle_type[" << PerceptionObstacle_Type_Name(obstacle_type)
         << "]";

  return true;
}

bool SpeedDecider::CheckIsFollowByT(const StBoundary& boundary) const {         // 检查是都在时序范围内可以进行正常的跟车
  if (boundary.BottomLeftPoint().s() > boundary.BottomRightPoint().s()) {
    return false;
  }
  constexpr double kFollowTimeEpsilon = 1e-3;                                   // boundary的时间至少要小于0.5秒
  constexpr double kFollowCutOffTime = 0.5;
  if (boundary.min_t() > kFollowCutOffTime ||
      boundary.max_t() < kFollowTimeEpsilon) {
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
