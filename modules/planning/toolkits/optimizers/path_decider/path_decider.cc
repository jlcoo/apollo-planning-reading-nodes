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

#include "modules/planning/toolkits/optimizers/path_decider/path_decider.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/proto/decision.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;                                                       // 错误码
using apollo::common::Status;                                                          // 状态值
using apollo::common::VehicleConfigHelper;                                             // 车辆配置的帮助者

PathDecider::PathDecider() : Task("PathDecider") {}                                    // 构造函数

apollo::common::Status PathDecider::Execute(                                           // 执行路径决策的任务
    Frame *frame, ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);                                           // 其实就是进行初始化, 将所有数据的一帧数据和参考线信息拷贝过来
  return Process(reference_line_info->path_data(),
                 reference_line_info->path_decision());
}

Status PathDecider::Process(const PathData &path_data,                                 // 处理路径优化的决策者
                            PathDecision *const path_decision) {
  CHECK_NOTNULL(path_decision);                                                        // 检查指针是否为空指针
  if (!MakeObjectDecision(path_data, path_decision)) {                                 // 生成对象的决策
    AERROR << "Failed to make decision based on tunnel";
    return Status(ErrorCode::PLANNING_ERROR, "dp_road_graph decision ");               // 在通路上进行决策失败
  }
  return Status::OK();
}

bool PathDecider::MakeObjectDecision(const PathData &path_data,                        // 对象的决策主要是执行障碍物的决策
                                     PathDecision *const path_decision) {
  DCHECK_NOTNULL(path_decision);
  if (!MakeStaticObstacleDecision(path_data, path_decision)) {
    AERROR << "Failed to make decisions for static obstacles";
    return false;
  }
  return true;
}

bool PathDecider::MakeStaticObstacleDecision(                                          // 面对障碍物的决策
    const PathData &path_data, PathDecision *const path_decision) {
  DCHECK_NOTNULL(path_decision);                                                       // 决策的空指针
  const auto &frenet_path = path_data.frenet_frame_path();                             // 获取frenet的path(在sl坐标下的轨迹)
  const auto &frenet_points = frenet_path.points();                                    // 取出路径中的所有点
  if (frenet_points.empty()) {                                                         // 如果没有frenet的点
    AERROR << "Path is empty.";
    return false;                                                                      // 直接报错返回
  }

  const double half_width =                                                            // 车身一半的宽度
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  const double lateral_radius = half_width + FLAGS_lateral_ignore_buffer;              // 横向的旋转半径, FLAGS_lateral_ignore_buffer为3米

  const double lateral_stop_radius =                                                   // 横向停止的半径, 0.5以内的横向障碍物就忽略
      half_width + FLAGS_static_decision_nudge_l_buffer;

  for (const auto *path_obstacle : path_decision->path_obstacles().Items()) {          // 迭代path中所有的障碍物
    const auto &obstacle = *path_obstacle->obstacle();                                 // 获取障碍物
    bool is_bycycle_or_pedestrain =                                                    // 看看障碍物是否是自行车或这行人
        (obstacle.Perception().type() ==
             perception::PerceptionObstacle::BICYCLE ||
         obstacle.Perception().type() ==
             perception::PerceptionObstacle::PEDESTRIAN);

    if (!is_bycycle_or_pedestrain && !obstacle.IsStatic()) {                           // 如果不是自行车， 不是行人， 不是静态障碍物就继续其他障碍物
      continue;
    }

    if (path_obstacle->HasLongitudinalDecision() &&                                    // 如果横向和纵向的决策都可以忽略的话, 就继续迭代其他障碍物
        path_obstacle->LongitudinalDecision().has_ignore() &&
        path_obstacle->HasLateralDecision() &&
        path_obstacle->LateralDecision().has_ignore()) {
      continue;
    }
    if (path_obstacle->HasLongitudinalDecision() &&                                    // 如果纵向的决策为停止决策， 跳过该障碍物的决策
        path_obstacle->LongitudinalDecision().has_stop()) {
      // STOP decision
      continue;
    }
    if (path_obstacle->HasLateralDecision() &&
        path_obstacle->LateralDecision().has_sidepass()) {                             // 如果横向的决策为停止策略, 跳过该障碍物的决策
      // SIDE_PASS decision
      continue;
    }

    if (path_obstacle->reference_line_st_boundary().boundary_type() ==                 // 参考线的st坐标系下的边框类型是keep clear， 跳过该障碍物
        StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    // IGNORE by default
    ObjectDecisionType object_decision;                                                // 对象决策类型默认的是忽略
    object_decision.mutable_ignore();

    const auto &sl_boundary = path_obstacle->PerceptionSLBoundary();                   // 获取sl感知到的边框

    if (sl_boundary.end_s() < frenet_points.front().s() ||                             // 感知到的障碍物不在坐标点的前方
        sl_boundary.start_s() > frenet_points.back().s()) {
      path_decision->AddLongitudinalDecision("PathDecider/not-in-s",                   // 设置错误并跳过该障碍物
                                             obstacle.Id(), object_decision);
      path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle.Id(),
                                        object_decision);
      continue;
    }

    const auto frenet_point = frenet_path.GetNearestPoint(sl_boundary);                // 获取sl边框的最近的frenet坐标点
    const double curr_l = frenet_point.l();                                            // 获取该点的l
    if (curr_l - lateral_radius > sl_boundary.end_l() ||
        curr_l + lateral_radius < sl_boundary.start_l()) {                             // 可以忽略的决策
      // ignore
      path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle.Id(),         // 因为障碍物横向不在考虑范围内
                                        object_decision);
    } else if (curr_l - lateral_stop_radius < sl_boundary.end_l() &&
               curr_l + lateral_stop_radius > sl_boundary.start_l()) {                 // 如果特别近
      // stop
      *object_decision.mutable_stop() =
          GenerateObjectStopDecision(*path_obstacle);                                  // 就进行停车

      if (path_decision->MergeWithMainStop(
              object_decision.stop(), obstacle.Id(),
              reference_line_info_->reference_line(),
              reference_line_info_->AdcSlBoundary())) {
        path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",             // 设置停止的信息
                                               obstacle.Id(), object_decision);
      } else {
        ObjectDecisionType object_decision;                                            // 不是最近的停止点
        object_decision.mutable_ignore();
        path_decision->AddLongitudinalDecision("PathDecider/not-nearest-stop",
                                               obstacle.Id(), object_decision);
      }
    } else if (FLAGS_enable_nudge_decision) {                                          // FLAGS_enable_nudge_decision被配置为true， 使能了忽略决策
      // nudge
      if (curr_l - lateral_stop_radius > sl_boundary.end_l()) {                        // 左边宽一点, 就是忽略左边的
        // LEFT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
        object_nudge_ptr->set_distance_l(FLAGS_nudge_distance_obstacle);
        path_decision->AddLateralDecision("PathDecider/left-nudge",
                                          obstacle.Id(), object_decision);
      } else {                                                                         // 否则忽略右边的障碍物
        // RIGHT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
        object_nudge_ptr->set_distance_l(-FLAGS_nudge_distance_obstacle);
        path_decision->AddLateralDecision("PathDecider/right-nudge",
                                          obstacle.Id(), object_decision);
      }
    }
  }

  return true;
}

ObjectStop PathDecider::GenerateObjectStopDecision(                                  // 产生对象停止的决策
    const PathObstacle &path_obstacle) const {
  ObjectStop object_stop;                                                            // 新建一个决策对象

  double stop_distance = path_obstacle.MinRadiusStopDistance(                        // 获取最小的停止距离
      VehicleConfigHelper::GetConfig().vehicle_param());
  object_stop.set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);                 // 设置停止的原因
  object_stop.set_distance_s(-stop_distance);                                        // 停止距离设置为负数是什么意思哦???

  const double stop_ref_s =
      path_obstacle.PerceptionSLBoundary().start_s() - stop_distance;                // 感知到的障碍物减去停车的最小距离
  const auto stop_ref_point =
      reference_line_info_->reference_line().GetReferencePoint(stop_ref_s);          // 通过停车的s, 获取一个参考线上的一点
  object_stop.mutable_stop_point()->set_x(stop_ref_point.x());                       // 设置停车在xy坐标系下的坐标
  object_stop.mutable_stop_point()->set_y(stop_ref_point.y());
  object_stop.set_stop_heading(stop_ref_point.heading());                            // 设置停车的航向角
  return object_stop;                                                                // 并返回该停车对象
}

}  // namespace planning
}  // namespace apollo
