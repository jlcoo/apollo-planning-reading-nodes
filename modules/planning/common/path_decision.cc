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

#include "modules/planning/common/path_decision.h"

#include <memory>
#include <utility>

#include "modules/common/util/util.h"

namespace apollo {
namespace planning {
                           // IndexedList里面封装了一个unordered_map
using IndexedPathObstacles = IndexedList<std::string, PathObstacle>;            // 通过字符串的ID就可以索引到障碍物

PathObstacle *PathDecision::AddPathObstacle(const PathObstacle &path_obstacle) {
  std::lock_guard<std::mutex> lock(obstacle_mutex_);   // scoped lock
  return path_obstacles_.Add(path_obstacle.Id(), path_obstacle);                // 向保存道路上障碍物的容器path_obstacles_(内部是一个hash表实现)添加一个新的障碍物(对应的名字和障碍物)
}

const IndexedPathObstacles &PathDecision::path_obstacles() const {              // 返回一个内部列表的索引
  return path_obstacles_;
}

PathObstacle *PathDecision::Find(const std::string &object_id) {                // 通过物体的id地址进行索引具体的障碍物, 返回的值是一个PathObstacle引用
  return path_obstacles_.Find(object_id);
}

const PathObstacle *PathDecision::Find(const std::string &object_id) const {    // 通过const进行重载
  return path_obstacles_.Find(object_id);
}

void PathDecision::SetStBoundary(const std::string &id,                         // 设置每个障碍物的boundary(边框)
                                 const StBoundary &boundary) {
  auto *obstacle = path_obstacles_.Find(id);

  if (!obstacle) {
    AERROR << "Failed to find obstacle : " << id;
    return;
  } else {
    obstacle->SetStBoundary(boundary);
  }
}

bool PathDecision::AddLateralDecision(const std::string &tag,                  // 标签, 添加横向决策的策略
                                      const std::string &object_id,            // 障碍物的id地址
                                      const ObjectDecisionType &decision) {    // 决策的类型
  auto *path_obstacle = path_obstacles_.Find(object_id);                       // 找到对应id的障碍物
  if (!path_obstacle) {                                                        // 容错检查
    AERROR << "failed to find obstacle";
    return false;
  }
  path_obstacle->AddLateralDecision(tag, decision);                            // 调用障碍物path_obstacle的方法AddLateralDecision进行添加策略
  return true;
}

void PathDecision::EraseStBoundaries() {                                       // 封装path_obstacles_擦除边界的方法
  for (const auto *path_obstacle : path_obstacles_.Items()) {
    auto *obstacle_ptr = path_obstacles_.Find(path_obstacle->Id());
    obstacle_ptr->EraseStBoundary();
  }
}

bool PathDecision::AddLongitudinalDecision(const std::string &tag,             // 添加纵向的决策
                                           const std::string &object_id,
                                           const ObjectDecisionType &decision) {
  auto *path_obstacle = path_obstacles_.Find(object_id);                       // 先找到障碍物的地址
  if (!path_obstacle) {
    AERROR << "failed to find obstacle";
    return false;
  }
  path_obstacle->AddLongitudinalDecision(tag, decision);
  return true;
}

bool PathDecision::MergeWithMainStop(const ObjectStop &obj_stop,               // ObjectStop在decision.proto文件中定义的
                                     const std::string &obj_id,                // 物体的具体id
                                     const ReferenceLine &reference_line,      // 中心参考线
                                     const SLBoundary &adc_sl_boundary) {      // 自动驾驶车辆本身的sl坐标系下的边框(boundary)
  apollo::common::PointENU stop_point = obj_stop.stop_point();                 // 停止的坐标(UTM坐标系中的坐标)
  common::SLPoint stop_line_sl;                                                // 停止点的sl坐标
  reference_line.XYToSL({stop_point.x(), stop_point.y()}, &stop_line_sl);      // 将xy坐标转换为sl坐标

  double stop_line_s = stop_line_sl.s();
  if (stop_line_s < 0 || stop_line_s > reference_line.Length()) {
    AERROR << "Ignore object:" << obj_id << " fence route_s[" << stop_line_s
           << "] not in range[0, " << reference_line.Length() << "]";
    return false;
  }

  // check stop_line_s vs adc_s, ignore if it is further way than main stop
  const double kStopBuff = 1.0;                                                // 自动驾驶车辆会加一个buff(1.0米)
  stop_line_s = std::fmax(stop_line_s, adc_sl_boundary.end_s() - kStopBuff);   // 停止时候的距离的s

  if (stop_line_s >= stop_reference_line_s_) {
    ADEBUG << "stop point is further than current main stop point.";
    return false;
  }

  main_stop_.Clear();
  main_stop_.set_reason_code(obj_stop.reason_code());
  main_stop_.set_reason("stop by " + obj_id);
  main_stop_.mutable_stop_point()->set_x(obj_stop.stop_point().x());           // 设置x
  main_stop_.mutable_stop_point()->set_y(obj_stop.stop_point().y());           // 设置y
  main_stop_.set_stop_heading(obj_stop.stop_heading());
  stop_reference_line_s_ = stop_line_s;

  ADEBUG << " main stop obstacle id:" << obj_id                                // 输出一些信息
         << " stop_line_s:" << stop_line_s << " stop_point: ("
         << obj_stop.stop_point().x() << obj_stop.stop_point().y()
         << " ) stop_heading: " << obj_stop.stop_heading();
  return true;
}

}  // namespace planning
}  // namespace apollo
