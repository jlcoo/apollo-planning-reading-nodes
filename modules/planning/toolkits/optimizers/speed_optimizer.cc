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

#include "modules/planning/toolkits/optimizers/speed_optimizer.h"

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_limit.h"

namespace apollo {
namespace planning {

using apollo::common::Status;                                                        // 状态
using apollo::planning_internal::StGraphBoundaryDebug;                               // st图中boundary的debug信息
using apollo::planning_internal::STGraphDebug;                                       // st图debug的信息

SpeedOptimizer::SpeedOptimizer(const std::string& name) : Task(name) {}              // 构造函数， 需要声明速度优化器的名字

apollo::common::Status SpeedOptimizer::Execute(                                      // 执行速度优化器
    Frame* frame, ReferenceLineInfo* reference_line_info) {                          // 一帧数据, 全部的参考线的信息
  Task::Execute(frame, reference_line_info);                                         // 会在task中进行frame和reference line信息的复制

  auto ret = Process(                                                                // 调用内部函数进行处理
      reference_line_info->AdcSlBoundary(), reference_line_info->path_data(),
      frame->PlanningStartPoint(), reference_line_info->reference_line(),
      *reference_line_info->mutable_speed_data(),
      reference_line_info->path_decision(),
      reference_line_info->mutable_speed_data());

  RecordDebugInfo(reference_line_info->speed_data());                                // 将计算得到的规划速度信息放到debug的信息中
  return ret;
}

void SpeedOptimizer::RecordDebugInfo(const SpeedData& speed_data) {                  // debug速度信息
  auto* debug = reference_line_info_->mutable_debug();                               // 获取可以改变的速度信息
  auto ptr_speed_plan = debug->mutable_planning_data()->add_speed_plan();            // 设置速度的规划
  ptr_speed_plan->set_name(Name());                                                  // 设置名字
  ptr_speed_plan->mutable_speed_point()->CopyFrom(
      {speed_data.speed_vector().begin(), speed_data.speed_vector().end()});
}

void SpeedOptimizer::RecordSTGraphDebug(const StGraphData& st_graph_data,            // 获取st图数据中的debug信息
                                        STGraphDebug* st_graph_debug) const {
  if (!FLAGS_enable_record_debug || !st_graph_debug) {                               // 检查是否使能debug的信息
    ADEBUG << "Skip record debug info";
    return;
  }

  st_graph_debug->set_name(Name());                                                  // 设置debug的名字
  for (const auto& boundary : st_graph_data.st_boundaries()) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary->id());
    switch (boundary->boundary_type()) {                                             // 边框的类型
      case StBoundary::BoundaryType::FOLLOW:                                         // 跟车
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_FOLLOW);
        break;
      case StBoundary::BoundaryType::OVERTAKE:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
        break;
      case StBoundary::BoundaryType::STOP:                                           // 停车
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_STOP);
        break;
      case StBoundary::BoundaryType::UNKNOWN:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
        break;
      case StBoundary::BoundaryType::YIELD:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
        break;
      case StBoundary::BoundaryType::KEEP_CLEAR:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_KEEP_CLEAR);
        break;
    }

    for (const auto& point : boundary->points()) {                                  // 迭代boundary中的所有点
      auto point_debug = boundary_debug->add_point();                               // 添加到debug的信息中
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  for (const auto& point : st_graph_data.speed_limit().speed_limit_points()) {      // 速度限制拷贝到debug的信息中
    common::SpeedPoint speed_point;
    speed_point.set_s(point.first);
    speed_point.set_v(point.second);
    st_graph_debug->add_speed_limit()->CopyFrom(speed_point);
  }

  const auto& speed_data = reference_line_info_->speed_data();                      // 从参考线中获取速度信息
  st_graph_debug->mutable_speed_profile()->CopyFrom(
      {speed_data.speed_vector().begin(), speed_data.speed_vector().end()});        // 设置到st图中的debug信息
}

}  // namespace planning
}  // namespace apollo
