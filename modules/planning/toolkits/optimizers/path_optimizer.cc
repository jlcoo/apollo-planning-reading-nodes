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

#include "modules/planning/toolkits/optimizers/path_optimizer.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
// 路径优化器
PathOptimizer::PathOptimizer(const std::string& name) : Task(name) {}

apollo::common::Status PathOptimizer::Execute(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  Task::Execute(frame, reference_line_info);
  auto ret = Process(
      reference_line_info->speed_data(), reference_line_info->reference_line(),
      frame->PlanningStartPoint(), reference_line_info->mutable_path_data());   // 速度的数据, 参考线的数据, planning的起点, 可变的path的数据 
  RecordDebugInfo(reference_line_info->path_data());                            // 记录Debug的信息
  if (ret != Status::OK()) {                                                    // 调用不正常就设置错误
    reference_line_info->SetDrivable(false);
    AERROR << "Reference Line " << reference_line_info->Lanes().Id()
           << " is not drivable after " << Name();
  }
  return ret;
}

void PathOptimizer::RecordDebugInfo(const PathData& path_data) {                // 记录debug的信息
  const auto& path_points = path_data.discretized_path().path_points();         // 离散的点
  auto* ptr_optimized_path = reference_line_info_->mutable_debug()              // 增加对应的离散点
                                 ->mutable_planning_data()
                                 ->add_path();
  ptr_optimized_path->set_name(Name());                                         // 设置优化路径的名字
  ptr_optimized_path->mutable_path_point()->CopyFrom(                           // 设置优化路径的离散点(path point)
      {path_points.begin(), path_points.end()});
}

}  // namespace planning
}  // namespace apollo
