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

#include "modules/planning/toolkits/optimizers/task.h"

#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

Task::Task(const std::string& name) : name_(name) {}                             // 构造名字

const std::string& Task::Name() const { return name_; }                          // 任务的名字

bool Task::Init(const PlanningConfig&) { return true; }                          // 是否已经进行了初始化工作

Status Task::Execute(Frame* frame, ReferenceLineInfo* reference_line_info) {
  frame_ = frame;                                                                // task就是最名字的复制, 道路中心线的复制
  reference_line_info_ = reference_line_info;                                    // 这个执行函数execute更像一个init函数的功能
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
