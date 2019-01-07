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

#ifndef MODULES_PLANNING_COMMON_PATH_DECISION_H_
#define MODULES_PLANNING_COMMON_PATH_DECISION_H_

#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "modules/planning/proto/decision.pb.h"

#include "modules/planning/common/indexed_list.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path_obstacle.h"

namespace apollo {
namespace planning {

/**
 * @class PathDecision
 *
 * @brief PathDecision represents all obstacle decisions on one path.
 */ // 所有障碍物的决策
class PathDecision {                                                              // PathDecision的意思是在一个道路上的所有决策
 public:
  PathDecision() = default;                                                       // 默认的构造函数

  PathObstacle *AddPathObstacle(const PathObstacle &path_obstacle);               // 在一个道路上添加一个障碍物

  const IndexedList<std::string, PathObstacle> &path_obstacles() const;           // 将所有的障碍物放到一个列表中(这样就可以通过一个名字索引到一个障碍物)

  bool AddLateralDecision(const std::string &tag, const std::string &object_id,   // 每个障碍物都会贴一个标签, 一个固定的ID
                          const ObjectDecisionType &decision);                    // 添加侧向策略
  bool AddLongitudinalDecision(const std::string &tag,
                               const std::string &object_id,                      // 添加纵向策略
                               const ObjectDecisionType &decision);

  const PathObstacle *Find(const std::string &object_id) const;                   // const重载

  PathObstacle *Find(const std::string &object_id);                               // 通过物体的id找到道路上的障碍物

  void SetStBoundary(const std::string &id, const StBoundary &boundary);          // st的boundary, 边框
  void EraseStBoundaries();                                                       // 擦除掉边框
  MainStop main_stop() const { return main_stop_; }                               // MainStop是在decision.proto文件中定义
  double stop_reference_line_s() const { return stop_reference_line_s_; }         // 停止时在中心参考线中的s(路程)
  bool MergeWithMainStop(const ObjectStop &obj_stop, const std::string &obj_id,   // 合并停止点?, ObjectStop是在decision.proto文件中定义
                         const ReferenceLine &ref_line,
                         const SLBoundary &adc_sl_boundary);

 private:
  std::mutex obstacle_mutex_;                                                     // 障碍物的互斥量
  IndexedList<std::string, PathObstacle> path_obstacles_;                         // 内部通过障碍物的id索引到道路上的障碍物
  MainStop main_stop_;                                                            // 主要的停止点
  double stop_reference_line_s_ = std::numeric_limits<double>::max();             // 中心参考线的s, 初始化为最大值
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PATH_DECISION_H_
