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
 *   @file
 **/

#ifndef MODULES_PLANNING_TASKS_ST_GRAPH_ST_BOUNDARY_MAPPER_H_
#define MODULES_PLANNING_TASKS_ST_GRAPH_ST_BOUNDARY_MAPPER_H_

#include <string>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/st_boundary_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class StBoundaryMapper {                                                              // st边框的映射者
 public:
  StBoundaryMapper(const SLBoundary& adc_sl_boundary,                                 // 自动驾驶车辆的sl边框
                   const StBoundaryConfig& config,                                    // st坐标系的配置
                   const ReferenceLine& reference_line,                               // 中心参考线
                   const PathData& path_data, const double planning_distance,         // path(路程的数据)
                   const double planning_time, const bool is_change_lane);            // 做planning的时间

  virtual ~StBoundaryMapper() = default;                                              // 默认的析构函数

  apollo::common::Status CreateStBoundary(PathDecision* path_decision) const;         // 创建一个st的边框

  apollo::common::Status CreateStBoundaryWithHistory(                                 // 具有历史决策的st边框
      const ObjectDecisions& history_decisions,
      PathDecision* path_decision) const;

  apollo::common::Status CreateStBoundary(                                            // 通过障碍物和外部决策创建一个st的边框
      PathObstacle* path_obstacle,
      const ObjectDecisionType& external_decision) const;

 private:
  FRIEND_TEST(StBoundaryMapperTest, check_overlap_test);                              // 检查overlap的测试
  bool CheckOverlap(const apollo::common::PathPoint& path_point,                      // path中的点
                    const apollo::common::math::Box2d& obs_box,
                    const double buffer) const;

  /**
   * Creates valid st boundary upper_points and lower_points
   * If return true, upper_points.size() > 1 and
   * upper_points.size() = lower_points.size()
   */
  bool GetOverlapBoundaryPoints(                                                     // 创建一个有上界点和下界点的st boundary
      const std::vector<apollo::common::PathPoint>& path_points,
      const Obstacle& obstacle, std::vector<STPoint>* upper_points,                  // 障碍物， 上界点， 下界点
      std::vector<STPoint>* lower_points) const;

  apollo::common::Status MapWithoutDecision(PathObstacle* path_obstacle) const;      // 不带决策的map

  bool MapStopDecision(PathObstacle* stop_obstacle,                                  // 停止的决策
                       const ObjectDecisionType& decision) const;

  apollo::common::Status MapWithDecision(
      PathObstacle* path_obstacle, const ObjectDecisionType& decision) const;

 private:
  const SLBoundary& adc_sl_boundary_;                                                // 自动驾驶车辆的sl边框
  const StBoundaryConfig& st_boundary_config_;                                       // 边框的配置
  const ReferenceLine& reference_line_;                                              // 中心参考线
  const PathData& path_data_;                                                        // path的数据(data)
  const apollo::common::VehicleParam& vehicle_param_;                                // 车辆的参数
  const double planning_distance_;                                                   // 做planning的距离
  const double planning_time_;                                                       // 做planning的时间
  bool is_change_lane_ = false;                                                      // 是否进行变道
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_ST_GRAPH_ST_BOUNDARY_MAPPER_H_
