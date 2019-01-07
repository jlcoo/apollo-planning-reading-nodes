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

#ifndef MODULES_PLANNING_COMMON_FRAME_H_
#define MODULES_PLANNING_COMMON_FRAME_H_

#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/geometry.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/change_lane_decider.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/lag_prediction.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/reference_line/reference_line_provider.h"

namespace apollo {
namespace planning {

/**
 * @class Frame
 *
 * @brief Frame holds all data for one planning cycle.
 */
// 一帧数据就是一次planning
// 用于数据准备,包括地图，车辆状态，prediction数据，ref_line 
class Frame {                                                         // frame是指做planning所需的数据
 public:     // 构造函数
  explicit Frame(uint32_t sequence_num,                               // 序列号
                 const common::TrajectoryPoint &planning_start_point, // 规划的起点
                 const double start_time,                             // 开始的时间
                 const common::VehicleState &vehicle_state,           // 车辆的状态
                 ReferenceLineProvider *reference_line_provider);     // 参考线提供者

  const common::TrajectoryPoint &PlanningStartPoint() const;
  common::Status Init();   // 初始化

  uint32_t SequenceNum() const;

  std::string DebugString() const;

  const PublishableTrajectory &ComputedTrajectory() const;            // 可发布的轨迹

  void RecordInputDebug(planning_internal::Debug *debug);

  std::list<ReferenceLineInfo> &reference_line_info();                // 车道中心线信息(reference line)

  Obstacle *Find(const std::string &id);                              // 通过障碍物的id查找出指定的障碍物

  const ReferenceLineInfo *FindDriveReferenceLineInfo();              // 找出可行驶参考线的信息

  const ReferenceLineInfo *DriveReferenceLineInfo() const;

  const std::vector<const Obstacle *> obstacles() const;

  const Obstacle *CreateStopObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_s);

  const Obstacle *CreateStopObstacle(const std::string &obstacle_id,
                                     const std::string &lane_id,
                                     const double lane_s);

  const Obstacle *CreateStaticObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_start_s,
      const double obstacle_end_s);

  bool Rerouting();    // 重新routing参考线

  const common::VehicleState &vehicle_state() const;

  static void AlignPredictionTime(
      const double planning_start_time,
      prediction::PredictionObstacles *prediction_obstacles);

  ADCTrajectory *mutable_trajectory() { return &trajectory_; }

  const ADCTrajectory &trajectory() const { return trajectory_; }

  const bool is_near_destination() const { return is_near_destination_; }

  /**
   * @brief Adjust reference line priority according to actual road conditions
   * @id_to_priority lane id and reference line priority mapping relationship
   */
  void UpdateReferenceLinePriority(
      const std::map<std::string, uint32_t> &id_to_priority);

 private:
  bool CreateReferenceLineInfo();

  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const Obstacle *FindCollisionObstacle() const;

  /**
   * @brief create a static virtual obstacle
   */
  const Obstacle *CreateStaticVirtualObstacle(const std::string &id,
                                              const common::math::Box2d &box);

  void AddObstacle(const Obstacle &obstacle);

 private:
  uint32_t sequence_num_ = 0;
  const hdmap::HDMap *hdmap_ = nullptr;            // 指向高精地图的指针
  common::TrajectoryPoint planning_start_point_;   // 从stitching_trajectory里面获取的点 
  const double start_time_ = 0.0;
  common::VehicleState vehicle_state_;             // 车辆状态
  std::list<ReferenceLineInfo> reference_line_info_;  // CreateReferenceLineInfo
  bool is_near_destination_ = false;

  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

  prediction::PredictionObstacles prediction_;                   // 感知的障碍物
  ThreadSafeIndexedObstacles obstacles_;                         // 线程安全的障碍物索引
  ChangeLaneDecider change_lane_decider_;                        // 变道决策者
  ADCTrajectory trajectory_;  // last published trajectory
  std::unique_ptr<LagPrediction> lag_predictor_;
  ReferenceLineProvider *reference_line_provider_ = nullptr;     // 参考线
  apollo::common::monitor::MonitorLogger monitor_logger_;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 private:
  DECLARE_SINGLETON(FrameHistory);    // 单例对象
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_FRAME_H_
