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

  const common::TrajectoryPoint &PlanningStartPoint() const;          // 返回planning的起点
  common::Status Init();                                              // 初始化

  uint32_t SequenceNum() const;                                       // 第几帧(frame的序列号)

  std::string DebugString() const;                                    // debug的信息

  const PublishableTrajectory &ComputedTrajectory() const;            // 可发布的轨迹

  void RecordInputDebug(planning_internal::Debug *debug);             // 记录输入的debug信息

  std::list<ReferenceLineInfo> &reference_line_info();                // 车道中心线信息(reference line), ReferenceLineInfo中包含了很多信息

  Obstacle *Find(const std::string &id);                              // 通过障碍物的id查找出指定的障碍物, 查找一帧数据中的某个障碍物

  const ReferenceLineInfo *FindDriveReferenceLineInfo();              // 找出可行驶参考线的信息, 但返回的是信息更多的ReferenceLineInfo

  const ReferenceLineInfo *DriveReferenceLineInfo() const;            // 可行驶车道的各种信息(主要包含了路程信息, 速度信息, 障碍物信息等)

  const std::vector<const Obstacle *> obstacles() const;              // 一阵frame数据中的所有障碍物, 会用一个数组保存, 数组中的内容是一个Obstacle的指针(Obstacle*)

  const Obstacle *CreateStopObstacle(                                 // 通过参考中心线的信息ReferenceLineInfo, 障碍物的id名字, 障碍物在s上的坐标, 创建一个障碍物, 然后返回一个指针
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_s);

  const Obstacle *CreateStopObstacle(const std::string &obstacle_id,  // 通过障碍物的id, 车道的id号, lane中的s距离, 创建一个停止的障碍物(有可能是终点的障碍物)
                                     const std::string &lane_id,      // 车道的id号
                                     const double lane_s);

  const Obstacle *CreateStaticObstacle(
      ReferenceLineInfo *const reference_line_info,                   // 通过中心参考线的信息ReferenceLineInfo, 字符串的障碍物id号, 
      const std::string &obstacle_id, const double obstacle_start_s,  // 障碍物起点的s, 障碍物终点的s
      const double obstacle_end_s);

  bool Rerouting();                                                   // 重新routing参考线

  const common::VehicleState &vehicle_state() const;                  // 车辆的状态

  static void AlignPredictionTime(                                    // 和预测的时间对齐
      const double planning_start_time,                               // planning开始的时间
      prediction::PredictionObstacles *prediction_obstacles);         // 预测的障碍物

  ADCTrajectory *mutable_trajectory() { return &trajectory_; }        // 返回可以改变的自动驾驶轨迹的值

  const ADCTrajectory &trajectory() const { return trajectory_; }     // 返回只读的自动驾驶车辆的轨迹
                                                                      // 是否离目标点很近了
  const bool is_near_destination() const { return is_near_destination_; } 

  /**
   * @brief Adjust reference line priority according to actual road conditions
   * @id_to_priority lane id and reference line priority mapping relationship
   */                                                                 // 根据实际道路的坐标调整道路参考线的优先级
  void UpdateReferenceLinePriority(                                   // id_to_priority是道路id和优先级进行映射的关系表(红黑树)
      const std::map<std::string, uint32_t> &id_to_priority);

 private:
  bool CreateReferenceLineInfo();                                     // 创建一个ReferenceLineInfo对象??

  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const Obstacle *FindCollisionObstacle() const;                      // 找一个与自动驾驶车辆相撞的障碍物

  /**
   * @brief create a static virtual obstacle
   */                                                                 // 创建一个静态的虚拟障碍物
  const Obstacle *CreateStaticVirtualObstacle(const std::string &id,  // 该静态的虚拟障碍物的id号
                                              const common::math::Box2d &box); // 障碍物的边框box(二维的)

  void AddObstacle(const Obstacle &obstacle);                         // 向一帧数据中添加一个障碍物(Obstacle)

 private:
  uint32_t sequence_num_ = 0;                                         // 最开始初始化帧数为0
  const hdmap::HDMap *hdmap_ = nullptr;                               // 指向高精地图的指针
  common::TrajectoryPoint planning_start_point_;                      // 从stitching_trajectory里面获取的点 
  const double start_time_ = 0.0;                                     // 开始的时间初始化为0
  common::VehicleState vehicle_state_;                                // 车辆状态
  std::list<ReferenceLineInfo> reference_line_info_;                  // CreateReferenceLineInfo, 很多条车道中心参考线(reference line)
  bool is_near_destination_ = false;                                  // 离终点(目标点)还有很远

  /**
   * the reference line info that the vehicle finally choose to drive on
   **/                                                                // 从众多的车道参考线中选一条来驾驶
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;      // 初始化为一个空指针

  prediction::PredictionObstacles prediction_;                        // 感知的障碍物
  ThreadSafeIndexedObstacles obstacles_;                              // 线程安全的障碍物索引
  ChangeLaneDecider change_lane_decider_;                             // 变道决策者
  ADCTrajectory trajectory_;  // last published trajectory            // 最新的自动驾驶车辆的轨迹
  std::unique_ptr<LagPrediction> lag_predictor_;                      // 滞后的预测器
  ReferenceLineProvider *reference_line_provider_ = nullptr;          // 参考线的提供者, 会把ReferenceLine保存在list中
  apollo::common::monitor::MonitorLogger monitor_logger_;             // 日志监控者
};
// 数据的历史数据放到的是planning的历史空间中
class FrameHistory : public IndexedQueue<uint32_t, Frame> {           // 历史数据继承于一个引用队列, 而且是一个单例对象
 private:
  DECLARE_SINGLETON(FrameHistory);    // 单例对象
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_FRAME_H_
