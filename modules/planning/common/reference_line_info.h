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

#ifndef MODULES_PLANNING_COMMON_REFERENCE_LINE_INFO_H_
#define MODULES_PLANNING_COMMON_REFERENCE_LINE_INFO_H_

#include <algorithm>
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

/**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */    // 包含了所有的数据
class ReferenceLineInfo {                                                       // 一条中心参考线的信息
 public:
  ReferenceLineInfo() = default;                                                // 默认的构造函数
  explicit ReferenceLineInfo(const common::VehicleState& vehicle_state,         // 车辆状态
                             const common::TrajectoryPoint& adc_planning_point, // TrajectoryPoint, 轨迹点
                             const ReferenceLine& reference_line,               // 参考线, ReferenceLine是一个类
                             const hdmap::RouteSegments& segments);             // routing 片段，RouteSegments继承于LaneSegment

  bool Init(const std::vector<const Obstacle*>& obstacles);                     // 通过障碍物指针的数组进行初始化中心参考线(reference line)

  bool IsInited() const;                                                        // 中心参考线是否被初始化

  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);             // 添加障碍物
  PathObstacle* AddObstacle(const Obstacle* obstacle);                          // 在车道上(path)添加一个障碍物
  bool AddObstacleHelper(const Obstacle* obstacle);                             // 障碍物的帮助者, 这个是什么意思?

  PathDecision* path_decision();                                                // path的决策者(看选择那一条道路)
  const PathDecision& path_decision() const;                                    // 通过const关键字重载返回只读的path决策的方法
  const ReferenceLine& reference_line() const;                                  // 返回只读的参考线

  bool ReachedDestination() const;                                              // 是否到达终点

  void SetTrajectory(const DiscretizedTrajectory& trajectory);                  // 通过离散的轨迹点设计车的轨迹

  const DiscretizedTrajectory& trajectory() const;                              // 返回离散的轨迹
  double TrajectoryLength() const;                                              // 获得轨迹的长度

  double Cost() const { return cost_; }                                         // 代价函数的值
  void AddCost(double cost) { cost_ += cost; }                                  // 添加代价函数
  void SetCost(double cost) { cost_ = cost; }                                   // 设置代价函数的值
  double PriorityCost() const { return priority_cost_; }                        // 有限代价
  void SetPriorityCost(double cost) { priority_cost_ = cost; }                  // 设置优先级代价
  // For lattice planner'speed planning target
  void SetStopPoint(const StopPoint& stop_point);                               // 对格子规划器设置停止点
  void SetCruiseSpeed(double speed);                                            // 设置巡航的速度
  const PlanningTarget& planning_target() const { return planning_target_; }    // planning的目标点

  /**
   * @brief check if current reference line is started from another reference
   *line info line. The method is to check if the start point of current
   *reference line is on previous reference line info.
   * @return returns true if current reference line starts on previous reference
   *line, otherwise false.
   **/                                                                          // 检查当前的参考线是否在之前的参考线上
  bool IsStartFrom(const ReferenceLineInfo& previous_reference_line_info) const;

  planning_internal::Debug* mutable_debug() { return &debug_; }                 // debug的信息
  const planning_internal::Debug& debug() const { return debug_; }              // 通过const关键字进行重载
  LatencyStats* mutable_latency_stats() { return &latency_stats_; }             // 延迟状态, 返回的状态可以修改
  const LatencyStats& latency_stats() const { return latency_stats_; }

  const PathData& path_data() const;                                            // 返回的path的数据, 这是一个类
  const SpeedData& speed_data() const;                                          // 速度的数据
  PathData* mutable_path_data();                                                // 获取一个可以修改的path data
  SpeedData* mutable_speed_data();                                              // 获取一个可以修改的speed data
  // aggregate final result together by some configuration
  bool CombinePathAndSpeedProfile(                                              // 通过一些配置项, 合并path和speed的结果, 最终以离散的轨迹作为输出(那么trajectory是path和speed的综合产物)
      const double relative_time, const double start_s,
      DiscretizedTrajectory* discretized_trajectory);

  const SLBoundary& AdcSlBoundary() const;                                      // 自动驾驶车辆的sl坐标系中的boundary(边框)
  const SLBoundary& VehicleSlBoundary() const;                                  // 其他车辆的sl坐标下的boundary(边框)
  std::string PathSpeedDebugString() const;                                     // path中速度和path的debug信息(字符串)

  /**
   * Check if the current reference line is a change lane reference line, i.e.,
   * ADC's current position is not on this reference line.
   */
  bool IsChangeLanePath() const;                                                // 当前的参考线是否是一条变道的参考线, adc当前的位置不在这个参考线上, 则就应该变道

  /**
   * Check if the current reference line is the neighbor of the vehicle
   * current position
   */
  bool IsNeighborLanePath() const;                                              // 检查当前的参考中心线是否和一个车辆相邻

  /**
   * Set if the vehicle can drive following this reference line
   * A planner need to set this value to true if the reference line is OK
   */
  void SetDrivable(bool drivable);                                              // 设置一辆车是否可以在这条中心参考线上进行跟车行驶
  bool IsDrivable() const;                                                      // 返回一个bool类型的值, 判断车是否能够进行跟车行驶

  void ExportEngageAdvice(common::EngageAdvice* engage_advice) const;           // 导入enage的建议

  bool IsSafeToChangeLane() const { return is_safe_to_change_lane_; }           // 变道是否是安全的

  const hdmap::RouteSegments& Lanes() const;                                    // 返回高精地图的车道, 是LaneSegment的数组
  const std::list<hdmap::Id> TargetLaneId() const;                              // 返回目标车道的id信息

  void ExportDecision(DecisionResult* decision_result) const;                   // 导入决策信息, DecisionResult主要包括主要决策, 对象决策和车辆信号等信息

  void SetJunctionRightOfWay(double junction_s, bool is_protected);             // 设置交叉路口右转
// ADCTrajectory在planning.proto文件中定义
  ADCTrajectory::RightOfWayStatus GetRightOfWayStatus() const;                  // 返回自动驾驶车辆的轨迹中的右转状态

  bool IsRightTurnPath() const;                                                 // 是否要进行右转

  double OffsetToOtherReferenceLine() const {                                   // 到另一条参考线的偏差
    return offset_to_other_reference_line_;
  } // 设置到其他参考线的误差
  void SetOffsetToOtherReferenceLine(const double offset) {                     // 设置到另一条参考线的偏差值
    offset_to_other_reference_line_ = offset;
  }

  void set_is_on_reference_line() { is_on_reference_line_ = true; }             // 设置一辆车辆(vechile)是否在中心参考线上

  uint32_t GetPriority() const { return reference_line_.GetPriority(); }        // 获取当前中心参考线的优先级

  void SetPriority(uint32_t priority) { reference_line_.SetPriority(priority); }// 设置参考线的优先级, 越大越好? 会在navi_planner.cc文件中参与cost的计算

  void set_trajectory_type(                                                     // 设置轨迹的类型
      const ADCTrajectory::TrajectoryType trajectory_type) {                    // 自动驾驶车辆轨迹的类型
    trajectory_type_ = trajectory_type;                                         // 有四种类型(不知道的类型, 正常的类型, 速度反馈, 路程反馈)
  }

  ADCTrajectory::TrajectoryType trajectory_type() const {                       // 返回轨迹的类型
    return trajectory_type_;
  }

 private:
  bool CheckChangeLane() const;                                                 // 私有的成员函数, 检查是否需要进行换道

  void ExportTurnSignal(common::VehicleSignal* signal) const;                   // 导入转向灯信息

  bool IsUnrelaventObstacle(PathObstacle* path_obstacle);                       // 是否是无关紧要的障碍物

  void MakeDecision(DecisionResult* decision_result) const;                     // 在ReferenceLineInfo中会进行决策
  int MakeMainStopDecision(DecisionResult* decision_result) const;              // 自动驾驶车辆的主要决策
  void MakeMainMissionCompleteDecision(DecisionResult* decision_result) const;  // 完成主要任务决策
  void MakeEStopDecision(DecisionResult* decision_result) const;                // 做出紧急停止的决策
  void SetObjectDecisions(ObjectDecisions* object_decisions) const;             // 设置目标的决策(我把它理解为是障碍物的决策)
  const common::VehicleState vehicle_state_;                                    // 车辆的状态
  const common::TrajectoryPoint adc_planning_point_;                            // 自动驾驶车辆规划的轨迹点(PathPoint加上速度, 加速度, 相对时间戳等信息)
  ReferenceLine reference_line_;                                                // 中心参考线(ReferenceLine是一个类)

  /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
  double cost_ = 0.0;                                                           // 测量代价函数的好坏, 值越小越好

  bool is_inited_ = false;                                                      // ReferenceLineInfo是否已经被初始化了

  bool is_drivable_ = true;                                                     // 先初始化为可行驶的

  PathDecision path_decision_;                                                  // 在path上的一些决策(主要包括了横向和纵向的决策)

  PathData path_data_;                                                          // 私有的path的数据, path会被映射到x,y坐标系和sl坐标系中
  SpeedData speed_data_;                                                        // 私有的speed的数据, 主要包含了s, t, v, a, da等信息

  DiscretizedTrajectory discretized_trajectory_;                                // 离散的轨迹, DiscretizedTrajectory为一个类, 在discretized_trajectory.h中定义

  struct {                                                                      // 在一个class中嵌套定义一个数据结构
    /**
     * @brief SL boundary of stitching point (starting point of plan trajectory)
     * relative to the reference line
     */                                                                         // 这是自动驾驶车辆的边框信息(boundary)
    SLBoundary adc_sl_boundary_;                                                // sl坐标系中的边框信息(boundary)
    /**
     * @brief SL boundary of vehicle realtime state relative to the reference
     * line
     */                                                                         // 车辆实时状态相对于中心参考线的sl边框(boundary)
    SLBoundary vehicle_sl_boundary_;
  } sl_boundary_info_;                                                          // 哪里用到了sl的边框信息呢

  planning_internal::Debug debug_;                                              // planning 内部debug的信息
  LatencyStats latency_stats_;                                                  // 延迟的状态信息
  // 所有的lanes就是一个RouteSegments(继承与vector, 内部是LaneSegment)
  hdmap::RouteSegments lanes_;                                                  // 车道的信息RouteSegments在route_segment.h中定义

  bool is_on_reference_line_ = false;                                           // 是否在车道中心参考线上

  bool is_safe_to_change_lane_ = false;                                         // 是否可以安全地变道

  ADCTrajectory::RightOfWayStatus status_ = ADCTrajectory::UNPROTECTED;         // 没有保护栏???

  double offset_to_other_reference_line_ = 0.0;                                 // 到另外一条参考线的偏差值

  double priority_cost_ = 0.0;                                                  // 优先级的代价

  PlanningTarget planning_target_;                                              // 规划器的目标地, PlanningTarget在lattice_structure.proto文件中定义的

  ADCTrajectory::TrajectoryType trajectory_type_ = ADCTrajectory::UNKNOWN;      // 轨迹的类型设置为不知道的

  DISALLOW_COPY_AND_ASSIGN(ReferenceLineInfo);                                  // ReferenceLineInfo类禁止了copy构造和直接赋值
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_REFERENCE_LINE_INFO_H_
