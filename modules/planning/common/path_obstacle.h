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

#ifndef MODULES_PLANNING_COMMON_PATH_OBSTACLE_H_
#define MODULES_PLANNING_COMMON_PATH_OBSTACLE_H_

#include <list>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/decision.pb.h"
#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/indexed_list.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

/**
 * @class PathObstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 *
 * The decisions have two categories: lateral decision and longitudinal
 * decision.
 * Lateral decision includes: nudge, ignore.
 * Lateral decision saftey priority: nudge > ignore.
 * Longitudinal decision includes: stop, yield, follow, overtake, ignore.
 * Decision safety priorities order: stop > yield >= follow > overtake > ignore
 *
 * Ignore decision belongs to both lateral decision and longitudinal decision,
 * and it has the lowest priority.
 */
class PathObstacle {                                                         // sl坐标系中的障碍物类, path被映射到sl坐标系中
 public:
  PathObstacle() = default;                                                  // 默认的构造函数
  explicit PathObstacle(const Obstacle* obstacle);                           // 禁止隐式转换的构造函数

  const std::string& Id() const;                                             // 障碍物的id号

  const Obstacle* obstacle() const;                                          // 获取这个障碍物

  /**
   * return the merged lateral decision
   * Lateral decision is one of {Nudge, Ignore}
   **/
  const ObjectDecisionType& LateralDecision() const;                         // 横向的决策是Nudge和Ignore两者的一个

  /**
   * @brief return the merged longitudinal decision
   * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
   **/
  const ObjectDecisionType& LongitudinalDecision() const;                    // 纵向的决策策略

  const std::string DebugString() const;                                     // debug的信息

  const SLBoundary& PerceptionSLBoundary() const;                            // 感知到障碍物的sl边界

  const StBoundary& reference_line_st_boundary() const;                      // 参考线的st边界

  const StBoundary& st_boundary() const;                                     // 障碍物在st坐标系下的边框(boundary)

  const std::vector<std::string>& decider_tags() const;                      // 决策的标签(tags)

  const std::vector<ObjectDecisionType>& decisions() const;                  // 决策的类型

  void AddLongitudinalDecision(const std::string& decider_tag,               // 添加纵向决策的策略
                               const ObjectDecisionType& decision);

  void AddLateralDecision(const std::string& decider_tag,                    // 添加横向决策的策略
                          const ObjectDecisionType& decision);
  bool HasLateralDecision() const;                                           // 是否有横向的决策策略

  void SetStBoundary(const StBoundary& boundary);                            // 设置该障碍物在st坐标系下的边框

  void SetStBoundaryType(const StBoundary::BoundaryType type);               // 设置该边框的类型

  void EraseStBoundary();                                                    // 删除该st坐标下的边框

  void SetReferenceLineStBoundary(const StBoundary& boundary);               // 设置st坐标下的参考线

  void SetReferenceLineStBoundaryType(const StBoundary::BoundaryType type);  // 设置st坐标下的参考线的类型

  void EraseReferenceLineStBoundary();                                       // 删除所有的st坐标下的参考线

  bool HasLongitudinalDecision() const;                                      // 是否有纵向参考策略

  bool HasNonIgnoreDecision() const;                                         // 是否有不能被忽略的决策

  /**
   * @brief Calculate stop distance with the obstacle using the ADC's minimum
   * turning radius
   */                                                                        // 使用自动驾驶的车辆的最小转弯半径计算到障碍物的停止距离
  double MinRadiusStopDistance(const common::VehicleParam& vehicle_param) const;

  /**
   * @brief Check if this object can be safely ignored.
   * The object will be ignored if the lateral decision is ignore and the
   * longitudinal decision is ignore
   *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
   */
  bool IsIgnore() const;                                                     // 是否可以忽略该障碍物
  bool IsLongitudinalIgnore() const;                                         // 从纵向上是否可以忽略该障碍物
  bool IsLateralIgnore() const;                                              // 横向上是否可以忽略该障碍物

  void BuildReferenceLineStBoundary(const ReferenceLine& reference_line,     // 建立一个中心参考线的boundary(边框)
                                    const double adc_start_s);

  void SetPerceptionSlBoundary(const SLBoundary& sl_boundary);               // 设置感知的sl坐标系的边框(boundary)

  /**
   * @brief check if a ObjectDecisionType is a longitudinal decision.
   **/
  static bool IsLongitudinalDecision(const ObjectDecisionType& decision);    // 检查一个决策是否是纵向的决策

  /**
   * @brief check if a ObjectDecisionType is a lateral decision.
   **/
  static bool IsLateralDecision(const ObjectDecisionType& decision);         // 检查一个决策是否是横向的决策

  void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; } // 把该障碍物设置为一个不能通过的障碍物
  bool IsBlockingObstacle() const { return is_blocking_obstacle_; }          // 看看一个障碍物是否是一个阻塞的障碍物

 private:
  FRIEND_TEST(MergeLongitudinalDecision, AllDecisions);
  static ObjectDecisionType MergeLongitudinalDecision(                       // 合并所有纵向的决策
      const ObjectDecisionType& lhs, const ObjectDecisionType& rhs);
  FRIEND_TEST(MergeLateralDecision, AllDecisions);
  static ObjectDecisionType MergeLateralDecision(const ObjectDecisionType& lhs, // 合并所有横向的决策
                                                 const ObjectDecisionType& rhs);

  bool BuildTrajectoryStBoundary(const ReferenceLine& reference_line,           // 通过中心参考线, 起点, 建立一个运行轨迹(trajectory)的st边框
                                 const double adc_start_s,
                                 StBoundary* const st_boundary);
  bool IsValidObstacle(
      const perception::PerceptionObstacle& perception_obstacle);
  std::string id_;                              // 障碍物的地址
  const Obstacle* obstacle_ = nullptr;          // 障碍物的指针
  std::vector<ObjectDecisionType> decisions_;   // 决策的类型
  std::vector<std::string> decider_tags_;       // 决策的名称
  SLBoundary perception_sl_boundary_;           // sl坐标系下的边界

  StBoundary reference_line_st_boundary_;       // 中心参考线的边界
  StBoundary st_boundary_;                      // st坐标系下的边界

  ObjectDecisionType lateral_decision_;         // 侧向的决策类型
  ObjectDecisionType longitudinal_decision_;    // 纵向的决策类型

  bool is_blocking_obstacle_ = false;           // 是否有过不去的障碍物

  double min_radius_stop_distance_ = -1.0;      // 到停止点的最小剩余距离

  struct ObjectTagCaseHash {                    // 对象标记的哈希(这是一个结构体)
    std::size_t operator()(
        const planning::ObjectDecisionType::ObjectTagCase tag) const {
      return static_cast<std::size_t>(tag);
    }
  };

  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>    // 用一个hash表来放侧向的安全决策
      s_lateral_decision_safety_sorter_;
  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>    // 用一个hash来放纵向的值
      s_longitudinal_decision_safety_sorter_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PATH_OBSTACLE_H_
