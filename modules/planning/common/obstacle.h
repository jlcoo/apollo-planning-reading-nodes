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
 * @file obstacle.h
 **/

#ifndef MODULES_PLANNING_COMMON_OBSTACLE_H_
#define MODULES_PLANNING_COMMON_OBSTACLE_H_

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/common/math/box2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/indexed_list.h"

namespace apollo {
namespace planning {

/**
 * @class Obstacle
 *
 * @brief Obstacle represents one perception obstacle.
 */  // 一个感知的障碍物
class Obstacle {
 public:
  Obstacle() = default;                                                         // 默认的轨迹

  Obstacle(const std::string &id,                                               // 轨迹的id
           const perception::PerceptionObstacle &perception_obstacle);          // 感知到的障碍物

  Obstacle(const std::string &id,                                               // 障碍物的ID
           const perception::PerceptionObstacle &perception,                    // 感知的障碍物
           const prediction::Trajectory &trajectory);                           // 感知障碍物的轨迹

  const std::string &Id() const;                                                // 获得障碍物的ID
  void SetId(const std::string &id) { id_ = id; }                               // 每个障碍物都有一个Id, 设置障碍物的ID

  std::int32_t PerceptionId() const;                                            // 感知障碍物的id

  double Speed() const;                                                         // 返回障碍物的速度

  bool IsStatic() const;        // 静态障碍物
  bool IsVirtual() const;       // 虚拟障碍物

  common::TrajectoryPoint GetPointAtTime(const double time) const;              // 根据时间获取点

  common::math::Box2d GetBoundingBox(
      const common::TrajectoryPoint &point) const;                              // 根据轨迹点获取BOX
  /**
   * @brief get the perception bounding box
   */
  const common::math::Box2d &PerceptionBoundingBox() const; // bounding box边界框

  /**
   * @brief get the perception polygon for the obstacle. It is more precise than
   * bounding box
   */    // 获取感知多边形 perception polygon
  const common::math::Polygon2d &PerceptionPolygon() const;

  const prediction::Trajectory &Trajectory() const;
  common::TrajectoryPoint *AddTrajectoryPoint();
  bool HasTrajectory() const;

  const perception::PerceptionObstacle &Perception() const;

  /**
   * @brief This is a helper function that can create obstacles from prediction
   * data.  The original prediction may have multiple trajectories for each
   * obstacle. But this function will create one obstacle for each trajectory.
   * @param predictions The prediction results
   * @return obstacles The output obstacles saved in a list of unique_ptr.
   */  // unique_ptr指针的单链表中
  static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
      const prediction::PredictionObstacles &predictions);

  static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(                // 创建一个虚拟的障碍物
      const std::string &id, const common::math::Box2d &obstacle_box);

  static bool IsStaticObstacle(
      const perception::PerceptionObstacle &perception_obstacle);               // 判断一个障碍物是否是一个静态障碍物

  static bool IsVirtualObstacle(
      const perception::PerceptionObstacle &perception_obstacle);               // 判断一个障碍物是否是一个虚拟的障碍物

  static bool IsValidTrajectoryPoint(const common::TrajectoryPoint &point);     // 一个轨迹点是否是合法的轨迹点

 private:
  std::string id_;                                                              // 障碍物的ID
  std::int32_t perception_id_ = 0;                                              // 感知的ID
  bool is_static_ = false;                                                      // 默认不是静态障碍物
  bool is_virtual_ = false;                                                     // 默认不是虚拟障碍物
  double speed_ = 0.0;                                                          // 速度初始化时为0.0
  prediction::Trajectory trajectory_;                                           // 每个障碍物都有一个轨迹
  perception::PerceptionObstacle perception_obstacle_;                          // 每个障碍物包含了一个感知到的一个障碍物
  common::math::Box2d perception_bounding_box_;                                 // 感知到的aabox(二维轴对齐的矩形)
  common::math::Polygon2d perception_polygon_;                                  // 感知到的多边形
};

typedef IndexedList<std::string, Obstacle> IndexedObstacles;                    // 用一个列表保存障碍物的名字和对应的障碍物
typedef ThreadSafeIndexedList<std::string, Obstacle> ThreadSafeIndexedObstacles;// 线程安全的列表

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_OBSTACLE_H_
