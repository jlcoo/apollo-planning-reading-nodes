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

#ifndef MODULES_PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_
#define MODULES_PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_

#include <vector>

#include "modules/planning/proto/planning.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/planning/common/trajectory/trajectory.h"

namespace apollo {
namespace planning {
// 离散的轨迹, 基于protobuf                                                           // Trajectory类是在prediction_obstacle.proto文件中定义的
class DiscretizedTrajectory : public Trajectory {                                   // 离散的轨迹点继承与轨迹点
 public:
  DiscretizedTrajectory() = default;                                                // 默认的构造函数

  /**
   * Create a DiscretizedTrajectory based on protobuf message
   */                                                           // 基于protobuf的消息创建一个离散的轨迹
  explicit DiscretizedTrajectory(const ADCTrajectory& trajectory);                  // 构造函数
                                                                // explicit修饰的函数禁止隐式转换
  explicit DiscretizedTrajectory(
      const std::vector<common::TrajectoryPoint>& trajectory_points);               // 通过所有的轨迹点进行构造

  void SetTrajectoryPoints(
      const std::vector<common::TrajectoryPoint>& trajectory_points);               // 通过一个数组设置轨迹点

  virtual ~DiscretizedTrajectory() = default;                                       // 析构函数

  common::TrajectoryPoint StartPoint() const override;                              // 轨迹的起点

  double GetTemporalLength() const override;                                        // 得到时间长度

  double GetSpatialLength() const override;                                         // 获得空间长度

  common::TrajectoryPoint Evaluate(const double relative_time) const override;      // 评估某个时刻的轨迹

  virtual uint32_t QueryLowerBoundPoint(const double relative_time) const;          // 查询某个时刻的下界

  virtual uint32_t QueryNearestPoint(const common::math::Vec2d& position) const;    // 查询某个点的位置

  virtual void AppendTrajectoryPoint(                                               // 向轨迹中添加一个轨迹点
      const common::TrajectoryPoint& trajectory_point);

  template <typename Iter>
  void PrependTrajectoryPoints(Iter begin, Iter end) {                              // 用一个模板函数预先设定轨迹点
    if (!trajectory_points_.empty() && begin != end) {
      CHECK((end - 1)->relative_time() <
            trajectory_points_.front().relative_time());
    }
    trajectory_points_.insert(trajectory_points_.begin(), begin, end);
  }

  const common::TrajectoryPoint& TrajectoryPointAt(                                  // 在索引点的轨迹
      const std::uint32_t index) const;

  uint32_t NumOfPoints() const;                                                      // 轨迹中的所有的点

  const std::vector<common::TrajectoryPoint>& trajectory_points() const;             // 返回只读的轨迹点
  std::vector<common::TrajectoryPoint>& trajectory_points();                         // 返回可以改变的轨迹点

  virtual void Clear();                                                              // 把一个离散的轨迹全部清空

 protected:
  std::vector<common::TrajectoryPoint> trajectory_points_;                           // 用一个vector存放TrajectoryPoint
};

inline std::uint32_t DiscretizedTrajectory::NumOfPoints() const {
  return trajectory_points_.size();                                                  // 点数就是返回数组的大小
}

inline const std::vector<common::TrajectoryPoint>&
DiscretizedTrajectory::trajectory_points() const {   // 重载const函数
  return trajectory_points_;
}

inline std::vector<common::TrajectoryPoint>&
DiscretizedTrajectory::trajectory_points() {
  return trajectory_points_;
}

inline void DiscretizedTrajectory::SetTrajectoryPoints(
    const std::vector<common::TrajectoryPoint>& trajectory_points) {
  trajectory_points_ = trajectory_points;
}

inline void DiscretizedTrajectory::Clear() { trajectory_points_.clear(); }           // 将数组清空

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_
