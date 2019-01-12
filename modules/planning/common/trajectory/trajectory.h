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
 * @file trajectory.h
 **/

#ifndef MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_H_
#define MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_H_

#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning {
// 基类, 评估函数, 起点,  
class Trajectory {                                                 // 轨迹点的基类
 public:
  Trajectory() = default;                                          // 默认的构造函数

  virtual ~Trajectory() = default;                                 // 默认的虚的析构函数

  virtual common::TrajectoryPoint Evaluate(                        // 通过相对时间点进行插值处理, 得到断点或者是一个新的点
      const double relative_time) const = 0;

  virtual common::TrajectoryPoint StartPoint() const = 0;          // 返回轨迹的起点
  // 获得时间长度
  virtual double GetTemporalLength() const = 0;                    // 返回时间的长度值
  // 获得空间长度
  virtual double GetSpatialLength() const = 0;                     // 返回空间的长度值, 即路程s的长度值
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_H_
