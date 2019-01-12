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
 * @file discretized_path.h
 **/

#ifndef MODULES_PLANNING_COMMON_PATH_DISCRETIZED_PATH_H_
#define MODULES_PLANNING_COMMON_PATH_DISCRETIZED_PATH_H_

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning {

class DiscretizedPath {    // 离散道路
 public:
  DiscretizedPath() = default;                                                   // 默认的构造函数
                                                                                 // 禁止隐式转换的构造函数
  explicit DiscretizedPath(const std::vector<common::PathPoint>& path_points);   // 从PathPoint中得到数据(path point里面有三维空间坐标(x,y,z), theta, 路程s,曲率等信息)

  virtual ~DiscretizedPath() = default;                                          // 默认的析构函数

  void set_path_points(const std::vector<common::PathPoint>& path_points);       // 通过一个PathPoint的数组设置path里面的点

  double Length() const;                                                         // 获取长度信息

  const common::PathPoint& StartPoint() const;                                   // 获取一段path的起点

  const common::PathPoint& EndPoint() const;                                     // 获取该path的终点

  common::PathPoint Evaluate(const double path_s) const;                         // 通过一个path上的路程估计出一个path point

  const std::vector<common::PathPoint>& path_points() const;                     // 返回一个path点组成的数组(注意这里是引用)

  std::uint32_t NumOfPoints() const;                                             // 返回数组中的个数

  virtual void Clear();                                                          // 清空后数组

 protected:
  std::vector<common::PathPoint>::const_iterator QueryLowerBound(                // 通过path上的路程path_s返回小于s的所有点?
      const double path_s) const;
// PathPoint是pnc_point.proto文件中定义的消息字段
  std::vector<common::PathPoint> path_points_;                                   // PathPoint是什么数据结构?(就是一些坐标, 曲率等信息)
};                                                                               // 一个path里面有很多path点, 用一个数组进行保存

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PATH_PATH_H_
