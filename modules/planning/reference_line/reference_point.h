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
 * @file reference_point.h
 **/

#ifndef MODULES_PLANNING_REFERENCE_LINE_REFERENCE_POINT_H_
#define MODULES_PLANNING_REFERENCE_LINE_REFERENCE_POINT_H_

#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/map/pnc_map/path.h"

namespace apollo {
namespace planning {  // 重要的信息主要是在path.h和path.cc文件中
// 参考线的点, 二维向量                                                                  // MapPathPoint继承于LaneWaypoint(内部有一个LaneInfoConstPtr的指针和一个sl坐标)
class ReferencePoint : public hdmap::MapPathPoint {                                   // 参考线的点, MapPathPoint继承于二维向量点common::math::Vec2d
 public:                                                                              // 所以ReferencePoint应该也有sl坐标和lane的相关信息, 除此之外还有, 航向角, 曲率和曲率的微分
  ReferencePoint() = default;                                                         // 默认的构造函数

  ReferencePoint(const MapPathPoint& map_path_point, const double kappa,              // 通过地图中的道路点, 曲率和曲率的微分进行构造中心参考线
                 const double dkappa);

  common::PathPoint ToPathPoint(double s) const;                                      // 将s距离上的一个点转换为道路上的一个点

  double kappa() const;                                                               // 获取中心参考线上一点的曲率
  double dkappa() const;                                                              // 获取中心参考线上一点的曲率的微分

  std::string DebugString() const;                                                    // debug得到的一些信息

  static void RemoveDuplicates(std::vector<ReferencePoint>* points);                  // 移除中心参考线中的一些重复的点

 private:
  double kappa_ = 0.0;                                                                // 曲率
  double dkappa_ = 0.0;                                                               // 曲率的微分
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_REFERENCE_POINT_H_
