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
 * @file reference_point.cc
 **/

#include "modules/planning/reference_line/reference_point.h"

#include <string>
#include <utility>
#include <vector>

#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;                                                     // 道路上的一个点
using apollo::common::util::StrCat;                                                  // 拷贝一个字符串

namespace {
// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;                                        // 要移除重复点的最小距离
}  // namespace

ReferencePoint::ReferencePoint(const MapPathPoint& map_path_point,                   // 构造参考线的一个点
                               const double kappa, const double dkappa)
    : hdmap::MapPathPoint(map_path_point), kappa_(kappa), dkappa_(dkappa) {}

common::PathPoint ReferencePoint::ToPathPoint(double s) const {                      // 通过s(距离)构造一个路径点
  common::PathPoint path_point = common::util::MakePathPoint(
      x(), y(), 0.0, heading(), kappa_, dkappa_, 0.0);
  path_point.set_s(s);
  return path_point;
}

double ReferencePoint::kappa() const { return kappa_; }                              // 获取中心参考线的曲率

double ReferencePoint::dkappa() const { return dkappa_; }                            // 获取中心参考线曲率的微分

std::string ReferencePoint::DebugString() const {                                    // debug的信息
  // StrCat only support 9 parameters
  return StrCat("{x: ", x(), ", y: ", y(), ", theta: ", heading()) +
         StrCat(", kappa: ", kappa(), ", dkappa: ", dkappa(), "}");
}

void ReferencePoint::RemoveDuplicates(std::vector<ReferencePoint>* points) {         // 移除中心参考线的重复的点
  CHECK_NOTNULL(points);
  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } else {
      (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    }
  }
  points->resize(count);
}

}  // namespace planning
}  // namespace apollo
