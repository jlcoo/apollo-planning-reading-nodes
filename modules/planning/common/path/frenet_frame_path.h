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
 * @file frenet_frame_path.h
 **/

#ifndef MODULES_PLANNING_COMMON_PATH_FRENET_FRAME_PATH_H_
#define MODULES_PLANNING_COMMON_PATH_FRENET_FRAME_PATH_H_

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/sl_boundary.pb.h"

namespace apollo {
namespace planning {

class FrenetFramePath {                                                             // frenet坐标系下的path类
 public:
  FrenetFramePath() = default;                                                      // 默认的构造函数
  explicit FrenetFramePath(                                                         // 禁止隐式转换的构造函数
      const std::vector<common::FrenetFramePoint> &sl_points);                      // FrenetFramePoint是sl坐标和l方向的一次微分, 二次微分
  virtual ~FrenetFramePath() = default;                                             // 默认的构造函数

  void set_points(const std::vector<common::FrenetFramePoint> &points);             // 设置sl坐标系下path的点
  const std::vector<common::FrenetFramePoint> &points() const;                      // 获取只读的sl坐标系中的path的点集
  std::uint32_t NumOfPoints() const;                                                // sl坐标系中点的个数
  double Length() const;
  const common::FrenetFramePoint &PointAt(const std::uint32_t index) const;         // 通过索引值index进行访问frenet坐标系中的点
  common::FrenetFramePoint EvaluateByS(const double s) const;                       // 通过一个路程进行估计一个sl坐标系中的一个点(其实就是进行插值处理)

  /**
   * @brief Get the FrenetFramePoint that is within SLBoundary, or the one with
   * smallest l() in SLBoundary's s range [start_s(), end_s()]
   */                                          //SLBoundary在sl_boundary.proto文件中定义
  common::FrenetFramePoint GetNearestPoint(const SLBoundary &sl) const;             // 在一个sl boundary中获取一个点, 或者返回一个最近的点

  virtual void Clear();                                                             // 清除数组中的内容

 private:   // 私有的static函数
  static bool LowerBoundComparator(const common::FrenetFramePoint &p,               // 所有对象所共有的数据
                                   const double s) {                                // s点处的下界
    return p.s() < s;
  }
  static bool UpperBoundComparator(const double s,                                  // s点处的上界
                                   const common::FrenetFramePoint &p) {
    return s < p.s();
  }
                                                                                    // frenet坐标系中的点集
  std::vector<common::FrenetFramePoint> points_;                                    // 都是用vector来存储的散点
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PATH_FRENET_FRAME_PATH_H_
