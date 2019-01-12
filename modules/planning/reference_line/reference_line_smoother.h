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

#ifndef MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_H_
#define MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_H_

#include <vector>

#include "modules/planning/proto/reference_line_smoother_config.pb.h"

#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {
                                       // 一个锚点就是path上的点, 横向的边框和纵向的边框, 
struct AnchorPoint {
  common::PathPoint path_point;        // PathPoint在pnc_point.proto文件中定义
  double lateral_bound = 0.0;          // 侧边框为0
  double longitudinal_bound = 0.0;     // 纵边框为0
  // enforce smoother to strictly follow this reference point
  bool enforced = false;               // 强制执行参考点, 让其变平滑
};
// 参考线的平滑器中定义的都是一些接口
class ReferenceLineSmoother {                                                       // 道路中心参考线的平滑器
 public:
  explicit ReferenceLineSmoother(const ReferenceLineSmootherConfig& config)         // 通过平滑器的配置进行构造
      : config_(config) {}

  /**
   * Smoothing constraints   平滑约束就是设置锚点
   */
  virtual void SetAnchorPoints(                                                     // 锚点的数组就是平滑的约束点??
      const std::vector<AnchorPoint>& achor_points) = 0;

  /**
   * Smooth a given reference line
   */
  virtual bool Smooth(const ReferenceLine&, ReferenceLine* const) = 0;              // 输入一个中心参考线, 然后通过smooth函数进行平滑处理

  virtual ~ReferenceLineSmoother() = default;                                       // 默认的析构函数

 protected:
  ReferenceLineSmootherConfig config_;                                              // 中心参考线平滑的配置项, 在reference_line_config.proto文件中定义
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_H_
