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

struct AnchorPoint {
  common::PathPoint path_point;        // PathPoint在pnc_point.proto文件中定义
  double lateral_bound = 0.0;          // 侧边框为0
  double longitudinal_bound = 0.0;     // 纵边框为0
  // enforce smoother to strictly follow this reference point
  bool enforced = false;               // 强制执行参考点
};

class ReferenceLineSmoother {
 public:
  explicit ReferenceLineSmoother(const ReferenceLineSmootherConfig& config)
      : config_(config) {}

  /**
   * Smoothing constraints   平滑约束就是设置锚点
   */
  virtual void SetAnchorPoints(
      const std::vector<AnchorPoint>& achor_points) = 0;

  /**
   * Smooth a given reference line
   */
  virtual bool Smooth(const ReferenceLine&, ReferenceLine* const) = 0;

  virtual ~ReferenceLineSmoother() = default;

 protected:
  ReferenceLineSmootherConfig config_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_H_
