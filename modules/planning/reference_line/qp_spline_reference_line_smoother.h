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
 * @file qp_spline_reference_line_smoother.h
 **/

#ifndef MODULES_PLANNING_REFERENCE_LINE_QP_SPLINE_REFERENCE_LINE_SMOOTHER_H_
#define MODULES_PLANNING_REFERENCE_LINE_QP_SPLINE_REFERENCE_LINE_SMOOTHER_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/reference_line_smoother_config.pb.h"

#include "modules/planning/math/smoothing_spline/spline_2d_solver.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_line_smoother.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class QpSplineReferenceLineSmoother : public ReferenceLineSmoother {                  // qp(二次规划)的平滑器, 继承于中心参考线的平滑器
 public:
  explicit QpSplineReferenceLineSmoother(                                             // 禁止隐式转换的构造函数
      const ReferenceLineSmootherConfig& config);                                     // 通过平滑器的配置参数进行构造

  virtual ~QpSplineReferenceLineSmoother() = default;                                 // 默认的虚的构造函数, 意思是可以实现多态

  bool Smooth(const ReferenceLine& raw_reference_line,                                // 输入一个原始的中心参考线, 输出一个平滑后的中心参考线
              ReferenceLine* const smoothed_reference_line) override;

  void SetAnchorPoints(const std::vector<AnchorPoint>& achor_points) override;        // 设置锚点

 private:
  void Clear();                                                                       // 清理掉所有的信息

  bool Sampling();                                                                    // 采样

  bool AddConstraint();                                                               // 添加一个约束项

  bool AddKernel();                                                                   // 添加一个kernel(内核)进行计算

  bool Solve();                                                                       // 得到函数的解

  bool ExtractEvaluatedPoints(                                                        // 提取计算获得的点
      const ReferenceLine& raw_reference_line, const std::vector<double>& vec_t,      // 原始的中心参考线, 向量点
      std::vector<common::PathPoint>* const path_points) const;                       // 输出是path point组成的数组???

  bool GetSFromParamT(const double t, double* const s) const;                         // 通过时间t, 获取一个路程s

  std::uint32_t FindIndex(const double t) const;                                      // 通过时间点t, 找到一个index(索引值)

 private:
  std::vector<double> t_knots_;   // 一个结点？                                         // 时间节点的数组
  std::vector<AnchorPoint> anchor_points_;                                            // 锚点的数组(一个锚点的意思是横纵坐标的bound(边界), path point)
  std::unique_ptr<Spline2dSolver> spline_solver_;                                     // 二维spline的求解器, 独享指针

  double ref_x_ = 0.0;                                                                // reference在x轴上的坐标
  double ref_y_ = 0.0;                                                                // reference在y轴上的坐标
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_QP_SPLINE_REFERENCE_LINE_SMOOTHER_H_
