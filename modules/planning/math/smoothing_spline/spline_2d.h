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
 * @file : spline_2d.h
 * @brief: piecewise smoothing spline 2d class
 **/

#ifndef MODULES_PLANNING_SMOOTHING_SPLINE_SPLINE_2D_H_
#define MODULES_PLANNING_SMOOTHING_SPLINE_SPLINE_2D_H_

#include <utility>
#include <vector>

#include "Eigen/Core"

#include "modules/planning/math/polynomial_xd.h"
#include "modules/planning/math/smoothing_spline/spline_2d_seg.h"

namespace apollo {
namespace planning {
// 2维的spline
class Spline2d {                                                             // spline的曲线
 public:
  Spline2d(const std::vector<double>& t_knots, const uint32_t order);        // 通过时间节点的数组和几次方进行构造
  std::pair<double, double> operator()(const double t) const;                // 重载操作符
  double x(const double t) const;                                            // 设置x
  double y(const double t) const;                                            // 设置y
  double DerivativeX(const double t) const;                                  // 衍生的x
  double DerivativeY(const double t) const;                                  // 衍生的y
  double SecondDerivativeX(const double t) const;                            // 二次微分
  double SecondDerivativeY(const double t) const;
  double ThirdDerivativeX(const double t) const;                             // 三次微分
  double ThirdDerivativeY(const double t) const;
  bool set_splines(const Eigen::MatrixXd& params, const uint32_t order);     // 通过一个矩阵, 和次方数, 设置spline曲线
  const Spline2dSeg& smoothing_spline(const uint32_t index) const;           // 通过索引值index, 找到一个spline的曲线值
  const std::vector<double>& t_knots() const;                                // 时间节点的数组(vector)
  uint32_t spline_order() const;                                             // 返回spline的次方值(order)

 private:
  uint32_t find_index(const double x) const;                                 // 通过x值找到对应的索引值

 private:
  std::vector<Spline2dSeg> splines_;                                         // spline曲线是有一段一段的二维曲线组成
  std::vector<double> t_knots_;                                              // 对应的时间节点
  uint32_t spline_order_;                                                    // spline的次方数
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_SMOOTHING_SPLINE_SPLINE_2D_H_
