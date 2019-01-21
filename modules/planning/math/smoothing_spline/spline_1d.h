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
 * @file : spline_1d.h
 * @brief: piecewise smoothing spline class
 *       1. Model description: piecewise smoothing spline are made by pieces of
 *smoothing splines
 *          joint at knots;
 *       2. To guarantee smoothness, pieces value at knots should joint together
 *with
 *           same value, derivative, and etc. Higher the order, More smoothness
 *the piecewise spline;
 **/

#ifndef MODULES_PLANNING_MATH_SMOOTHING_SPLINE_SPLINE_1D_H_
#define MODULES_PLANNING_MATH_SMOOTHING_SPLINE_SPLINE_1D_H_

#include <vector>

#include "Eigen/Core"

#include "modules/planning/math/polynomial_xd.h"
#include "modules/planning/math/smoothing_spline/affine_constraint.h"
#include "modules/planning/math/smoothing_spline/spline_1d_seg.h"

namespace apollo {
namespace planning {
// 做一维的spline
class Spline1d {
 public:
  Spline1d(const std::vector<double>& x_knots, const uint32_t order);                   // spline曲线的值

  // @brief: given x return f(x) value, derivative, second order derivative and
  // the third order;
  double operator()(const double x) const;                                              // 返回f(x)的值
  double Derivative(const double x) const;                                              // 一次微分
  double SecondOrderDerivative(const double x) const;                                   // 二次微分
  double ThirdOrderDerivative(const double x) const;                                    // 三次微分

  // @brief: set spline segments
  bool SetSplineSegs(const Eigen::MatrixXd& param_matrix, const uint32_t order);        // 设置spline的片段

  const std::vector<double>& x_knots() const;                                           // x的节点个数
  uint32_t spline_order() const;                                                        // spline的次方数

 private:
  uint32_t FindIndex(const double x) const;                                             // 通过一个值找到对应的insdex的引用值

 private:
  std::vector<Spline1dSeg> splines_;                                                    // spline通过一个一个片段组成
  std::vector<double> x_knots_;                                                         // x的节点值(放到一个数组中)
  uint32_t spline_order_;                                                               // spline的次方数
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_MATH_SMOOTHING_SPLINE_SPLINE_1D_H_
