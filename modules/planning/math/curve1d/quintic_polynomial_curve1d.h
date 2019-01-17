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
 * @file quintic_polynomial_curve1d.h
 **/

#ifndef MODULES_PLANNING_MATH_CURVE1D_QUINTIC_POLYNOMIAL_CURVE1D_H_
#define MODULES_PLANNING_MATH_CURVE1D_QUINTIC_POLYNOMIAL_CURVE1D_H_

#include <array>
#include <string>

#include "modules/planning/math/curve1d/polynomial_curve1d.h"

namespace apollo {
namespace planning {

// 1D quintic polynomial curve:
// (x0, dx0, ddx0) -- [0, param] --> (x1, dx1, ddx1)   // 5次多项式的曲线
class QuinticPolynomialCurve1d : public PolynomialCurve1d {                               // 多项式的曲线
 public:
  QuinticPolynomialCurve1d() = default;                                                   // 默认的构造函数

  QuinticPolynomialCurve1d(const std::array<double, 3>& start,                            // 通过三个点的起点和3个点的终点， 还有就是参数(param)中进行构造
                           const std::array<double, 3>& end,
                           const double param);

  QuinticPolynomialCurve1d(const double x0, const double dx0, const double ddx0,          // 起点的x, x的一次微分， x的二次微分
                           const double x1, const double dx1, const double ddx1,          // 终点的x1, x1的一次微分, x1的二次微分
                           const double param);                                           // 参数prama是什么意思呢?

  QuinticPolynomialCurve1d(const QuinticPolynomialCurve1d& other);                        // 通过另外的一条曲线进行构造

  virtual ~QuinticPolynomialCurve1d() = default;                                          // 析构函数

  double Evaluate(const std::uint32_t order, const double p) const override;              // order应该是第几个参数?

  double ParamLength() const { return param_; }                                           // 参数的长度就是param_(在polynomial_curve1d中定义)
  std::string ToString() const override;                                                  // debug的string信息

 protected:
  void ComputeCoefficients(const double x0, const double dx0, const double ddx0,          // 通过起点的x0, x0的一次微分和二次微分
                           const double x1, const double dx1, const double ddx1,          // 通过终点的x1，x1的一次微分和二次微分
                           const double param);                                           // param是5次多项式平滑的值？
  // i等于0， 才是常数项
  // f = sum(coef_[i] * x^i), i from 0 to 5                                               // f就是5次多项式的和(参数x的0次方到5次方的和， 每项前面应该有一个参数项)
  std::array<double, 6> coef_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> start_condition_{{0.0, 0.0, 0.0}};                                // 开始的条件
  std::array<double, 3> end_condition_{{0.0, 0.0, 0.0}};                                  // 结束的条件
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_MATH_CURVE1D_QUINTIC_POLYNOMIAL_CURVE1D_H_
