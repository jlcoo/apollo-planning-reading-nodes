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
 * @file : piecewise_linear_constraint.h
 * @brief: Definition of PiecewiseLinearConstraint class.
 **/

#ifndef MODULES_PLANNING_MATH_SMOOTHING_SPLINE_PIECEWISE_LINEAR_CONSTRAINT_H_
#define MODULES_PLANNING_MATH_SMOOTHING_SPLINE_PIECEWISE_LINEAR_CONSTRAINT_H_

#include <vector>

#include "Eigen/Core"

namespace apollo {
namespace planning {

class PiecewiseLinearConstraint {                                                // 线段的线性约束
 public:
  PiecewiseLinearConstraint(const uint32_t dimension,                            // 构造函数, 维度和单位的片段
                            const double unit_segment);
  virtual ~PiecewiseLinearConstraint() = default;                                // 析构函数


  Eigen::MatrixXd inequality_constraint_matrix() const;                          // 不等式约束矩阵
  Eigen::MatrixXd inequality_constraint_boundary() const;                        // 不等式的boundary
  Eigen::MatrixXd equality_constraint_matrix() const;                            // 等式的约束矩阵
  Eigen::MatrixXd equality_constraint_boundary() const;                          // 不等式的约束boundary

  /**
   * @brief: inequality boundary constraints
   **/  // 不等式的线性约束
  bool AddBoundary(const std::vector<uint32_t>& index_list,
                   const std::vector<double>& lower_bound,
                   const std::vector<double>& upper_bound);                      // 添加一个boundary(边框)
  bool AddDerivativeBoundary(const std::vector<uint32_t>& index_list,            // 添加微分边界
                             const std::vector<double>& lower_bound,
                             const std::vector<double>& upper_bound);
  bool AddSecondDerivativeBoundary(const double init_derivative,                 // 添加二次微分边界
                                   const std::vector<uint32_t>& index_list,
                                   const std::vector<double>& lower_bound,
                                   const std::vector<double>& upper_bound);

  /**
   * @brief: equality constraints
   **/    // 等式的点约束
  bool AddPointConstraint(const uint32_t index, const double val);               // 添加一个点的约束
  bool AddPointDerivativeConstraint(const uint32_t index, const double val);     // 添加一个衍生的约束

  /**
   * @brief: Add monotone constraint inequality at all indices
   **/  // 单调不等式约束
  bool AddMonotoneInequalityConstraint();                                        // 添加一个单调不等式的约束

 private:
  const uint32_t dimension_;                                                     // 维度
  const double unit_segment_;                                                    // 单位的片段
  std::vector<Eigen::MatrixXd> inequality_matrices_;                             // 不等式的矩阵
  std::vector<Eigen::MatrixXd> inequality_boundaries_;                           // 不等式的boundary(边框)

  std::vector<Eigen::MatrixXd> equality_matrices_;                               // 等式的矩阵
  std::vector<Eigen::MatrixXd> equality_boundaries_;                             // 等式的boundary
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_MATH_SMOOTHING_SPLINE_PIECEWISE_LINEAR_CONSTRAINT_H_
