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
 * @file spline_2d_solver.h
 **/

#ifndef MODULES_PLANNING_SMOOTHING_SPLINE_SPLINE_2D_SOLVER_H_
#define MODULES_PLANNING_SMOOTHING_SPLINE_SPLINE_2D_SOLVER_H_

#include <qpOASES.hpp>

#include <memory>
#include <vector>

#include "modules/common/math/qp_solver/qp_solver.h"
#include "modules/planning/math/smoothing_spline/spline_2d.h"
#include "modules/planning/math/smoothing_spline/spline_2d_constraint.h"
#include "modules/planning/math/smoothing_spline/spline_2d_kernel.h"

namespace apollo {
namespace planning {
// 2维spline的求解器
class Spline2dSolver {                                                          // 二维spline的求解器
 public:
  Spline2dSolver(const std::vector<double>& t_knots, const uint32_t order);     // 通过时间节点和几次方进行构造

  void Reset(const std::vector<double>& t_knots, const uint32_t order);         // 重新复位时间节点和次方数

  // customize setup
  Spline2dConstraint* mutable_constraint();                                     // 用户设置二维的约束
  Spline2dKernel* mutable_kernel();                                             // 用户设置二维的kernel函数
  Spline2d* mutable_spline();                                                   // 用户设置二维的spline曲线

  // solve
  bool Solve();                                                                 // 求解过程

  // extract
  const Spline2d& spline() const;                                               // 获取spline的曲线

 private:
  Spline2d spline_;                                                             // spline曲线
  Spline2dKernel kernel_;                                                       // spline的核函数
  Spline2dConstraint constraint_;                                               // spline的约束
  std::unique_ptr<::qpOASES::SQProblem> sqp_solver_;                            // sqp的求解器

  int last_num_constraint_ = 0;                                                 // 上一次约束的数量
  int last_num_param_ = 0;                                                      // 上一次参数的数量
  bool last_problem_success_ = false;                                           // 上一个问题是否已经求解成功
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_SMOOTHING_SPLINE_SPLINE_2D_SOLVER_H_
