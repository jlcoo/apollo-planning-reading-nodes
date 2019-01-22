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
 * @file qp_spline_st_graph.h
 **/

#ifndef MODULES_PLANNING_TASKS_QP_SPLINE_ST_SPEED_QP_PIECEWISE_ST_GRAPH_H_
#define MODULES_PLANNING_TASKS_QP_SPLINE_ST_SPEED_QP_PIECEWISE_ST_GRAPH_H_

#include <memory>
#include <utility>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/qp_st_speed_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/string_util.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/math/smoothing_spline/piecewise_linear_generator.h"
#include "modules/planning/toolkits/optimizers/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

class QpPiecewiseStGraph {                                                                     // qp的分段st的图
 public:
  explicit QpPiecewiseStGraph(const QpStSpeedConfig& qp_st_speed_config);                      // 禁止隐式转换的构造函数

  void SetDebugLogger(planning_internal::STGraphDebug* st_graph_debug);                        // 设置debug的日志接口

  common::Status Search(const StGraphData& st_graph_data,
                        SpeedData* const speed_data,                                           // 搜索speed的数据
                        const std::pair<double, double>& accel_bound);

 private:
  void Init();                                                                                 // 进行初始化

  // Add st graph constraint
  common::Status AddConstraint(const common::TrajectoryPoint& init_point,                      // 添加约束项
                               const SpeedLimit& speed_limit,
                               const std::vector<const StBoundary*>& boundaries,
                               const std::pair<double, double>& accel_bound);

  // Add objective function
  common::Status AddKernel(const std::vector<const StBoundary*>& boundaries,                   // 添加目标函数
                           const SpeedLimit& speed_limit);

  // solve
  common::Status Solve();                                                                      // 求解器

  // extract upper lower bound for constraint;
  common::Status GetSConstraintByTime(                                                         // 计算上下界的约束
      const std::vector<const StBoundary*>& boundaries, const double time,
      const double total_path_s, double* const s_upper_bound,
      double* const s_lower_bound) const;

  // generate reference speed profile
  // common::Status ApplyReferenceSpeedProfile();
  common::Status AddCruiseReferenceLineKernel(const SpeedLimit& speed_limit,                   // 产生速度曲线
                                              const double weight);

  common::Status AddFollowReferenceLineKernel(                                                 // 添加核函数
      const std::vector<const StBoundary*>& boundaries, const double weight);

  common::Status EstimateSpeedUpperBound(                                                      // 计算速度的上界
      const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
      std::vector<double>* speed_upper_bound) const;

 private:
  // qp st configuration
  const QpStSpeedConfig qp_st_speed_config_;                                                   // QP的st配置

  // initial status
  common::TrajectoryPoint init_point_;                                                         // 初始化的状态

  // solver
  std::unique_ptr<PiecewiseLinearGenerator> generator_ = nullptr;                              // 求解器

  // evaluated t resolution
  double t_evaluated_resolution_ = 0.0;                                                        // t的分辨率

  // evaluated points
  std::vector<double> t_evaluated_;                                                            // 对应的分辨率

  // reference line kernel
  std::vector<double> cruise_;                                                                 // 参考线的核函数

  planning_internal::STGraphDebug* st_graph_debug_ = nullptr;                                  // planning的内部debug接口
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_QP_SPLINE_ST_SPEED_QP_PIECEWISE_ST_GRAPH_H_
