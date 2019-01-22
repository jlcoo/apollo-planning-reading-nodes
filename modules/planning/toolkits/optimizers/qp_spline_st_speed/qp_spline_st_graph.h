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

#ifndef MODULES_PLANNING_TASKS_QP_SPLINE_ST_SPEED_QP_SPLINE_ST_GRAPH_H_
#define MODULES_PLANNING_TASKS_QP_SPLINE_ST_SPEED_QP_SPLINE_ST_GRAPH_H_

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
#include "modules/planning/math/smoothing_spline/spline_1d_generator.h"
#include "modules/planning/toolkits/optimizers/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

class QpSplineStGraph {                                                                  // QP相关的st图(spline)
 public:
  QpSplineStGraph(Spline1dGenerator* spline_generator,
                  const QpStSpeedConfig& qp_st_speed_config,
                  const apollo::common::VehicleParam& veh_param,
                  const bool is_change_lane);                                            // 默认的构造函数

  void SetDebugLogger(planning_internal::STGraphDebug* st_graph_debug);                  // 设置debug的log信息

  common::Status Search(const StGraphData& st_graph_data,                                // 在st图中进行speed data的搜索
                        const std::pair<double, double>& accel_bound,
                        const SpeedData& reference_speed_data,
                        SpeedData* const speed_data);

 private:
  void Init();                                                                           // 私有的初始化函数

  // Add st graph constraint
  common::Status AddConstraint(const common::TrajectoryPoint& init_point,                // 添加qp的约束项
                               const SpeedLimit& speed_limit,
                               const std::vector<const StBoundary*>& boundaries,
                               const std::pair<double, double>& accel_bound);

  // Add objective function
  common::Status AddKernel(const std::vector<const StBoundary*>& boundaries,             // 添加目标函数, 即二次规划的核函数
                           const SpeedLimit& speed_limit);

  // solve
  common::Status Solve();                                                                // 求解函数值

  // extract upper lower bound for constraint;
  common::Status GetSConstraintByTime(                                                   // 计算约束项的上界和下界
      const std::vector<const StBoundary*>& boundaries, const double time,
      const double total_path_s, double* const s_upper_bound,
      double* const s_lower_bound) const;

  // reference line kernel is a constant s line at s = 250m
  common::Status AddCruiseReferenceLineKernel(const double weight);                      // 参考中心线的常量s等于250m

  // follow line kernel
  common::Status AddFollowReferenceLineKernel(                                           // 跟车的核函数
      const std::vector<const StBoundary*>& boundaries, const double weight);

  // yield line kernel
  common::Status AddYieldReferenceLineKernel(                                            // 避让车辆的核函数
      const std::vector<const StBoundary*>& boundaries, const double weight);

  const SpeedData GetHistorySpeed() const;                                               // 历史速度的speed data
  common::Status EstimateSpeedUpperBound(                                                // 计算速度的上界, 保存在double的数组中
      const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
      std::vector<double>* speed_upper_bound) const;

  bool AddDpStReferenceKernel(const double weight) const;                                // 增加dp中心参考线的核函数

 private:
  // solver
  Spline1dGenerator* spline_generator_ = nullptr;                                        // 1维的spline求解器

  // qp st configuration
  const QpStSpeedConfig qp_st_speed_config_;                                             // QP二次规划的配置参数

  // initial status
  common::TrajectoryPoint init_point_;                                                   // 轨迹的起点

  // is change lane
  bool is_change_lane_ = false;                                                          // 是否进行变道

  // t knots resolution
  double t_knots_resolution_ = 0.0;                                                      // 时间的节点数t的刻度个数

  // knots
  std::vector<double> t_knots_;                                                          // 每个刻度的值(t)

  // evaluated t resolution
  double t_evaluated_resolution_ = 0.0;                                                  // t的分辨率

  // evaluated points
  std::vector<double> t_evaluated_;                                                      // t分辨率的刻度保存在数组中

  // reference line kernel
  std::vector<double> cruise_;                                                           // 参考线的核函数, cruise是巡航的意思

  // reference st points from dp optimizer
  std::vector<common::SpeedPoint> reference_dp_speed_points_;                            // 参考线在st坐标系中的点?

  planning_internal::STGraphDebug* st_graph_debug_ = nullptr;                            // 内部debug的指针
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_QP_SPLINE_ST_SPEED_QP_SPLINE_ST_GRAPH_H_
