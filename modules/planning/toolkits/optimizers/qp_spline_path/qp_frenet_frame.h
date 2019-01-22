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
 * @file qp_frenet_frame.h
 * @brief: natural coordinate system
 **/

#ifndef MODULES_PLANNING_TASKS_QP_SPLINE_PATH_QP_FRENET_FRAME_H_
#define MODULES_PLANNING_TASKS_QP_SPLINE_PATH_QP_FRENET_FRAME_H_

#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Core"

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"

#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class QpFrenetFrame {                                                                        // qp坐标系中frenet数据框
 public:
  QpFrenetFrame(const ReferenceLine& reference_line,                                         // 构造函数
                const SpeedData& speed_data,
                const common::FrenetFramePoint& init_frenet_point,
                const double time_resolution,
                const std::vector<double>& evaluated_s);
  virtual ~QpFrenetFrame() = default;                                                        // 析构函数

  bool Init(const std::vector<const PathObstacle*>& path_obstacles);                         // 利用path上的障碍物进行初始化

  void LogQpBound(apollo::planning_internal::Debug* planning_debug);                         // debug的接口

  const std::vector<std::pair<double, double>>& GetMapBound() const;                         // 获得map的bound
  const std::vector<std::pair<double, double>>& GetStaticObstacleBound() const;              // 获得静态障碍物的boundary
  const std::vector<std::pair<double, double>>& GetDynamicObstacleBound() const;             // 获得动态障碍物的boundary

 private:
  bool CalculateDiscretizedVehicleLocation();                                                // 计算车辆的位置

  bool MapDynamicObstacleWithDecision(const PathObstacle& path_obstacle);                    // 映射动态障碍物的决策

  bool MapStaticObstacleWithDecision(const PathObstacle& path_obstacle);                     // 映射静态障碍物的决策

  bool MapNudgePolygon(const common::math::Polygon2d& polygon,
                       const ObjectNudge& nudge,
                       std::vector<std::pair<double, double>>* const bound_map);             // nudge的多项式

  bool MapNudgeLine(const common::SLPoint& start, const common::SLPoint& end,
                    const ObjectNudge::Type type,
                    std::vector<std::pair<double, double>>* const constraint);               // 线

  std::pair<double, double> MapLateralConstraint(                                            // 横向的约束
      const common::SLPoint& start, const common::SLPoint& end,
      const ObjectNudge::Type nudge_type, const double s_start,
      const double s_end);

  std::pair<uint32_t, uint32_t> FindInterval(const double start,                             // 找到内部值
                                             const double end) const;

  bool CalculateHDMapBound();                                                                // 计算高精地图的bound

  bool CalculateObstacleBound(
      const std::vector<const PathObstacle*>& path_obstacles);

  uint32_t FindIndex(const double s) const;                                                  // 通过一个s找到对应点的index(索引值)

 private:
  const ReferenceLine& reference_line_;                                                      // 中心参考点
  const SpeedData& speed_data_;                                                              // 速度的数据

  common::VehicleParam vehicle_param_;                                                       // 车辆的参数
  common::FrenetFramePoint init_frenet_point_;                                               // 起点的值(frenet坐标系中计算)

  double feasible_longitudinal_upper_bound_ = 0.0;                                           // 纵向的上界
  double start_s_ = 0.0;                                                                     // 起点的s， 和终点的s
  double end_s_ = 0.0;
  double time_resolution_ = 0.1;                                                             // 时间的分辨率

  std::vector<double> evaluated_s_;                                                          // s的评估值
  std::vector<common::SpeedPoint> discretized_vehicle_location_;                             // 车辆的离散点
  std::vector<std::pair<double, double>> hdmap_bound_;                                       // 高精地图的bound
  std::vector<std::pair<double, double>> static_obstacle_bound_;                             // 静态障碍物的bound
  std::vector<std::pair<double, double>> dynamic_obstacle_bound_;                            // 动态障碍物的bound
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_QP_SPLINE_PATH_QP_FRENET_FRAME_H_
