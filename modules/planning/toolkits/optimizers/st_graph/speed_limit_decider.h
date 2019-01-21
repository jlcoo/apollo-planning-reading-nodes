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
 *   @file
 **/

#ifndef MODULES_PLANNING_TASKS_ST_GRAPH_SPEED_LIMIT_DECIDER_H_
#define MODULES_PLANNING_TASKS_ST_GRAPH_SPEED_LIMIT_DECIDER_H_

#include <string>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/st_boundary_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class SpeedLimitDecider {                                                          // 速度限制的决策者
 public:
  SpeedLimitDecider(const SLBoundary& adc_sl_boundary,                             // 自动驾驶车辆在sl坐标系中的边框
                    const StBoundaryConfig& config,                                // st的boundary配置者
                    const ReferenceLine& reference_line,                           // 中心参考线
                    const PathData& path_data);                                    // path上的数据

  virtual ~SpeedLimitDecider() = default;                                          // 析构函数

  virtual apollo::common::Status GetSpeedLimits(                                   // 获取速度的限制, 输出是一个SpeedLimit的指针
      const IndexedList<std::string, PathObstacle>& path_obstacles,                // 输入是path上的障碍物
      SpeedLimit* const speed_limit_data) const;

 private:
  FRIEND_TEST(SpeedLimitDeciderTest, get_centric_acc_limit);                       // 做测试(获得中心点的加速度限制??)
  double GetCentricAccLimit(const double kappa) const;                             // 通过kappa获得加速度

  void GetAvgKappa(const std::vector<common::PathPoint>& path_points,              // 输入path上的点, 获得对应点上的Kappa值(曲率值)
                   std::vector<double>* kappa) const;

 private:
  const SLBoundary& adc_sl_boundary_;                                              // 自动驾驶车辆的sl boundary(边框)
  const StBoundaryConfig& st_boundary_config_;                                     // st 边框(boundary)的配置
  const ReferenceLine& reference_line_;                                            // 中心参考线
  const PathData& path_data_;                                                      // path上的数据
  const apollo::common::VehicleParam& vehicle_param_;                              // 车辆的参数
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_ST_GRAPH_SPEED_LIMIT_DECIDER_H_
