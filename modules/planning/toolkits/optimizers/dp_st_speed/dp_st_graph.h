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
 * @file dp_st_graph.h
 **/

#ifndef MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_GRAPH_H_
#define MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_GRAPH_H_

#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/dp_st_speed_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/toolkits/optimizers/dp_st_speed/dp_st_cost.h"
#include "modules/planning/toolkits/optimizers/dp_st_speed/st_graph_point.h"
#include "modules/planning/toolkits/optimizers/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

class DpStGraph {                                                                           // st坐标系下的图
 public:
  DpStGraph(const StGraphData& st_graph_data, const DpStSpeedConfig& dp_config,             // 速度做动态规划的st图中的数据, 速度动态规划的配置参数
            const std::vector<const PathObstacle*>& obstacles,                              // path上的障碍物
            const common::TrajectoryPoint& init_point,                                      // 初始点(轨迹的起点)
            const SLBoundary& adc_sl_boundary);                                             // 自动驾驶车辆的sl坐标系中的boundary

  apollo::common::Status Search(SpeedData* const speed_data);                               // 要搜索的速度数据

 private:
  apollo::common::Status InitCostTable();                                                   // 初始化代价函数

  apollo::common::Status RetrieveSpeedProfile(SpeedData* const speed_data);                 // 纠正速度的曲线

  apollo::common::Status CalculateTotalCost();                                              // 计算一共的代价函数值
  void CalculateCostAt(const uint32_t r, const uint32_t c);                                 // 计算代价值

  float CalculateEdgeCost(const STPoint& first, const STPoint& second,                      // 计算一条边的代价值
                           const STPoint& third, const STPoint& forth,
                           const float speed_limit);                                        // 速度额限制
  float CalculateEdgeCostForSecondCol(const uint32_t row,
                                       const float speed_limit);                            // 二次什么?
  float CalculateEdgeCostForThirdCol(const uint32_t curr_r,
                                      const uint32_t pre_r,
                                      const float speed_limit);

  void GetRowRange(const StGraphPoint& point, int* highest_row,                             // 获取行的range(范围)
                   int* lowest_row);

 private:
  const StGraphData& st_graph_data_;                                                        // st图中的数据

  // dp st configuration
  DpStSpeedConfig dp_st_speed_config_;                                                      // st动态规划的配置参数

  // obstacles based on the current reference line
  const std::vector<const PathObstacle*>& obstacles_;                                       // 在当前中心参考线上的障碍物

  // vehicle configuration parameter
  const common::VehicleParam& vehicle_param_ =                                              // 车辆的配置参数
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  // initial status
  common::TrajectoryPoint init_point_;                                                      // 轨迹的起点

  // cost utility with configuration;
  DpStCost dp_st_cost_;                                                                     // 代价函数的配置

  const SLBoundary& adc_sl_boundary_;                                                       // 自动驾驶车辆的sl边框

  float unit_s_ = 0.0;                                                                      // s距离
  float unit_t_ = 0.0;                                                                      // 时间t

  // cost_table_[t][s]                                                                      // 动态规划的二维数组
  // row: s, col: t --- NOTICE: Please do NOT change.
  std::vector<std::vector<StGraphPoint>> cost_table_;                                       // 二维数组， 里面的的是st的点
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_GRAPH_H_
