/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PLANNING_STD_PLANNING_H_
#define MODULES_PLANNING_STD_PLANNING_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/common/util/thread_pool.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/planner/std_planner_dispatcher.h"
#include "modules/planning/planning_base.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class StdPlanning : public PlanningBase {
 public:
  StdPlanning() {   // 根据GPS信号和IMU的输入得到planning的信息， StdPlannerDispatcher是std planning的调度员
    planner_dispatcher_ = std::make_unique<StdPlannerDispatcher>(); // 构造成一个stdplanner的空对象，在构造函数指定了使用std相关的planner(规划器)的提供者
    // Dispatcher是调度的意思
  }
  virtual ~StdPlanning();                                           // 析构函数

  /**
   * @brief Planning algorithm name.
   */
  std::string Name() const override;                                // planning算法的名字

  /**
   * @brief module initialization function
   * @return initialization status                                  // 返回初始化状态
   */
  apollo::common::Status Init() override;                           // 初始化函数

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start() override;                          // planning开始的函数

  /**
   * @brief module stop function
   */
  void Stop() override;                                             // planning的停止函数

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  void RunOnce() override;                                          // planning的主要逻辑, planning由定时器触发

  void OnTimer(const ros::TimerEvent&) override;                    // 创建一个定时器

  apollo::common::Status Plan(                                      // planning做规划的函数
      const double current_time_stamp,                              // 现在的时间戳
      const std::vector<common::TrajectoryPoint>& stitching_trajectory, // 合并的轨迹
      ADCTrajectory* trajectory) override;                          // 自动驾驶车辆的轨迹

 private:
  common::Status InitFrame(const uint32_t sequence_num,             // 初始化一帧数据的函数, 序列号
                           const common::TrajectoryPoint& planning_start_point, // planning的起点
                           const double start_time,                 // 开始时间
                           const common::VehicleState& vehicle_state); // 车辆的状态

  routing::RoutingResponse last_routing_;                           // 上一次routing请求的结果

  void ExportReferenceLineDebug(planning_internal::Debug* debug);   // 导入参考线的debug的参数

  std::unique_ptr<Frame> frame_;                                    // Frame的独享智能指针

  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;    // 基于地图提供的粗糙参考线，构建平滑的参考线
};

}  // namespace planning  VehicleStateProvider
}  // namespace apollo

#endif /* MODULES_PLANNING_STD_PLANNING_H_ */
