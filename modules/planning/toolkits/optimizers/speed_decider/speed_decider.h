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
 * @file
 **/

#ifndef MODULES_PLANNING_TASKS_SPEED_DECIDER_SPEED_DECIDER_H_
#define MODULES_PLANNING_TASKS_SPEED_DECIDER_SPEED_DECIDER_H_

#include <string>

#include "modules/planning/proto/dp_st_speed_config.pb.h"
#include "modules/planning/proto/st_boundary_config.pb.h"

#include "modules/planning/toolkits/optimizers/task.h"
namespace apollo {
namespace planning {

class SpeedDecider : public Task {                                                    // 速度决策器
 public:
  SpeedDecider();                                                                     // 速度决策器的构造函数
  ~SpeedDecider() = default;                                                          // 速度决策函数的析构函数

  bool Init(const PlanningConfig& config) override;                                   // 初始化函数, 通过planning_config.proto文件进行初始化话

  apollo::common::Status Execute(                                                     // 执行函数, 重写了task里面的Execute函数
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

 private:
  enum StPosition {                                                                   // st坐标下的位置
    ABOVE = 1,                                                                        // 在什么之上
    BELOW = 2,                                                                        // 在什么之下
    CROSS = 3,                                                                        // 交叉路口
  };

  StPosition GetStPosition(const SpeedData& speed_profile,                            // 获取st坐标下的位置
                           const StBoundary& st_boundary) const;                      // 输入为一个速度的曲线和对应的st的boundary(边框)
  /**
   * @brief check if the ADC should follow an obstacle by examing the
   *StBoundary of the obstacle.
   * @param boundary The boundary of the obstacle.
   * @return true if the ADC believe it should follow the obstacle, and
   *         false otherwise.
   **/
  bool CheckIsFollowByT(const StBoundary& boundary) const;                            // 检查自动驾驶车辆是否应该follow(跟随)一个障碍物

  bool CreateStopDecision(const PathObstacle& path_obstacle,                          // 创建一个停止的目标点
                          ObjectDecisionType* const stop_decision,
                          double stop_distance) const;

  /**
   * @brief create follow decision based on the boundary
   **/
  bool CreateFollowDecision(const PathObstacle& path_obstacle,                        // 在boundary的基础上创建一个跟车的决策
                            ObjectDecisionType* const follow_decision) const;

  /**
   * @brief create yield decision based on the boundary
   **/
  bool CreateYieldDecision(const PathObstacle& path_obstacle,                         // 创建一个避让车辆的决策
                           ObjectDecisionType* const yield_decision) const;

  /**
   * @brief create overtake decision based on the boundary
   **/
  bool CreateOvertakeDecision(                                                        // 创建一个超车的决策
      const PathObstacle& path_obstacle,
      ObjectDecisionType* const overtake_decision) const;

  apollo::common::Status MakeObjectDecision(                                          // 设置对应对象的决策
      const SpeedData& speed_profile, PathDecision* const path_decision) const;

  void AppendIgnoreDecision(PathObstacle* path_obstacle) const;                       // 对一个障碍物进行忽略的决策

  /**
   * @brief "too close" is determined by whether ego vehicle will hit the front
   * obstacle if the obstacle drive at current speed and ego vehicle use some
   * reasonable deceleration
   **/
  bool IsFollowTooClose(const PathObstacle& path_obstacle) const;                     // 太近是否进行跟车

 private:
  DpStSpeedConfig dp_st_speed_config_;                                                // 速度动态规划的配置项
  StBoundaryConfig st_boundary_config_;                                               // st边界(boundary)配置参数
  SLBoundary adc_sl_boundary_;                                                        // 自动化驾驶车辆的边框(boundary)
  apollo::common::TrajectoryPoint init_point_;                                        // 轨迹的起点
  const ReferenceLine* reference_line_ = nullptr;                                     // 中心参考线
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_SPEED_DECIDER_SPEED_DECIDER_H_
