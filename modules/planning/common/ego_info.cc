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

/**
 * @file speed_limit.cc
 **/

#include "modules/planning/common/ego_info.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"

namespace apollo {
namespace planning {

using common::math::Vec2d;                                                                 // 二维的向量
using common::math::Box2d;                                                                 // 二维的box(边框)

EgoInfo::EgoInfo() {                                                                       // 构造函数, 从车辆的配置项帮助者中获取配置
  common::VehicleConfig ego_vehicle_config_ =
      common::VehicleConfigHelper::GetConfig();
}

bool EgoInfo::Update(const common::TrajectoryPoint& start_point,                           // 更新状态
                     const common::VehicleState& vehicle_state,
                     const std::vector<const Obstacle*>& obstacles) {
  set_start_point(start_point);                                                            // 设置起点
  set_vehicle_state(vehicle_state);                                                        // 设置状态
  CalculateFrontObstacleClearDistance(obstacles);                                          // 计算前向障碍物的距离

  return true;
}

void EgoInfo::Clear() {                                                                    // 清空内部的数据
  start_point_.Clear();                                                                    // 内部的起点清空
  vehicle_state_.Clear();                                                                  // 内部的状态清空
  front_clear_distance_ = std::numeric_limits<double>::max();                              // 设置前向距离无限大
}
// 计算前障碍物的清除距离  ego是自我评价的意思
void EgoInfo::CalculateFrontObstacleClearDistance(
    const std::vector<const Obstacle*>& obstacles) {                                       // 给定一个障碍物的数组
  Vec2d position(vehicle_state_.x(), vehicle_state_.y());                                  // 车的中心位置

  const auto& param = ego_vehicle_config_.vehicle_param();                                 // 车自身的参数
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,                  //中心长多少
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);

  Vec2d center(position + vec_to_center.rotate(vehicle_state_.heading()));                 // 车旋转航向角的角度

  const double buffer = 0.1;  // in meters, 车身加0.1米的边框
  Box2d ego_box(center, vehicle_state_.heading(), param.length() + buffer,                 // ego的信息
                param.width() + buffer);
  const double adc_half_diagnal = ego_box.diagonal() / 2.0;                                // 对角线大小 

  Vec2d unit_vec_heading = Vec2d::CreateUnitVec2d(vehicle_state_.heading());               //根据航向角创建一个单位向量

  // Due to the error of ego heading, only short range distance is meaningful
  const double kDistanceThreshold = 50.0;                                                  // 航向角有误差, 只能在短距离中才会有用
  const double impact_region_length =
      param.length() + buffer + kDistanceThreshold;    // 50米范围才会有用
  Box2d ego_front_region(center + unit_vec_heading * kDistanceThreshold / 2.0,
                         vehicle_state_.heading(), impact_region_length,
                         param.width() + buffer);

  for (const auto& obstacle : obstacles) {                                                // 迭代每个障碍物
    if (obstacle->IsVirtual() ||
        !ego_front_region.HasOverlap(obstacle->PerceptionBoundingBox())) {
      continue;   // 障碍物已经被可视化了, 并且boundingBox没有重叠
    }
    // 
    double dist = ego_box.center().DistanceTo(
                      obstacle->PerceptionBoundingBox().center()) -
                  adc_half_diagnal;

    if (front_clear_distance_ < 0.0 || dist < front_clear_distance_) {                    // 距离必须是正数
      front_clear_distance_ = dist;                                                       // 更新前方最小距离
    }
  }
}

}  // namespace planning
}  // namespace apollo
