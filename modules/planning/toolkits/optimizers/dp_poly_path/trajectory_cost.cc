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

#include "modules/planning/toolkits/optimizers/dp_poly_path/trajectory_cost.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;                                                             // 轨迹点
using apollo::common::math::Box2d;                                                                 // 二维的box
using apollo::common::math::Sigmoid;                                                               // s函数
using apollo::common::math::Vec2d;                                                                 // 二维的向量

TrajectoryCost::TrajectoryCost(                                                                    // 轨迹的代价函数
    const DpPolyPathConfig &config, const ReferenceLine &reference_line,                           // dp多项式动态规划path的配置
    const bool is_change_lane_path,                                                                // path被映射到了sl坐标系中
    const std::vector<const PathObstacle *> &obstacles,                                            // path上的障碍物
    const common::VehicleParam &vehicle_param,                                                     // 车辆的状态
    const SpeedData &heuristic_speed_data, const common::SLPoint &init_sl_point)                   // 启发式的速度， SpeedData是一个类
    : config_(config),
      reference_line_(&reference_line),
      is_change_lane_path_(is_change_lane_path),
      vehicle_param_(vehicle_param),
      heuristic_speed_data_(heuristic_speed_data),
      init_sl_point_(init_sl_point) {                                                              // 初始化列表
  const float total_time =
      std::min(heuristic_speed_data_.TotalTime(), FLAGS_prediction_total_time);                    // prediction的预测的总时间是5.0

  num_of_time_stamps_ = static_cast<uint32_t>(
      std::floor(total_time / config.eval_time_interval()));                                       // std::floor是向下取整, std::ceil是向上取整

  for (const auto *ptr_path_obstacle : obstacles) {                                                // 迭代所有障碍物
    if (ptr_path_obstacle->IsIgnore()) {                                                           // 如果该障碍物被忽略了就继续迭代下一个障碍物
      continue;
    } else if (ptr_path_obstacle->LongitudinalDecision().has_stop()) {                             // 如果有纵向的策略为停止，继续迭代下一个障碍物
      continue;
    }
    const auto &sl_boundary = ptr_path_obstacle->PerceptionSLBoundary();                           // 获取障碍物的sl坐标系

    const float adc_left_l =
        init_sl_point_.l() + vehicle_param_.left_edge_to_center();                                 // 在参考线的左正右负
    const float adc_right_l =
        init_sl_point_.l() - vehicle_param_.right_edge_to_center();

    if (adc_left_l + FLAGS_lateral_ignore_buffer < sl_boundary.start_l() ||                        // 横向只考虑3米内的障碍物
        adc_right_l - FLAGS_lateral_ignore_buffer > sl_boundary.end_l()) {
      continue;
    }

    const auto *ptr_obstacle = ptr_path_obstacle->obstacle();                                      // 取出只读的障碍物
    bool is_bycycle_or_pedestrian =
        (ptr_obstacle->Perception().type() ==
             perception::PerceptionObstacle::BICYCLE ||
         ptr_obstacle->Perception().type() ==
             perception::PerceptionObstacle::PEDESTRIAN);                                          // 判断是否为自行车或者行人

    if (Obstacle::IsVirtualObstacle(ptr_obstacle->Perception())) {                                 // 如果是虚拟障碍物就直接跳过
      // Virtual obstacle
      continue;
    } else if (Obstacle::IsStaticObstacle(ptr_obstacle->Perception()) ||                           // 将自行车或者行人都当做静态障碍物，并且添加边框(sl_boundary)
               is_bycycle_or_pedestrian) {
      static_obstacle_sl_boundaries_.push_back(std::move(sl_boundary));
    } else {                                                                                       // 其他的东西都看成动态障碍物
      std::vector<Box2d> box_by_time;                                                              // 通过时间获取障碍物的box
      for (uint32_t t = 0; t <= num_of_time_stamps_; ++t) {                                        // 一个时间点，一个时间点地迭代
        TrajectoryPoint trajectory_point =
            ptr_obstacle->GetPointAtTime(t * config.eval_time_interval());                         // 通过时间点获得轨迹的一点

        Box2d obstacle_box = ptr_obstacle->GetBoundingBox(trajectory_point);                       // 通过轨迹上的一点获取一个bounding box(边框的box), 返回的就是一个box2d
        constexpr float kBuff = 0.5;                                                               // 给每个box都添加一个0.5的外围边框(buff)
        Box2d expanded_obstacle_box =
            Box2d(obstacle_box.center(), obstacle_box.heading(),
                  obstacle_box.length() + kBuff, obstacle_box.width() + kBuff);                    // 在障碍物周围再加上一层保护环
        box_by_time.push_back(expanded_obstacle_box);                                              // 添加到时间点找到的障碍物中(意思是动态障碍物随时间得到的轨迹)
      }
      dynamic_obstacle_boxes_.push_back(std::move(box_by_time));                                   // 再放到所有的动态障碍物中
    }
  }
}

ComparableCost TrajectoryCost::CalculatePathCost(                                                  // 计算path的代价
    const QuinticPolynomialCurve1d &curve, const float start_s,
    const float end_s, const uint32_t curr_level, const uint32_t total_level) {
  ComparableCost cost;                                                                             // 作为比较代价的输出
  float path_cost = 0.0;                                                                           // path上的代价
  std::function<float(const float)> quasi_softmax = [this](const float x) {                        // 这个计算代价函数的值不太懂
    const float l0 = this->config_.path_l_cost_param_l0();
    const float b = this->config_.path_l_cost_param_b();
    const float k = this->config_.path_l_cost_param_k();
    return (b + std::exp(-k * (x - l0))) / (1.0 + std::exp(-k * (x - l0)));
  };

  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();                                        // 获取车辆的状态
  const float width = vehicle_config.vehicle_param().width();

  for (float curve_s = 0.0; curve_s < (end_s - start_s);
       curve_s += config_.path_resolution()) {                                                     // 迭代曲线的s
    const float l = curve.Evaluate(0, curve_s);                                                    // 根据case 0进行计算(用到了6个多项式信息)， 通过s， 算出l

    path_cost += l * l * config_.path_l_cost() * quasi_softmax(std::fabs(l));                      // 通过l计算出代价函数的值(累加的值)

    double left_width = 0.0;
    double right_width = 0.0;
    reference_line_->GetLaneWidth(curve_s + start_s, &left_width, &right_width);                   // 通过s点的坐标，获取车道左右两边的距离

    constexpr float kBuff = 0.2;                                                                   // 这个buff为什么又只加0.2呢
    if (!is_change_lane_path_ && (l + width / 2.0 + kBuff > left_width ||
                                  l - width / 2.0 - kBuff < -right_width)) {                       // 在车道的20cm以内是可以接收的???
      cost.cost_items[ComparableCost::OUT_OF_BOUNDARY] = true;                                     // 超出了车道
    }

    const float dl = std::fabs(curve.Evaluate(1, curve_s));                                        // curve的一次求导就是dl
    path_cost += dl * dl * config_.path_dl_cost();                                                 // 叠加dl的代价

    const float ddl = std::fabs(curve.Evaluate(2, curve_s));                                       // 叠加ddl的代价
    path_cost += ddl * ddl * config_.path_ddl_cost();
  }
  path_cost *= config_.path_resolution();                                                          // 为什么还要乘以path上的分辨率???

  if (curr_level == total_level) {                                                                 // 这里的level应该就是平滑的程度
    const float end_l = curve.Evaluate(0, end_s - start_s);                                        // 获取终点的l
    path_cost +=
        std::sqrt(end_l - init_sl_point_.l() / 2.0) * config_.path_end_l_cost();                   // 再加上这个代价
  }
  cost.smoothness_cost = path_cost;                                                                // 平滑的代价就是所有叠加的path
  return cost;
}

ComparableCost TrajectoryCost::CalculateStaticObstacleCost(                                        // 计算静态障碍物的代价
    const QuinticPolynomialCurve1d &curve, const float start_s,
    const float end_s) {
  ComparableCost obstacle_cost;
  for (float curr_s = start_s; curr_s <= end_s;
       curr_s += config_.path_resolution()) {                                                      // 直接迭代path中的分辨率(一个一个的来)
    const float curr_l = curve.Evaluate(0, curr_s - start_s);                                      // 通过5次多项式计算l??
    for (const auto &obs_sl_boundary : static_obstacle_sl_boundaries_) {                           // 迭代静态障碍物中的所有sl的边框
      obstacle_cost += GetCostFromObsSL(curr_s, curr_l, obs_sl_boundary);                          // 从边框中获取代价值
    }
  }
  obstacle_cost.safety_cost *= config_.path_resolution();                                          // 静态障碍的代价就是安全的代价
  return obstacle_cost;
}

ComparableCost TrajectoryCost::CalculateDynamicObstacleCost(                                       // 计算动态障碍物的代价值
    const QuinticPolynomialCurve1d &curve, const float start_s,
    const float end_s) const {
  ComparableCost obstacle_cost;                                                                    // 动态障碍物的代价函数值
  float time_stamp = 0.0;
  for (size_t index = 0; index < num_of_time_stamps_;
       ++index, time_stamp += config_.eval_time_interval()) {                                      // 动态障碍物要考虑时间信息
    common::SpeedPoint speed_point;
    heuristic_speed_data_.EvaluateByTime(time_stamp, &speed_point);                                // 通过时间点获取speed的信息
    float ref_s = speed_point.s() + init_sl_point_.s();                                            // 中心参考线的s, 就是起点加上当前点的距离s
    if (ref_s < start_s) {                                                                         // 当前的速度点过时就继续迭代
      continue;
    }
    if (ref_s > end_s) {                                                                           // 如果当前点的距离超过了需要求的最大距离
      break;                                                                                       // 就马上退出
    }

    const float s = ref_s - start_s;  // s on spline curve                                         // s是在spline曲线上的距离
    const float l = curve.Evaluate(0, s);                                                          // 通过五次多项式曲线获得l(横向的距离)
    const float dl = curve.Evaluate(1, s);                                                         // l的微分

    const common::SLPoint sl = common::util::MakeSLPoint(ref_s, l);                                // 构造一个sl坐标系中的一点
    const Box2d ego_box = GetBoxFromSLPoint(sl, dl);                                               // 获得一个新的box
    for (const auto &obstacle_trajectory : dynamic_obstacle_boxes_) {                              // 迭代所有的动态障碍物
      obstacle_cost +=
          GetCostBetweenObsBoxes(ego_box, obstacle_trajectory.at(index));
    }
  }
  constexpr float kDynamicObsWeight = 1e-6;
  obstacle_cost.safety_cost *=
      (config_.eval_time_interval() * kDynamicObsWeight);
  return obstacle_cost;
}

ComparableCost TrajectoryCost::GetCostFromObsSL(
    const float adc_s, const float adc_l, const SLBoundary &obs_sl_boundary) {                  // 给定一个自动驾驶车辆在sl坐标系下的位置, 障碍物的sl边框, 计算一个代价值
  const auto &vehicle_param =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();                     // 获取车辆的状态

  ComparableCost obstacle_cost;                                                                 // 声明一个代价值

  const float adc_front_s = adc_s + vehicle_param.front_edge_to_center();                       // 自动驾驶车辆的前向距离
  const float adc_end_s = adc_s - vehicle_param.back_edge_to_center();                          // 后向距离
  const float adc_left_l = adc_l + vehicle_param.left_edge_to_center();                         // 左侧距离
  const float adc_right_l = adc_l - vehicle_param.right_edge_to_center();                       // 右侧距离

  if (adc_left_l + FLAGS_lateral_ignore_buffer < obs_sl_boundary.start_l() ||
      adc_right_l - FLAGS_lateral_ignore_buffer > obs_sl_boundary.end_l()) {                    // 如果这个障碍物可以直接忽略, 那就返回空对象
    return obstacle_cost;
  }

  bool no_overlap = ((adc_front_s < obs_sl_boundary.start_s() ||
                      adc_end_s > obs_sl_boundary.end_s()) ||  // longitudinal
                     (adc_left_l + FLAGS_static_decision_nudge_l_buffer <                       // FLAGS_static_decision_nudge_l_buffer设置为0.5
                          obs_sl_boundary.start_l() ||
                      adc_right_l - FLAGS_static_decision_nudge_l_buffer >
                          obs_sl_boundary.end_l()));  // lateral

  if (!no_overlap) {                                                                            // 车和障碍物已经重叠
    obstacle_cost.cost_items[ComparableCost::HAS_COLLISION] = true;
  }

  // if obstacle is behind ADC, ignore its cost contribution.
  if (adc_front_s > obs_sl_boundary.end_s()) {                                                  // 障碍物已经在车的后面, 也会忽略
    return obstacle_cost;
  }

  const float delta_l = std::fabs(
      adc_l - (obs_sl_boundary.start_l() + obs_sl_boundary.end_l()) / 2.0);                     // 离障碍物中心线的距离

  const double kSafeDistance = 1.0;                                                             // 和障碍物的安全距离为1米
  if (delta_l < kSafeDistance) {                                                                // 横向安全距离小于1
    obstacle_cost.safety_cost +=
        config_.obstacle_collision_cost() *
        Sigmoid(config_.obstacle_collision_distance() - delta_l);                               // 安全的代价函数的计算利用到了s函数
  }

  const float delta_s = std::fabs(
      adc_s - (obs_sl_boundary.start_s() + obs_sl_boundary.end_s()) / 2.0);                     // s方向的距离
  obstacle_cost.safety_cost +=
      config_.obstacle_collision_cost() *
      Sigmoid(config_.obstacle_collision_distance() - delta_s);                                 // 这里也用到了s函数(取值在(-1, 1)之间)
  return obstacle_cost;
}

// Simple version: calculate obstacle cost by distance
ComparableCost TrajectoryCost::GetCostBetweenObsBoxes(                                          // 通过距离计算障碍物的cost(代价)
    const Box2d &ego_box, const Box2d &obstacle_box) const {                                    // 输入是自动驾驶车辆的box和障碍物的box
  ComparableCost obstacle_cost;                                                                 // 新建一个对象

  const float distance = obstacle_box.DistanceTo(ego_box);                                      // 两个box之间的距离
  if (distance > config_.obstacle_ignore_distance()) {                                          // 这两个box的距离是否可以忽略
    return obstacle_cost;                                                                       // 是的话就返回空对象
  }

  obstacle_cost.safety_cost +=                                                                  // 碰撞风险的代价
      config_.obstacle_collision_cost() *
      Sigmoid(config_.obstacle_collision_distance() - distance);
  obstacle_cost.safety_cost +=                                                                  // 碰撞距离是0.5米
      20.0 * Sigmoid(config_.obstacle_risk_distance() - distance);                              // 风险距离的权重更大, 设置为2米
  return obstacle_cost;
}

Box2d TrajectoryCost::GetBoxFromSLPoint(const common::SLPoint &sl,                             // 从一个sl坐标和l的一次微分中获取一个二维的box
                                        const float dl) const {
  Vec2d xy_point;                                                                              // 二维的向量
  reference_line_->SLToXY(sl, &xy_point);                                                      // 将sl的坐标转换为二维的xy坐标

  ReferencePoint reference_point = reference_line_->GetReferencePoint(sl.s());                 // 通过s点构造一个中心参考点

  const float one_minus_kappa_r_d = 1 - reference_point.kappa() * sl.l();                      // 1减去曲率乘以l(l不是旋转半径吧??)
  const float delta_theta = std::atan2(dl, one_minus_kappa_r_d);                               // 获取一个角度的增量(航向角的增量)
  const float theta =
      common::math::NormalizeAngle(delta_theta + reference_point.heading());                   // 更新到新的航向角的值
  return Box2d(xy_point, theta, vehicle_param_.length(),                                       // 构造新的box
               vehicle_param_.width());
}

// TODO(All): optimize obstacle cost calculation time
ComparableCost TrajectoryCost::Calculate(const QuinticPolynomialCurve1d &curve,                // 三种障碍物
                                         const float start_s, const float end_s,
                                         const uint32_t curr_level,
                                         const uint32_t total_level) {
  ComparableCost total_cost;
  // path cost
  total_cost +=
      CalculatePathCost(curve, start_s, end_s, curr_level, total_level);                       // path的代价函数值

  // static obstacle cost
  total_cost += CalculateStaticObstacleCost(curve, start_s, end_s);                            // 静态障碍物的代价函数值

  // dynamic obstacle cost
  total_cost += CalculateDynamicObstacleCost(curve, start_s, end_s);                           // 动态障碍物的代价函数值
  return total_cost;
}

}  // namespace planning
}  // namespace apollo
