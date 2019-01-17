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
 * @file speed_profile_generator.cc
 **/

#include "modules/planning/common/speed_profile_generator.h"

#include <algorithm>

#include "modules/common/log.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using common::SpeedPoint;                                                                 // 速度的点
using common::SLPoint;                                                                    // sl坐标系中的一个点
using common::TrajectoryPoint;                                                            // 轨迹点, 里面有(速度，加速度，相对时间，path point(x,y,z三个轴的坐标))
using common::math::Vec2d;                                                                // 二维的向量

std::vector<SpeedPoint> SpeedProfileGenerator::GenerateInitSpeedProfile(                  // 通过轨迹的初始化点和中心参考线的信息生成一些速度点
    const TrajectoryPoint& planning_init_point,
    const ReferenceLineInfo* reference_line_info) const {
  std::vector<SpeedPoint> speed_profile;                                                  // 速度变量组成的一个数组
  const auto* last_frame = FrameHistory::instance()->Latest();                            // 获得最新的历史数据
  if (!last_frame) {                                                                      // 进行容错处理, 返回的数据为空的话， 啥事都不做呀
    AWARN << "last frame is empty";
    return speed_profile;
  }
  const ReferenceLineInfo* last_reference_line_info =                                     // 找出最新可以行驶的中心参考线
      last_frame->DriveReferenceLineInfo();
  if (!last_reference_line_info) {                                                        // 返回的最新的参考线不能进行正常行驶的话就直接返回一个空数组
    ADEBUG << "last reference line info is empty";
    return speed_profile;
  }
  if (!reference_line_info->IsStartFrom(*last_reference_line_info)) {                     // 判断中心参考线是否是从之前可以行驶的车道线上开始的车道线
    ADEBUG << "Current reference line is not started previous drived line";
    return speed_profile;                                                                 // 不是的话就直接返回空数组
  }
  const auto& last_speed_vector =                                                         // 从参考线信息中获取速度的数组
      last_reference_line_info->speed_data().speed_vector();

  if (!last_speed_vector.empty()) {                                                       // 速度向量不为空
    const auto& last_init_point = last_frame->PlanningStartPoint().path_point();          // planning的起点
    Vec2d last_xy_point(last_init_point.x(), last_init_point.y());                        // 利用x,y坐标转换为二维的向量
    SLPoint last_sl_point;                                                                // 将xy坐标转换为sl坐标
    if (!last_reference_line_info->reference_line().XYToSL(last_xy_point,                 // 利用参考线信息的参考线然后转换为sl坐标
                                                           &last_sl_point)) {
      AERROR << "Fail to transfer xy to sl when init speed profile";
    }

    Vec2d xy_point(planning_init_point.path_point().x(),                                  // planning初始点的路径点
                   planning_init_point.path_point().y());
    SLPoint sl_point;                                                                     // sl坐标系中的一点
    if (!last_reference_line_info->reference_line().XYToSL(xy_point,                      // 转换为sl坐标
                                                           &sl_point)) {
      AERROR << "Fail to transfer xy to sl when init speed profile";
    }

    double s_diff = sl_point.s() - last_sl_point.s();                                     // 两个点的差距
    double start_time = 0.0;
    double start_s = 0.0;
    bool is_updated_start = false;                                                        // 迭代所有的速度点        
    for (const auto& speed_point : last_speed_vector) {
      if (speed_point.s() < s_diff) {                                                     // 如果速度点比这个差值小的话就会跳过这个点
        continue;
      }
      if (!is_updated_start) {                                                            // 还没有开始更新, 意思是第一个点
        start_time = speed_point.t();                                                     // 开始的时间
        start_s = speed_point.s();                                                        // 开始的路程s
        is_updated_start = true;                                                          // 表示设置过了第一个点
      }
      SpeedPoint refined_speed_point;                                                     // 精致的速度点
      refined_speed_point.set_s(speed_point.s() - start_s);                               // 设置路程(s), 时间(t), 速度(v), 加速度(a), 加速度的微分(da)
      refined_speed_point.set_t(speed_point.t() - start_time);                            
      refined_speed_point.set_v(speed_point.v());
      refined_speed_point.set_a(speed_point.a());
      refined_speed_point.set_da(speed_point.da());
      speed_profile.push_back(std::move(refined_speed_point));                            // 将速度的profile(轮廓)构造出来
    }
  }
  return speed_profile;
}

// a dummy simple hot start
// TODO(All): refine the hotstart speed profile                                           // 一个空的热启动的点
std::vector<SpeedPoint> SpeedProfileGenerator::GenerateSpeedHotStart(                     // 输入时轨迹初始化的起点， 输出是速度点的数组
    const TrajectoryPoint& planning_init_point) const {
  std::vector<SpeedPoint> hot_start_speed_profile;                                        // 临时变量，速度点的数组
  double s = 0.0;                                                                         // 速度
  double t = 0.0;                                                                         // 时间
  double v = common::math::Clamp(planning_init_point.v(), 5.0,                            // 初始化的速度是否在(5, 31.3)这个范围内
                                 FLAGS_planning_upper_speed_limit);                       // FLAGS_planning_upper_speed_limit设置为31.3
  while (t < FLAGS_trajectory_time_length) {                                              // 一条轨迹的时间长度最多为8秒， FLAGS_trajectory_time_length设置为8
    SpeedPoint speed_point;                                                               // 设置速度点的信息
    speed_point.set_s(s);
    speed_point.set_t(t);
    speed_point.set_v(v);
 
    hot_start_speed_profile.push_back(std::move(speed_point));                            // 放到热启动的速度文件中

    t += FLAGS_trajectory_time_min_interval;                                              // 轨迹点的时间间隔最小值为0.02秒(即20ms)
    s += v * FLAGS_trajectory_time_min_interval;                                          // s是的累加距离
  }
  return hot_start_speed_profile;                                                         // 返回热启动的速度
}

SpeedData SpeedProfileGenerator::GenerateFallbackSpeedProfile() {                         // 获取反馈的速度
  const double init_v = EgoInfo::instance()->start_point().v();                           // 初始点的速度
  const double init_a = EgoInfo::instance()->start_point().a();                           // 初始点的加速度
  if (init_v > FLAGS_polynomial_speed_fallback_velocity) {                                // 使用多项式计算速度反馈的速度
    return GenerateStopProfileFromPolynomial(init_v, init_a);                             // 构造一系列的点
  } else {
    auto speed_data = GenerateStopProfileFromPolynomial(init_v, init_a);
    if (!speed_data.Empty()) {                                                            // 和上面的逻辑有什么区别呢? 感觉是一样的呀
      return speed_data;
    }
  }                                                                                       // 不会执行到这里的逻辑吧?
  return GenerateStopProfile(init_v, init_a);
}

SpeedData SpeedProfileGenerator::GenerateStopProfile(                                     // 硬生的生成速度反馈的文件
    const double init_speed, const double init_acc) const {
  AERROR << "Using fallback stopping profile: Slowing down the car!";                     // 这里会报错
  SpeedData speed_data;

  const double max_t = FLAGS_fallback_total_time;                                         // 反馈轨迹的总时间设置为3.0秒
  const double unit_t = FLAGS_fallback_time_unit;                                         // 反馈轨迹的时间间隔为0.02秒

  double pre_s = 0.0;                                                                     // 前一个点的路程s
  double pre_v = init_speed;                                                              // 前一个点的速度v
  double acc = FLAGS_slowdown_profile_deceleration;                                       // 减速曲线的减速度为-1.0m/s^2

  for (double t = 0.0; t < max_t; t += unit_t) {                                          // 迭代总时间
    double s = 0.0;
    double v = 0.0;
    s = std::fmax(pre_s,
                  pre_s + 0.5 * (pre_v + (pre_v + unit_t * acc)) * unit_t);               // 物理公式计算
    v = std::fmax(0.0, pre_v + unit_t * acc);
    speed_data.AppendSpeedPoint(s, t, v, acc, 0.0);                                       // 追加到速度中
    pre_s = s;
    pre_v = v;
  }
  return speed_data;                                                                      // 直接返回
}

SpeedData SpeedProfileGenerator::GenerateStopProfileFromPolynomial(                              // 从多项式中返回一个停止的速度
    const double init_speed, const double init_acc) const {                                      // 输入是初始化的速度和初始化的加速度
  AERROR << "Slowing down the car with polynomial.";                                             // 用多项式给汽车减少
  constexpr double kMaxT = 4.0;                                                                  // 最大的时间
  for (double t = 2.0; t <= kMaxT; t += 0.5) {                                                   // 起点是2秒, 时间间隔是0.5秒, 最大是4
    for (double s = 0.0;
         s < std::min(50.0, EgoInfo::instance()->front_clear_distance() - 0.3);                  // 迭代前方障碍物，1米一米的迭代
         s += 1.0) {                                                                             // 一维的5次多项式曲线
      // 前三个是针对路程的计算约束项, 后三个需要计算的参数, 变量是t(f(t)的5次多项式)
      QuinticPolynomialCurve1d curve(0.0, init_speed, init_acc, s, 0.0, 0.0, t);                 // 通过一个初始化的速度， 初始化的加速度， 时间点构造一个多项式曲线
      if (!IsValidProfile(curve)) {                                                              // 计算出来的曲线不合法，就继续计算下一条曲线
        continue;
      }
      constexpr double kUnitT = 0.02;                                                            // 时间戳的单位为0.02秒(20毫秒)
      SpeedData speed_data;                                                                      // 速度点
      for (double curve_t = 0.0; curve_t <= t; curve_t += kUnitT) {
        const double curve_s = curve.Evaluate(0, curve_t);                                       // 0次是路程， 1次是速度， 2次是加速度， 3次是加速度的微分
        const double curve_v = curve.Evaluate(1, curve_t);                                       // 算出一系列的点
        const double curve_a = curve.Evaluate(2, curve_t);
        const double curve_da = curve.Evaluate(3, curve_t);
        speed_data.AppendSpeedPoint(curve_s, curve_t, curve_v, curve_a,
                                    curve_da);                                                   // 5次多项式平滑做出来的点再追加到SpeedData, 最后返回
      }
      return speed_data;
    }
  }
  return SpeedData();
}

bool SpeedProfileGenerator::IsValidProfile(                                             // 判断一个速度曲线是否合法(profile就可以理解为曲线的意思)
    const QuinticPolynomialCurve1d& curve) const {
  for (double evaluate_t = 0.1; evaluate_t <= curve.ParamLength();                      // 迭代每个参数(t), 然后让t从0.1开始, 每隔0.2秒计算一个值
       evaluate_t += 0.2) {
    const double v = curve.Evaluate(1, evaluate_t);                                     // 计算速度和加速度
    const double a = curve.Evaluate(2, evaluate_t);
    constexpr double kEpsilon = 1e-3;
    if (v < -kEpsilon || a < -5.0) {                                                    // 减速度不能太大， 速度不能为小于0
      return false;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
