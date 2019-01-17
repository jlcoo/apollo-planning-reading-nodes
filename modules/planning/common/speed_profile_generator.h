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
 * @file speed_profile_generator.h
 **/

#ifndef MODULES_PLANNING_COMMON_SPEED_PROFILE_GENERATOR_H_
#define MODULES_PLANNING_COMMON_SPEED_PROFILE_GENERATOR_H_

#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {
// 速度曲线的产生器
class SpeedProfileGenerator {                                                    // 速度反馈文件
 public:
  SpeedProfileGenerator() = default;                                             // 默认的构造和析构函数
  ~SpeedProfileGenerator() = default;
  // 生成初始速度配置文件
  std::vector<common::SpeedPoint> GenerateInitSpeedProfile(                      // 产生一些初始点
      const common::TrajectoryPoint& planning_init_point,                        // 输入为轨迹初始化的点, 参考线的信心
      const ReferenceLineInfo* reference_line_info) const;                       // 输出为速度点的数组

  std::vector<common::SpeedPoint> GenerateSpeedHotStart(                         // 产生速度的热启动
      const common::TrajectoryPoint& planning_init_point) const;                 // planning的一些初始化的点

  SpeedData GenerateFallbackSpeedProfile();                                      // 产生一些速度反馈的文件

 private:
  SpeedData GenerateStopProfile(const double init_speed,                         // 根据初始化的速度和初始化的加速度
                                const double init_acc) const;                    // 产生一个速度的信息

  SpeedData GenerateStopProfileFromPolynomial(const double init_speed,           // 从多项式中产生一个停止的文件(与速度相关的)
                                              const double init_acc) const;

  bool IsValidProfile(const QuinticPolynomialCurve1d& curve) const;              // 判断是否合法， 输入是一个五次多项式的曲线
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_SPEED_PROFILE_GENERATOR_H_
