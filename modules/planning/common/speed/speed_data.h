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
 * @file speed_data.h
 **/
#ifndef MODULES_PLANNING_COMMON_SPEED_SPEED_DATA_H_
#define MODULES_PLANNING_COMMON_SPEED_SPEED_DATA_H_

#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning {

class SpeedData {                                                             // 速度相关的数据
 public:
  SpeedData() = default;                                                      // 默认的构造函数
  // SpeedPoint是pnc_point.proto文件中定义的. 包含了s, t, v, a, da等信息
  explicit SpeedData(std::vector<common::SpeedPoint> speed_points);           // speed data是由很多个点构成的

  virtual ~SpeedData() = default;                                             // 默认的析构函数

  const std::vector<common::SpeedPoint>& speed_vector() const;                // 返回速度的向量

  void set_speed_vector(std::vector<common::SpeedPoint> speed_points);        // 设置速度向量

  void AppendSpeedPoint(const double s, const double time, const double v,    // 通过路程s, 时间t, 速度v, 加速度a, 加速度的微分da进行构造一个速度点
                        const double a, const double da);

  bool EvaluateByTime(const double time,                                      // 速度点就是通过一个时间t, 进行估计一个速度点(里面实际上是利用了线性插值)
                      common::SpeedPoint* const speed_point) const;

  double TotalTime() const;                                                   // 返回总时间

  bool Empty() const { return speed_vector_.empty(); }                        // 一个速度向量是否是空的

  void Clear();                                                               // 清空速度的数组

  virtual std::string DebugString() const;                                    // debug需要的字符串信息

 private:
  std::vector<common::SpeedPoint> speed_vector_;                              // 用一个vector放速度点
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_SPEED_SPEED_DATA_H_
