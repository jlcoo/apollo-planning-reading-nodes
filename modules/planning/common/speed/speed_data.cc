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
 * @file speed_data.cc
 **/

#include "modules/planning/common/speed/speed_data.h"

#include <algorithm>
#include <utility>

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

SpeedData::SpeedData(std::vector<common::SpeedPoint> speed_points)
    : speed_vector_(std::move(speed_points)) {}                                     // 利用带move的初始化列表进行构造对象才是一种正确方法

void SpeedData::AppendSpeedPoint(const double s, const double time,
                                 const double v, const double a,
                                 const double da) {
  if (!speed_vector_.empty()) {
    CHECK(speed_vector_.back().t() < time);                                         // 容错处理
  }
  speed_vector_.push_back(common::util::MakeSpeedPoint(s, time, v, a, da));
}

const std::vector<common::SpeedPoint>& SpeedData::speed_vector() const {            // 返回速度点
  return speed_vector_;
}

void SpeedData::set_speed_vector(std::vector<common::SpeedPoint> speed_points) {
  speed_vector_ = std::move(speed_points);                                          // 这里使用std::move不会构造新的对象了么?
}

bool SpeedData::EvaluateByTime(const double t,                                      // 通过t进行插值
                               common::SpeedPoint* const speed_point) const {
  if (speed_vector_.size() < 2) {
    return false;
  }
  if (!(speed_vector_.front().t() < t + 1.0e-6 &&                                   // 容错处理, 1.0e-6这个一个极小值
        t - 1.0e-6 < speed_vector_.back().t())) {
    return false;
  }

  auto comp = [](const common::SpeedPoint& sp, const double t) {                    // 函数对象
    return sp.t() < t;
  };

  auto it_lower =
      std::lower_bound(speed_vector_.begin(), speed_vector_.end(), t, comp);        // 找到下界
  if (it_lower == speed_vector_.end()) {                                            // 如果是两个顶点就直接返回
    *speed_point = speed_vector_.back();
  } else if (it_lower == speed_vector_.begin()) {
    *speed_point = speed_vector_.front();
  } else {                                                                          // 否则在p0和p1中间进行线性插值
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double t0 = p0.t();
    double t1 = p1.t();

    double s = common::math::lerp(p0.s(), t0, p1.s(), t1, t);                       // 线性插值
    double v = common::math::lerp(p0.v(), t0, p1.v(), t1, t);
    double a = common::math::lerp(p0.a(), t0, p1.a(), t1, t);
    double j = common::math::lerp(p0.da(), t0, p1.da(), t1, t);

    *speed_point = common::util::MakeSpeedPoint(s, t, v, a, j);                     // 通过s,t,v,a,jerk构造一个speed的点
  }
  return true;
}

double SpeedData::TotalTime() const {                                               // 返回速度向量里面的总时间
  if (speed_vector_.empty()) {
    return 0.0;
  }
  return speed_vector_.back().t() - speed_vector_.front().t();                      // 速度向量的back点和front点的时间作差
}

void SpeedData::Clear() { speed_vector_.clear(); }                                  // 清空速度向量

std::string SpeedData::DebugString() const {                                        // debug的字符串
  const auto limit =
      std::min(speed_vector_.size(),
               static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
  return apollo::common::util::StrCat(
      "[\n", apollo::common::util::PrintDebugStringIter(
                 speed_vector_.begin(), speed_vector_.begin() + limit, ",\n"),
      "]\n");
}

}  // namespace planning
}  // namespace apollo
