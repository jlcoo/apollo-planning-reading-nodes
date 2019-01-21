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
 * @file speed_limit.cc
 **/

#include "modules/planning/common/speed_limit.h"

#include <algorithm>

#include "modules/common/log.h"

namespace apollo {
namespace planning {

void SpeedLimit::AppendSpeedLimit(const double s, const double v) {                 // 在数组的末尾增加一个元素
  if (!speed_limit_points_.empty()) {                                               // 速度限制的点
    DCHECK_GE(s, speed_limit_points_.back().first);                                 // s必须是递增的, 你不可能越走越往后啥
  }
  speed_limit_points_.emplace_back(s, v);                                           // 每个点都有一个距离和速度
}

const std::vector<std::pair<double, double>>& SpeedLimit::speed_limit_points()      // 返回内部的数组的引用
    const {
  return speed_limit_points_;
}

double SpeedLimit::GetSpeedLimitByS(const double s) const {                         // 通过一点s获得该处的最大限速
  DCHECK_GE(speed_limit_points_.size(), 2);                                         // 限速的点对至少要有两个
  DCHECK_GE(s, speed_limit_points_.front().first);                                  // s必须要是递增的
  // 比s小的第一个
  auto compare_s = [](const std::pair<double, double>& point, const double s) {     // 构造一个比较的函数对象
    return point.first < s;                                                         // 找到比s大的第一个点
  };

  auto it_lower = std::lower_bound(speed_limit_points_.begin(),                     // 找到比s大的第一个点
                                   speed_limit_points_.end(), s, compare_s);

  if (it_lower == speed_limit_points_.end()) {                                      // 如果没有找到就返回前一个时间点的限速
    return (it_lower - 1)->second;
  }
  return it_lower->second;                                                          // 否则返回找到速度点中的限速
}

void SpeedLimit::Clear() { speed_limit_points_.clear(); }                           // 清空数组

}  // namespace planning
}  // namespace apollo
