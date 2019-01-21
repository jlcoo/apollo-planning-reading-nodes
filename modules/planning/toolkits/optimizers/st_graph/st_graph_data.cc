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
 * @file: st_graph_data.cc
 **/

#include "modules/planning/toolkits/optimizers/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;                                                 // 利用轨迹点

StGraphData::StGraphData(const std::vector<const StBoundary*>& st_boundaries,          // st的边框
                         const TrajectoryPoint& init_point,                            // 起点
                         const SpeedLimit& speed_limit,                                // 速度的限速
                         const double path_data_length)                                // path data的长度
    : st_boundaries_(st_boundaries),
      init_point_(init_point),
      speed_limit_(speed_limit),
      path_data_length_(path_data_length) {}                                           // 初始化列表进行构造

const std::vector<const StBoundary*>& StGraphData::st_boundaries() const {             // 返回st坐标系中的边框
  return st_boundaries_;
}

const TrajectoryPoint& StGraphData::init_point() const { return init_point_; }         // 返回起点

const SpeedLimit& StGraphData::speed_limit() const { return speed_limit_; }            // 速度限制

double StGraphData::path_data_length() const { return path_data_length_; }             // 返回st图中的数据长度

}  // namespace planning
}  // namespace apollo
