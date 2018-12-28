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
 * @file
 **/

#include "modules/planning/common/ego_info.h"

#include "gtest/gtest.h"

#include "modules/common/util/util.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line_provider.h"

namespace apollo {
namespace planning {

TEST(EgoInfoTest, EgoInfoSimpleTest) {
  const auto p =                 // x    y     z  theta kappa dkappa ddkappa
      common::util::MakePathPoint(1.23, 3.23, 52.18, 0.1, 0.3, 0.32, 0.4);  // 生成一个path的点
  common::TrajectoryPoint tp;  // temp point
  tp.mutable_path_point()->CopyFrom(p);   
  auto* ego_info = EgoInfo::instance();   // 生成ego的单例, instance一般都是static的公共函数, 在这个函数中生成一个类对象, 并返回
  ego_info->set_start_point(tp);    // ego_info就是EgoInfo对象, 开始点为tp
  EXPECT_DOUBLE_EQ(ego_info->start_point().path_point().x(), p.x());
  EXPECT_DOUBLE_EQ(ego_info->start_point().path_point().y(), p.y());
  EXPECT_DOUBLE_EQ(ego_info->start_point().path_point().z(), p.z());    // 没问题的话相等

  uint32_t sequence_num = 0;   
  common::TrajectoryPoint planning_start_point;   // planning的开始点
  const double start_time = 102342.0;             // 开始时间
  common::VehicleState vehicle_state;             // 车的状态
  ReferenceLineProvider reference_line_provider;  // 参考线的提供者
  // 一帧数据
  Frame frame(sequence_num, planning_start_point, start_time, vehicle_state,
              &reference_line_provider);
  ego_info->CalculateFrontObstacleClearDistance(frame.obstacles());

  
}

}  // namespace planning
}  // namespace apollo
