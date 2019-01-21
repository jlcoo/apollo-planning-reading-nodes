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
 * @file dp_st_speed_optimizer.cc
 **/

#include "modules/planning/toolkits/optimizers/dp_st_speed/dp_st_speed_optimizer.h"

#include <algorithm>
#include <vector>

#include "modules/planning/proto/planning_internal.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/toolkits/optimizers/dp_st_speed/dp_st_graph.h"
#include "modules/planning/toolkits/optimizers/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;                                                         // 错误代码
using apollo::common::Status;                                                            // 状态
using apollo::common::TrajectoryPoint;                                                   // 轨迹点
using apollo::common::VehicleConfigHelper;                                               // 车辆配置的帮助者
using apollo::common::adapter::AdapterManager;                                           // 适配器
using apollo::localization::LocalizationEstimate;                                        // 定位的估算
using apollo::planning_internal::STGraphDebug;                                           // st图中的debug的信息

DpStSpeedOptimizer::DpStSpeedOptimizer()                                                 // 构造函数
    : SpeedOptimizer("DpStSpeedOptimizer") {}

bool DpStSpeedOptimizer::Init(const PlanningConfig& config) {                            // 初始化dp速度优化器的值
  dp_st_speed_config_ =                                                                  // 通过跟车场景的配置
      config.lane_follow_scenario_config().dp_st_speed_config();
  st_boundary_config_ = dp_st_speed_config_.st_boundary_config();                        // 获取st的boundary
  is_init_ = true;                                                                       // 将初始化参数设置为true
  return true;
}

bool DpStSpeedOptimizer::SearchStGraph(                                                  // 搜索st图
    const StBoundaryMapper& boundary_mapper,                                             // 将障碍物和st坐标系下的boundary映射到一起
    const SpeedLimitDecider& speed_limit_decider, const PathData& path_data,             // 速度限制的决策者, path相关的数据
    SpeedData* speed_data, PathDecision* path_decision,                                  // speed相关的数据, path看到障碍物的决策者
    STGraphDebug* st_graph_debug) const {                                                // st图中debug的信息
  std::vector<const StBoundary*> boundaries;                                             // st的图中障碍物的boundary(边框)
  for (auto* obstacle : path_decision->path_obstacles().Items()) {                       // 迭代所有决策者对应的障碍物
    auto id = obstacle->Id();                                                            // 取出id值
    if (!obstacle->st_boundary().IsEmpty()) {                                            // 障碍物的st边框不为空
      if (obstacle->st_boundary().boundary_type() ==
          StBoundary::BoundaryType::KEEP_CLEAR) {                                        // keep clear的障碍物不会设置为堵塞的障碍物
        path_decision->Find(id)->SetBlockingObstacle(false);
      } else {
        path_decision->Find(id)->SetBlockingObstacle(true);                              // 其他障碍物都会设置为堵塞的
      }
      boundaries.push_back(&obstacle->st_boundary());                                    // 并且添加st的边框
    } else if (FLAGS_enable_side_vehicle_st_boundary &&
               (adc_sl_boundary_.start_l() > 2.0 ||
                adc_sl_boundary_.end_l() < -2.0)) {                                      // 车停得太偏了
      if (path_decision->Find(id)->reference_line_st_boundary().IsEmpty()) {
        continue;
      }
      ADEBUG << "obstacle " << id << " is NOT blocking.";                                // 不是堵塞的障碍物
      auto st_boundary_copy =
          path_decision->Find(id)->reference_line_st_boundary();                         // 获取path障碍物身上的st boundary
      auto st_boundary = st_boundary_copy.CutOffByT(3.5);                                // 3.5秒处的boundary
      if (!st_boundary.IsEmpty()) {                                                      // st的boundary不是
        auto decision = obstacle->LongitudinalDecision();                                // 获取纵向的决策
        if (decision.has_yield()) {                                                      // 避让
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::YIELD);
        } else if (decision.has_overtake()) {                                            // 超车
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::OVERTAKE);
        } else if (decision.has_follow()) {                                              // 跟车
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::FOLLOW);
        } else if (decision.has_stop()) {                                                // 停车
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::STOP);
        }
        st_boundary.SetId(st_boundary_copy.id());                                        // 设置障碍物的id值
        st_boundary.SetCharacteristicLength(
            st_boundary_copy.characteristic_length());                                   // 特征值长度有什么用呢??

        path_decision->SetStBoundary(id, st_boundary);
        boundaries.push_back(&obstacle->st_boundary());                                  // 将st的boundary放到数组中
      }
    }
  }

  // step 2 perform graph search
  SpeedLimit speed_limit;                                                                // 在st图中进行搜索
  if (!speed_limit_decider
           .GetSpeedLimits(path_decision->path_obstacles(), &speed_limit)
           .ok()) {
    AERROR << "Getting speed limits for dp st speed optimizer failed!";
    return false;
  }

  const float path_length = path_data.discretized_path().Length();                       // 获得离散path的长度
  StGraphData st_graph_data(boundaries, init_point_, speed_limit, path_length);          // 构造一个st图

  DpStGraph st_graph(
      st_graph_data, dp_st_speed_config_,
      reference_line_info_->path_decision()->path_obstacles().Items(),
      init_point_, adc_sl_boundary_);                                                    // 构造一个st的图

  if (!st_graph.Search(speed_data).ok()) {                                               // 搜索图中的点
    AERROR << "failed to search graph with dynamic programming.";
    RecordSTGraphDebug(st_graph_data, st_graph_debug);                                   // 在动态规划的图中没有对应的数据就debug对应的值
    return false;
  }
  RecordSTGraphDebug(st_graph_data, st_graph_debug);                                     // 成功了还是需要debug对应的值
  return true;
}

Status DpStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,                    // 自动驾驶车辆的sl boundary
                                   const PathData& path_data,                            // path的数据
                                   const TrajectoryPoint& init_point,
                                   const ReferenceLine& reference_line,
                                   const SpeedData& reference_speed_data,
                                   PathDecision* const path_decision,
                                   SpeedData* const speed_data) {
  if (!is_init_) {                                                                       // 没有进行初始化不能进行动态规划的计算
    AERROR << "Please call Init() before process DpStSpeedOptimizer.";
    return Status(ErrorCode::PLANNING_ERROR, "Not inited.");
  }
  init_point_ = init_point;                                                              // 轨迹的起点
  adc_sl_boundary_ = adc_sl_boundary;                                                    // 自动驾驶车辆的boundary(边框)
  reference_line_ = &reference_line;                                                     // 中心参考线

  if (path_data.discretized_path().NumOfPoints() == 0) {                                 // path上离散的点个数为0, 就是path data里面的数据是空的
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);                                       // 直接报错
  }

  StBoundaryMapper boundary_mapper(                                                      // 将boundary，decision和障碍物绑定在一起
      adc_sl_boundary, st_boundary_config_, *reference_line_, path_data,
      dp_st_speed_config_.total_path_length(), dp_st_speed_config_.total_time(),
      reference_line_info_->IsChangeLanePath());

  auto* debug = reference_line_info_->mutable_debug();
  STGraphDebug* st_graph_debug = debug->mutable_planning_data()->add_st_graph();         // debug st图中的信息

  path_decision->EraseStBoundaries();                                                    // 决策的boundary全部清空
  if (boundary_mapper.CreateStBoundary(path_decision).code() ==
      ErrorCode::PLANNING_ERROR) {                                                       // 将决策和boundary映射在一起(绑定障碍物对应的决策)
    const std::string msg =
        "Mapping obstacle for dp st speed optimizer failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, st_boundary_config_,           // 限速的决策者
                                        *reference_line_, path_data);

  if (!SearchStGraph(boundary_mapper, speed_limit_decider, path_data,
                     speed_data, path_decision, st_graph_debug)) {                      // 搜索对应的点
    const std::string msg(Name() +
                          ":Failed to search graph with dynamic programming.");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
