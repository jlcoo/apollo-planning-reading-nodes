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

#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"

#include <fstream>
#include <limits>
#include <utility>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/toolkits/optimizers/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/path_decider/path_decider.h"
#include "modules/planning/toolkits/optimizers/poly_st_speed/poly_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/speed_decider/speed_decider.h"

namespace apollo {
namespace planning {

using common::ErrorCode;                                 // 错误码， 在chassis.proto文件中定义的
using common::SLPoint;                                   // SL坐标系的点, 在pnc_point.proto文件中定义, 里面只有s和l坐标
using common::SpeedPoint;                                // 速度的点，  在pnc_point.proto文件中定义， 里面定义了s(已经走过的路程), 时间(t), 速度(v)， 加速度(a), 加速度的导数(da)
using common::Status;                                    // 状态
using common::TrajectoryPoint;                           // 轨迹的点， 在pnc_point.proto文件中定义, 包含了很多信息, 比如PathPoint, 线性速度，线性加速度和开始执行这个轨迹点的相对时间(relative_time)
using common::adapter::AdapterManager;                   // 适配的管理者， 在AdapterManager类中定义了很多adapter相关的适配器, REGISTER_ADAPTER是一个宏
using common::math::Vec2d;                               // 二维向量, 二维的向量
using common::time::Clock;                               // 时钟

namespace {
constexpr double kPathOptimizationFallbackClost = 2e4;   // 路径优化反馈代价2的四次方
constexpr double kSpeedOptimizationFallbackClost = 2e4;  // 速度优化的反馈代价
constexpr double kStraightForwardLineCost = 10.0;        // 直线的成本代价是10
}  // namespace

void LaneFollowScenario::RegisterTasks() {                                                     // laneFollow场景中， 向任务工厂中注册6个任务
  task_factory_.Register(DP_POLY_PATH_OPTIMIZER,
                         []() -> Task* { return new DpPolyPathOptimizer(); });                 // 动态规划的多项式的优化器
  task_factory_.Register(PATH_DECIDER,
                         []() -> Task* { return new PathDecider(); });                         // 路径决策器
  task_factory_.Register(DP_ST_SPEED_OPTIMIZER,
                         []() -> Task* { return new DpStSpeedOptimizer(); });                  // 动态规划的st的速度优化器
  task_factory_.Register(SPEED_DECIDER,
                         []() -> Task* { return new SpeedDecider(); });                        // 速度决策器
  task_factory_.Register(QP_SPLINE_ST_SPEED_OPTIMIZER, []() -> Task* {
    return new QpSplineStSpeedOptimizer();                                                     // 基于Qp的spline速度优化器
  });
  task_factory_.Register(POLY_ST_SPEED_OPTIMIZER,
                         []() -> Task* { return new PolyStSpeedOptimizer(); });                // Qp多项式st的速度优化器
}

bool LaneFollowScenario::Init(const PlanningConfig& config) {                                  // c初始化跟车的场景
  if (is_init_) {                                                                              // 如果已经初始化了就直接返回
    return true;
  }
  RegisterTasks();                                                                             // 注册的时候就会调用对应的函数, dpPath->pathDecider->dpst->speedDecider
  for (const auto task : config.lane_follow_scenario_config().task()) {                        // 迭代跟车场景中的任务
    // task : DP_POLY_PATH_OPTIMIZER
    // task : PATH_DECIDER
    // task : DP_ST_SPEED_OPTIMIZER
    // task : SPEED_DECIDER
    // task : QP_SPLINE_ST_SPEED_OPTIMIZER                                                     // 只初始化了5个任务
    tasks_.emplace_back(
        task_factory_.CreateObject(static_cast<TaskType>(task)));                              // 从map中找到对应的任务
    AINFO << "Created task:" << tasks_.back()->Name();
  }
  for (auto& task : tasks_) {
    if (!task->Init(config)) {                                                                 // 遍历所有的task, 然后一个一个挨着初始化
      std::string msg(
          common::util::StrCat("Init task[", task->Name(), "] failed."));                      // 出错了就要debug
      AERROR << msg;
      return false;
    }
  }
  is_init_ = true;                                                                             // 每个任务都被初始化
  return true;
}
// DpStSpeedOptimizer在工具包里面
void LaneFollowScenario::RecordObstacleDebugInfo(                                              // 记录下跟车场景中障碍物debug的信息
    ReferenceLineInfo* reference_line_info) {                                                  // 输入是参考线的info(信息)
  if (!FLAGS_enable_record_debug) {                                                            // FLAGS_enable_record_debug设置为true
    ADEBUG << "Skip record debug info";
    return;
  }
  auto ptr_debug = reference_line_info->mutable_debug();                                       // 获得debug的信息

  const auto path_decision = reference_line_info->path_decision();                             // 迭代所有参考线的path decision(path 的决策者)
  for (const auto path_obstacle : path_decision->path_obstacles().Items()) {                   // 迭代所有的障碍物
    auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();                  // 添加一个debug的障碍物
    obstacle_debug->set_id(path_obstacle->Id());                                               // 设置障碍物的id值
    obstacle_debug->mutable_sl_boundary()->CopyFrom(
        path_obstacle->PerceptionSLBoundary());                                                // 障碍物的sl的边框
    const auto& decider_tags = path_obstacle->decider_tags();                                  // 决策的标签
    const auto& decisions = path_obstacle->decisions();                                        // 障碍物的决策
    if (decider_tags.size() != decisions.size()) {                                             // 决策的标签和真正的决策要相等
      AERROR << "decider_tags size: " << decider_tags.size()
             << " different from decisions size:" << decisions.size();
    }
    for (size_t i = 0; i < decider_tags.size(); ++i) {                                         // 决策标签的大小
      auto decision_tag = obstacle_debug->add_decision_tag();                                  // 添加标签
      decision_tag->set_decider_tag(decider_tags[i]);
      decision_tag->mutable_decision()->CopyFrom(decisions[i]);
    }
  }
}

void LaneFollowScenario::RecordDebugInfo(ReferenceLineInfo* reference_line_info,               // lane follow的debug信息
                                         const std::string& name,                              // 参考中心线和名字和时间差
                                         const double time_diff_ms) {
  if (!FLAGS_enable_record_debug) {                                                            // 是否使能了debug
    ADEBUG << "Skip record debug info";
    return;
  }
  if (reference_line_info == nullptr) {
    AERROR << "Reference line info is null.";
    return;
  }

  auto ptr_latency_stats = reference_line_info->mutable_latency_stats();                       // 获取参考线延迟的状态

  auto ptr_stats = ptr_latency_stats->add_task_stats();                                        // 延迟的名字和时间差
  ptr_stats->set_name(name);
  ptr_stats->set_time_ms(time_diff_ms);
}

Status LaneFollowScenario::Process(const TrajectoryPoint& planning_start_point,                // 轨迹的起点和一帧数据
                                   Frame* frame) {
  bool has_drivable_reference_line = false;                                                    // 是否有可行驶的参考线
  bool disable_low_priority_path = false;                                                      // 禁止低优先级的路径
  auto status =
      Status(ErrorCode::PLANNING_ERROR, "reference line not drivable");                        // 初始化状态为Planning错误
  for (auto& reference_line_info : frame->reference_line_info()) {                             // 一个frame里面的中心参考线
    if (disable_low_priority_path) {                                                           // 不会使能低优先级的path
      reference_line_info.SetDrivable(false);                                                  // 该参考线不能开
    }                                                                                          // 会迭代一帧中所有的轨迹参考线
    if (!reference_line_info.IsDrivable()) {                                                   // 不能开的话就继续进行迭代
      continue;
    }
    auto cur_status =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);                // 在道路中心线的基础上做planning
    if (cur_status.ok() && reference_line_info.IsDrivable()) {                                 // 当前的状态ok， 并且参考线是可以开车的
      has_drivable_reference_line = true;                                                      // 参考线是否可以开车
      if (FLAGS_prioritize_change_lane &&                                                      // 是否使能了变道的优先级, FLAGS_prioritize_change_lane被配置为false
          reference_line_info.IsChangeLanePath() &&                                            // 参考线是否可以变道
          reference_line_info.Cost() < kStraightForwardLineCost) {                             // 并且代价比直行的代价还要低
        disable_low_priority_path = true;                                                      // 不是能低优先级的path
      }
    } else {
      reference_line_info.SetDrivable(false);                                                  // 否则设置可行驶的车辆为false
    }
  }
  return has_drivable_reference_line ? Status::OK() : status;                                  // has_drivable_reference_line为true才会返回有效的状态
}

Status LaneFollowScenario::PlanOnReferenceLine(                                                // 在车道中心线上做planning(规划)
    const TrajectoryPoint& planning_start_point, Frame* frame,                                 // 起点， 框
    ReferenceLineInfo* reference_line_info) {                                                  // 输出, 中心参考线
  if (!reference_line_info->IsChangeLanePath()) {                                              // 参考线是否改变了路径
    reference_line_info->AddCost(kStraightForwardLineCost);                                    // 增加代价， kStraightForwardLineCost为10， 最开始为0
  }
  ADEBUG << "planning start point:" << planning_start_point.DebugString();                     // debug开始点
  auto* heuristic_speed_data = reference_line_info->mutable_speed_data();                      // 启发式的速度
  auto speed_profile = speed_profile_generator_.GenerateInitSpeedProfile(                      // 初始化速度的曲线
      planning_start_point, reference_line_info);
  if (speed_profile.empty()) {                                                                 // 检查是否正常产生了速度曲线
    speed_profile =
        speed_profile_generator_.GenerateSpeedHotStart(planning_start_point);                  // 使用热启动
    ADEBUG << "Using dummy hot start for speed vector";
  }
  heuristic_speed_data->set_speed_vector(speed_profile);                                       // 启发式的速度

  auto ret = Status::OK();                                                                     // 返回值

  for (auto& optimizer : tasks_) {                                                             // tasks_里面全是优化器
    const double start_timestamp = Clock::NowInSeconds();                                      // 开始的时间戳
    ret = optimizer->Execute(frame, reference_line_info);                                      // 开始执行frame
    if (!ret.ok()) {                                                                           // 一个优化器出错了, 就会报错
      AERROR << "Failed to run tasks[" << optimizer->Name()
             << "], Error message: " << ret.error_message();
      break;
    }
    const double end_timestamp = Clock::NowInSeconds();                                        // 结束的时间戳
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;                      // 转换为毫秒

    ADEBUG << "after optimizer " << optimizer->Name() << ":"                                   // debug执行优化器后path和speed的信息(都可以在日志信息中看到)
           << reference_line_info->PathSpeedDebugString() << std::endl;
    ADEBUG << optimizer->Name() << " time spend: " << time_diff_ms << " ms.";                  // debug出一个优化器所用的时间

    RecordDebugInfo(reference_line_info, optimizer->Name(), time_diff_ms);                     // 记录下debug的信息, 就是添加一些延迟信息
  }
                                                                                               // 障碍物的sl边框和遇到障碍物的决策信息
  RecordObstacleDebugInfo(reference_line_info);                                                // 记录下障碍物debug的信息

  if (reference_line_info->path_data().Empty()) {                                              // planning的path数据为空
    ADEBUG << "Path fallback.";                                                                // debug一下path的反馈数据
    GenerateFallbackPathProfile(reference_line_info,                                           // 生成速度反馈曲线
                                reference_line_info->mutable_path_data());                     // 获取反馈的路径
    reference_line_info->AddCost(kPathOptimizationFallbackClost);                              // kPathOptimizationFallbackClost的值为20000
    reference_line_info->set_trajectory_type(ADCTrajectory::PATH_FALLBACK);                    // 返回的轨迹为path fallback(反馈的path)
  }

  if (!ret.ok() || reference_line_info->speed_data().Empty()) {                                // 速度的数据为空
    ADEBUG << "Speed fallback.";

    *reference_line_info->mutable_speed_data() =                                               // 产生反馈的速度曲线
        speed_profile_generator_.GenerateFallbackSpeedProfile();
    reference_line_info->AddCost(kSpeedOptimizationFallbackClost);
    reference_line_info->set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);                   // 设置速度反馈
  }

  reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);                             // 设置参考线的类型
  DiscretizedTrajectory trajectory;                                                            // 定义离散的轨迹
  if (!reference_line_info->CombinePathAndSpeedProfile(                                        // 合并速度和轨迹的曲线
          planning_start_point.relative_time(),                                                // 起点的相对时间
          planning_start_point.path_point().s(), &trajectory)) {
    std::string msg("Fail to aggregate planning trajectory.");                                 // 扩张planning的轨迹失败
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);                                             // 直接返回
  }

  // determine if there is a destination on reference line.
  double dest_stop_s = -1.0;                                                                   // 决定是否已经到达参考线的目的地
  for (const auto* path_obstacle :
       reference_line_info->path_decision()->path_obstacles().Items()) {                       // 迭代中心参考线中的所有障碍物
    if (path_obstacle->LongitudinalDecision().has_stop() &&                                    // 纵向决策里面有停止策略，并且停止的原因是到了目的地停车
        path_obstacle->LongitudinalDecision().stop().reason_code() ==
            STOP_REASON_DESTINATION) {
      SLPoint dest_sl = GetStopSL(path_obstacle->LongitudinalDecision().stop(),                // 获取停止点的sl坐标
                                  reference_line_info->reference_line());
      dest_stop_s = dest_sl.s();                                                               // 设置目标点的路程s
    }
  }

  for (const auto* path_obstacle :                                                             // 迭代路径中的所有障碍物
       reference_line_info->path_decision()->path_obstacles().Items()) {
    if (path_obstacle->obstacle()->IsVirtual()) {                                              // 如果是虚拟障碍物就直接跳过， 不做处理
      continue;
    }
    if (!path_obstacle->obstacle()->IsStatic()) {                                              // 如果是静态障碍物也会跳过不处理
      continue;
    }
    if (path_obstacle->LongitudinalDecision().has_stop()) {                                    // 纵向方向上有停止的策略
      bool add_stop_obstacle_cost = false;                                                     // 是否添加停止障碍物的cost(代价函数)
      if (dest_stop_s < 0.0) {                                                                 // 到目标点的距离为负数了
        add_stop_obstacle_cost = true;                                                         // 就使能代价
      } else {
        SLPoint stop_sl =                                                                      // 回去停止点的坐标
            GetStopSL(path_obstacle->LongitudinalDecision().stop(),
                      reference_line_info->reference_line());
        if (stop_sl.s() < dest_stop_s) {                                                       // 如果小于目标点的
          add_stop_obstacle_cost = true;
        }
      }
      if (add_stop_obstacle_cost) {
        constexpr double kRefrenceLineStaticObsCost = 1e3;                                     // 添加静态障碍物的代价1000
        reference_line_info->AddCost(kRefrenceLineStaticObsCost);
      }
    }
  }

  if (FLAGS_enable_trajectory_check) {                                                         // 是否检查轨迹， FLAGS_enable_trajectory_check设置为false
    if (ConstraintChecker::ValidTrajectory(trajectory) !=
        ConstraintChecker::Result::VALID) {                                                    // 检查轨迹是否合法
      std::string msg("Current planning trajectory is not valid.");
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  reference_line_info->SetTrajectory(trajectory);                                              // 全部合法就设置轨迹
  reference_line_info->SetDrivable(true);                                                      // 设置轨迹是可执行的
  return Status::OK();
}

void LaneFollowScenario::GenerateFallbackPathProfile(                                  // 生成path的反馈曲线
    const ReferenceLineInfo* reference_line_info, PathData* path_data) {               // 输入为中心参考线, 和path的数据
  auto adc_point = EgoInfo::instance()->start_point();                                 // 自动计驾驶车辆的起点
  double adc_s = reference_line_info->AdcSlBoundary().end_s();                         // 车辆sl边框的s(累加的s)
  const double max_s = 150.0;                                                          // 最大的s为150米
  const double unit_s = 1.0;                                                           // s的间隔单位为1米

  // projection of adc point onto reference line
  const auto& adc_ref_point =                                                          // 将自动驾驶车辆的一点投影到中心参考线上
      reference_line_info->reference_line().GetReferencePoint(adc_s);                  // 获取自动驾驶车辆在参考线上的一点

  DCHECK(adc_point.has_path_point());                                                  // 检查自动驾驶车辆的点是否存在
  const double dx = adc_point.path_point().x() - adc_ref_point.x();                    // 获得实际位置和中心参考线上的偏差
  const double dy = adc_point.path_point().y() - adc_ref_point.y();

  std::vector<common::PathPoint> path_points;                                          // 路径上的点
  for (double s = adc_s; s < max_s; s += unit_s) {                                     // 规划150这么远?
    const auto& ref_point =
        reference_line_info->reference_line().GetReferencePoint(adc_s);                // 这个放外面不是更好么?
    common::PathPoint path_point = common::util::MakePathPoint(                        // 创建一个路径的参考点
        ref_point.x() + dx, ref_point.y() + dy, 0.0, ref_point.heading(),
        ref_point.kappa(), ref_point.dkappa(), 0.0);
    path_point.set_s(s);                                                               // 更新path点中的s(路程)

    path_points.push_back(std::move(path_point));                                      // 然后一个一个地放到数组中
  }                                                                                    // 然后转换为离散的path
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));              // 把path的点转换为离散的path
}

SLPoint LaneFollowScenario::GetStopSL(                                                 // 获取停止点的sl坐标
    const ObjectStop& stop_decision,                                                   // 停止的决策
    const ReferenceLine& reference_line) const {                                       // 参考的中心线
  SLPoint sl_point;                                                                    // 临时的sl坐标点
  reference_line.XYToSL(                                                               // 通过参考线的内部函数XYToSL进行转换
      {stop_decision.stop_point().x(), stop_decision.stop_point().y()},
      &sl_point);
  return sl_point;
}

ScenarioConfig::ScenarioType LaneFollowScenario::Transfer(                             // 场景的切换
    const ScenarioConfig::ScenarioType& current_scenario,
    const common::TrajectoryPoint& ego_point, const Frame& frame) const {              // 具体的代码没有实现呀
  // implement here
  return ScenarioConfig::LANE_FOLLOW;                                                  // 直接返回跟车的场景
}

}  // namespace planning
}  // namespace apollo
