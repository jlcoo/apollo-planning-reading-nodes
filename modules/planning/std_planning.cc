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

#include "modules/planning/std_planning.h"

#include <algorithm>
#include <list>
#include <memory>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/routing/proto/routing.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/trajectory_stitcher.h"
#include "modules/planning/planner/navi/navi_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/planning/toolkits/deciders/traffic_decider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;                                                                // 错误码， 在chassis.proto文件中定义
using apollo::common::Status;                                                                   // 状态status, 在/common/status/status.h中定义
using apollo::common::TrajectoryPoint;                                                          // 轨迹点, 在common/proto/pnc_point.proto中定义(速度信息, 加速度信息， path point(x,y,z上的坐标，方向，曲率，积累的距离s,曲率的微分（一次微分和二次微分)，车道上的ID值)
using apollo::common::VehicleState;                                                             // 车辆的状态，在moudels/common/vehicle_status/proto/vehicle_status.proto文件中定义
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;                                                  // 在moudels/common/adapters/adapter_manager.h中定义, 注册了很多的适配器
using apollo::common::time::Clock;                                                              // 整个系统的时钟, 是在moudels/common/time/time.h中定义
using apollo::hdmap::HDMapUtil;                                                                 // 高精地图中可以利用的单元, 在moudels/map/hdmap/hamap_util.h中定义
using apollo::routing::RoutingResponse;                                                         // routing模块获得的结果处理后的输出， 在moudels/routing/proto/routing.proto文件中定义

StdPlanning::~StdPlanning() { Stop(); }                                                         // 在析构函数中进行停止操作

std::string StdPlanning::Name() const { return "std_planning"; }                                // 构造planning的名字

bool IsDifferentRouting(const RoutingResponse& first,                                           // 检查两次的routing结果是否相同
                        const RoutingResponse& second) {                                        // 输入是第一次和第二次的routing结果
  if (first.has_header() && second.has_header()) {
    if (first.header().sequence_num() != second.header().sequence_num()) {                      // 如果routing的结果不相同，那肯定routing不相同
      return true;
    }
    if (first.header().timestamp_sec() != second.header().timestamp_sec()) {                    // 如果两次routing的时间戳不相同， 那肯定routing的不相同
      return true;
    }
    return false;                                                                               // 否则相同
  } else {
    return true;                                                                                // 有一个的头节点不存在, 则不相同
  }
}

Status StdPlanning::Init() {                                                                    // std规划器的初始化函数
  common::util::ThreadPool::Init(FLAGS_max_planning_thread_pool_size);                          // 搞个线程池， 线程池最大的数量为15
  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,                      // FLAGS_planning_config_file定义为modules/planning/conf/planning_config.pb.txt
                                               &config_))                                       // 看看planning相关的proto参数是否可用
      << "failed to load planning config file " << FLAGS_planning_config_file;                  // 从planning_config.pb.txt文件中获得
  // planner的类型配置为EM
  // lane_follow_scenario_config的配置为:
    // task : DP_POLY_PATH_OPTIMIZER
    // task : PATH_DECIDER
    // task : DP_ST_SPEED_OPTIMIZER
    // task : SPEED_DECIDER
    // task : QP_SPLINE_ST_SPEED_OPTIMIZER 
  CheckPlanningConfig();                                                                        //所以产出的第一个参数应该是proto的参数文件

  planner_dispatcher_->Init();                                                                  // 注册5种不同的planner策略
  // 看看交通规则的文件是否存在
  CHECK(apollo::common::util::GetProtoFromFile(
      FLAGS_traffic_rule_config_filename, &traffic_rule_configs_))
      << "Failed to load traffic rule config file "
      << FLAGS_traffic_rule_config_filename;

  // clear planning status
  GetPlanningStatus()->Clear();                                                                 // 将planning相关的状态标志全部清零
  // 适配器编程模式， AdapterManager是用来管理适配器的类
  if (!AdapterManager::Initialized()) {   // 会启动对应的topic
    AdapterManager::Init(FLAGS_planning_adapter_config_filename);                               // 通过适配器的配置文件初始化适配器的管理者(这个类对象)
  }                                                                                             // FLAGS_planning_adapter_config_filename被定义为modules/planning/conf/adapter.conf
  CHECK_ADAPTER(Localization);            // 检查定位是否启动了
  CHECK_ADAPTER(Chassis);                 // 检查底盘
  CHECK_ADAPTER(RoutingResponse);         // 车道线的response
  CHECK_ADAPTER(RoutingRequest);          // 车道线的request
  CHECK_ADAPTER(Prediction);              // 感知模块
  CHECK_ADAPTER(TrafficLightDetection);   // 交通灯检测

  hdmap_ = HDMapUtil::BaseMapPtr();       // basemap地图是否可用， 在配置项中设置
  CHECK(hdmap_) << "Failed to load map";
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>(hdmap_);                  // 从高精地图中获得中心参考线
  planner_ = planner_dispatcher_->DispatchPlanner();                                           // 调度planner规划层, 将内部的planner_复制为EM规划器
  // 这里会创建一个EM的规划决策器
  if (!planner_) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }

  return planner_->Init(config_);                                                             // 初始化EM planning的内容
    // 在Status EMPlanner::Init(const PlanningConfig& config)函数中, 先初始化情景管理器
    // scenario_manager_.Init();  
        // ScenarioManager scenario_manager_;   // 场景管理项, 来自Preception
            // 在bool ScenarioManager::Init()中调用一个RegisterScenarios(); 
            // 和scenario_factory_.CreateObject(ScenarioConfig::LANE_FOLLOW);
                // LaneFollowScenario()对象 ScenarioConfig::LANE_FOLLOW
}

Status StdPlanning::InitFrame(const uint32_t sequence_num,                                    // 初始化一帧数据
                              const TrajectoryPoint& planning_start_point,                    // 轨迹的起点
                              const double start_time,                                        // 开始的时间
                              const VehicleState& vehicle_state) {                            // 车辆的状态
  frame_.reset(new Frame(sequence_num, planning_start_point, start_time,                      // 新建一帧数据
                         vehicle_state, reference_line_provider_.get()));                     // 中心参考提供者获取一个中心参考线(reference line)
  auto status = frame_->Init();                                                               // 新建一帧的数据要进行初始化
  if (!status.ok()) {                                                                         // 进行容错处理
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  return Status::OK();                                                                        // 初始化成功的话就返回OK
}

void StdPlanning::OnTimer(const ros::TimerEvent&) {                                           // 在规划器中创建一个定时器
  RunOnce();                                                                                  // 在定时器中运行一次stdplanning
  // FLAGS_planning_test_mode设置为false， FLAGS_test_duration设置为-1.0
  if (FLAGS_planning_test_mode && FLAGS_test_duration > 0.0 &&
      Clock::NowInSeconds() - start_time_ > FLAGS_test_duration) {                            // 测试模式就会停止planning模块
    Stop();
    ros::shutdown();
  }
}

Status StdPlanning::Start() {                                                                 // 标准的规划器的开始函数
  // 创建一个定时器
  timer_ =                                                                                    // FLAGS_planning_loop_rate设置为10, 100ms做一次planning
      AdapterManager::CreateTimer(ros::Duration(1.0 / FLAGS_planning_loop_rate),
                                  &StdPlanning::OnTimer, this);                               // OnTimer是在创建定时器的时候调用
  // 车道线开始
  reference_line_provider_->Start();                                                          // 车道线的产生这
  // 记录开始时间
  start_time_ = Clock::NowInSeconds();                                                        // 记录下开始做planning的时间
  AINFO << "Planning started";
  return Status::OK();                                                                        // 并返回OK的值
}
// 处理即将到来的数据
void StdPlanning::RunOnce() {                                                                 // Run Once是指一直运行这个函数
  // snapshot all coming data
  AdapterManager::Observe();                                                                  // 获取数据快照：observer, 探测适配器里面的所有对象

  const double start_timestamp = Clock::NowInSeconds();                                       // 开始的时间戳

  ADCTrajectory not_ready_pb;                                                                 // 自动驾驶车辆轨迹的数据
  // 校验数据是否为空：没有clean的情况下，重复校验意义不大
  auto* not_ready = not_ready_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();
  // 通过适配器的manager来调用相关的函数
  if (AdapterManager::GetLocalization()->Empty()) {                                           // 检查定位信息
    not_ready->set_reason("localization not ready");
  } else if (AdapterManager::GetChassis()->Empty()) {                                         // 获取底盘的信息
    not_ready->set_reason("chassis not ready");
  } else if (AdapterManager::GetRoutingResponse()->Empty()) {                                 // 获取routing的结果
    not_ready->set_reason("routing not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {                                            // 获取高精地图
    not_ready->set_reason("map not ready");                                                   // 没有准备好的原因
  }

  if (not_ready->has_reason()) {                                                              // 是否有没有准备好的原因
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    PublishPlanningPb(&not_ready_pb, start_timestamp);                                        // 没有准备好的轨迹照样需要发布出去
    return;
  }
  // 更新车辆状态：使用location和chassis信息，预估下一时刻车辆状态，校验车辆信息是否ok
  // localization
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();                                 // 获取最新的定位信息
  ADEBUG << "Get localization:" << localization.DebugString();

  // chassis
  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();                    // 获取底盘最新的信息
  ADEBUG << "Get chassis:" << chassis.DebugString();

  Status status =
      VehicleStateProvider::instance()->Update(localization, chassis);                        // 根据定位和底盘的信息，更新车辆的状态

  VehicleState vehicle_state =                                                                // 直接获取车辆的状态
      VehicleStateProvider::instance()->vehicle_state();

  // estimate (x, y) at current timestamp
  // This estimate is only valid if the current time and vehicle state timestamp
  // differs only a small amount (20ms). When the different is too large, the
  // estimation is invalid.
  DCHECK_GE(start_timestamp, vehicle_state.timestamp());                                      // 当前的时间戳比车状态的时间戳大20ms以内才会合法, 否则会不合法
  if (FLAGS_estimate_current_vehicle_state &&                                                 // FLAGS_estimate_current_vehicle_state设置为true
      start_timestamp - vehicle_state.timestamp() < 0.020) {
    auto future_xy = VehicleStateProvider::instance()->EstimateFuturePosition(                // 估计车辆未来的位置在哪里
        start_timestamp - vehicle_state.timestamp());                                         // VehicleStateProvider的单例调用
    vehicle_state.set_x(future_xy.x());                                                       // 设置车的状态
    vehicle_state.set_y(future_xy.y());
    vehicle_state.set_timestamp(start_timestamp);                                             // 设置当前的时间戳
  }

  if (!status.ok() || !IsVehicleStateValid(vehicle_state)) {                                  // 车的状态不合法
    std::string msg("Update VehicleStateProvider failed");
    AERROR << msg;
    not_ready->set_reason(msg);                                                               // 设置不合法的原因
    status.Save(not_ready_pb.mutable_header()->mutable_status());
    PublishPlanningPb(&not_ready_pb, start_timestamp);                                        // 车辆状态没有准备好，还是要将对应的轨迹发布出去
    return;
  }
  // 把RoutingResponse和vehicle_state传递给 reference_line_provider_
  const auto& latest_routing =                                                                // 从routing的结果中, 获取最新的routing结果
      AdapterManager::GetRoutingResponse()->GetLatestObserved();
  ADEBUG << "Get routing:" << latest_routing.DebugString();                                   // debug显示routing的结果
  if (IsDifferentRouting(last_routing_, latest_routing)) {                                    // 看看现在获得最新的routing结果是否和上一次处理的结果一样, 如果一样的话就会直接跳过, 不一样的话， 就会拷贝
    last_routing_ = latest_routing;                                                           // 拷贝最新的数据
    GetPlanningStatus()->Clear();                                                             // 将之前做planning的状态清空
    reference_line_provider_->UpdateRoutingResponse(latest_routing);                          // 更新参考线提供者中的routing结果(用于计算新的reference line)
  }

  if (AdapterManager::GetPrediction()->Empty()) {                                             // 获取预测的结果
    AWARN_EVERY(100) << "prediction is enabled but no prediction provided";                   // 预测模块使能了, 但是没有预测模块的提供者
  }
  // Update reference line provider and reset pull over if necessary
  reference_line_provider_->UpdateVehicleState(vehicle_state);                                // 更新reference line的提供者， 必要时进行重置处理

  const double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;                          // 频率为10Hz， 做planning所用的时间为100ms
  // 创建一个stitching_trajectory轨迹，用于和前一条轨迹对齐，保证线的平滑
  std::vector<TrajectoryPoint> stitching_trajectory;                                          // 合并轨迹点
  stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(                      // TrajectoryStitcher为一个类在models/common/planning/common/trajectory/trajectory_stitcher.h中定义
      vehicle_state, start_timestamp, planning_cycle_time,                                    // 车的状态， 开始的时间戳，做一次planning的时间，上次发布的轨迹
      last_publishable_trajectory_.get());
  // 初始化Frame，并对frame做检查：不过这个检查是无效的
  const uint32_t frame_num = AdapterManager::GetPlanning()->GetSeqNum() + 1;                  // 获得做planning的当前帧数
  status = InitFrame(frame_num, stitching_trajectory.back(), start_timestamp,                 // 通过帧数， 最新的合并轨迹的最后一个轨迹点， 开始的时间戳，车辆的状态进行初始化一帧
                     vehicle_state);
  if (!frame_) {                                                                              // 初始化不成功
    std::string msg("Failed to init frame");                                                  // debug出对应的状态
    AERROR << msg;
    not_ready->set_reason(msg);
    status.Save(not_ready_pb.mutable_header()->mutable_status());
    PublishPlanningPb(&not_ready_pb, start_timestamp);                                        // 不成功还是要发布轨迹, 只是是一个没有准备好的轨迹
    return;
  }
  // 处理超声波雷达的逻辑：刹车
  EgoInfo::instance()->Update(stitching_trajectory.back(), vehicle_state,                     // 根据合并轨迹最后的轨迹点，车辆的状态，一帧数据中的障碍物，再更新ego的信息
                              frame_->obstacles());

  auto* trajectory_pb = frame_->mutable_trajectory();                                         // 获得一帧数据中轨迹的指针， 而且可以通过这个指针修改这个轨迹
  if (FLAGS_enable_record_debug) {                                                            // 记录debug的信息到debug的protobuf中
    frame_->RecordInputDebug(trajectory_pb->mutable_debug());
  }
  trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(                             // 初始话一帧数据的延迟状态
      Clock::NowInSeconds() - start_timestamp);
  if (!status.ok()) {                                                                         // 如果测的状态有问题， 就即使debug
    AERROR << status.ToString();
    if (FLAGS_publish_estop) {
      // Because the function "Control::ProduceControlCommand()" checks the
      // "estop" signal with the following line (Line 170 in control.cc):
      // estop_ = estop_ || trajectory_.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      ADCTrajectory estop_trajectory;                                                         // 应该添加更多的信息， 因为紧急刹车的触发
      EStop* estop = estop_trajectory.mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
      status.Save(estop_trajectory.mutable_header()->mutable_status());
      PublishPlanningPb(&estop_trajectory, start_timestamp);
    } else {
      trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());                                                    // 设置轨迹为什么没有准备好的状态
      status.Save(trajectory_pb->mutable_header()->mutable_status());
      PublishPlanningPb(trajectory_pb, start_timestamp);                                      // 车的状态没有准备好， 但是还是发布出去了?
    }

    auto seq_num = frame_->SequenceNum();                                                     // 获取当前的帧数
    FrameHistory::instance()->Add(seq_num, std::move(frame_));                                // 并且添加到历史轨迹的数据中

    return;                                                                                   // 车辆的状态没有保存好， 就马上返回
  }
  // 遍历ref_line_info，进行TrafficDecider和ParkingDecider，结果写回ref_line_info
  for (auto& ref_line_info : frame_->reference_line_info()) {                                 // 迭代当前帧的所有的所有参考线信息
    TrafficDecider traffic_decider;                                                           // 交通决策者
    traffic_decider.Init(traffic_rule_configs_);                                              // 通过交通配置项进行配置
    auto traffic_status = traffic_decider.Execute(frame_.get(), &ref_line_info);              // 获取对应参考线上的交通规则
    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {                                // 如果交通规则是空, 或者参考线必能开车
      ref_line_info.SetDrivable(false);                                                       // 就会发出警告， 并跳过这个中心参考线
      AWARN << "Reference line " << ref_line_info.Lanes().Id()
            << " traffic decider failed";
      continue;
    }
  }
  // 进行Plan
  status = Plan(start_timestamp, stitching_trajectory, trajectory_pb);                        // trajectory_pb是frame中的轨迹

  const auto time_diff_ms = (Clock::NowInSeconds() - start_timestamp) * 1000;                 // 获得ms(毫秒级的时间)
  ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";                          // 显示时间

  trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);                    // 设置总时间
  ADEBUG << "Planning latency: "
         << trajectory_pb->latency_stats().DebugString();                                     // debug延迟的状态

  auto* ref_line_task =
      trajectory_pb->mutable_latency_stats()->add_task_stats();                               // 参考线的状态
  ref_line_task->set_time_ms(reference_line_provider_->LastTimeDelay() *
                             1000.0);                                                         // 延时的时间
  ref_line_task->set_name("ReferenceLineProvider");                                           // 设置参考线的任务

  if (!status.ok()) {                                                                         // planner(规划器)失败
    status.Save(trajectory_pb->mutable_header()->mutable_status());                           // 保存自动驾驶车辆的轨迹
    AERROR << "Planning failed:" << status.ToString();                                        // debug错误的信息
    if (FLAGS_publish_estop) {                                                                // FLAGS_publish_estop设置为false， 在planning模块中不会发布estop的决策
      AERROR << "Planning failed and set estop";
      // Because the function "Control::ProduceControlCommand()" checks the
      // "estop" signal with the following line (Line 170 in control.cc):
      // estop_ = estop_ || trajectory_.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      EStop* estop = trajectory_pb->mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
    }
  }

  trajectory_pb->set_is_replan(stitching_trajectory.size() == 1);                             // 是否要进行轨迹的重新planner
  PublishPlanningPb(trajectory_pb, start_timestamp);                                          // 将自动驾驶车辆的轨迹发布出去
  ADEBUG << "Planning pb:" << trajectory_pb->header().DebugString();                          // debug相关的信息

  auto seq_num = frame_->SequenceNum();                                                       // 数据框的内容
  FrameHistory::instance()->Add(seq_num, std::move(frame_));                                  // 将这帧数据放到历史链表中
}

void StdPlanning::ExportReferenceLineDebug(planning_internal::Debug* debug) {                 // 导出参考线的debug信息
  if (!FLAGS_enable_record_debug) {                                                           // FLAGS_enable_record_debug被设置为true
    return;
  }
  for (auto& reference_line_info : frame_->reference_line_info()) {                           // 迭代reference_line_info这个双链表
    auto rl_debug = debug->mutable_planning_data()->add_reference_line();                     // 将链表中的参考线信息, 全部放到debug的信息里
    rl_debug->set_id(reference_line_info.Lanes().Id());
    rl_debug->set_length(reference_line_info.reference_line().Length());                      // planning_internal::Debug在moudels/planning/proto/planning_internal.proto文件中定义
    rl_debug->set_cost(reference_line_info.Cost());
    rl_debug->set_is_change_lane_path(reference_line_info.IsChangeLanePath());
    rl_debug->set_is_drivable(reference_line_info.IsDrivable());
    rl_debug->set_is_protected(reference_line_info.GetRightOfWayStatus() ==
                               ADCTrajectory::PROTECTED);
  }
}
// 在std_planning中封装了一层Plan函数, 在这个函数中, 调用planner_->Plan函数进行操作
Status StdPlanning::Plan(                                                                        // 在RunOnce调用的plan函数
    const double current_time_stamp,                                                             // 当前的时间戳
    const std::vector<TrajectoryPoint>& stitching_trajectory,                                    // 合并的轨迹
    ADCTrajectory* trajectory_pb) {                                                              // 自动驾驶的轨迹
  auto* ptr_debug = trajectory_pb->mutable_debug();                                              // 获取adc轨迹的debug信息
  if (FLAGS_enable_record_debug) {                                                               // FLAGS_enable_record_debug被设置为true
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(                          // debug的起点就是合并点的最后一个轨迹点
        stitching_trajectory.back());
  }
  // 调用planner->Plan
  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get());                       // 调用配置好的规划器进行规划路径, 通过合并轨迹的最后的点，一帧数据

  ptr_debug->mutable_planning_data()->set_front_clear_distance(
      EgoInfo::instance()->front_clear_distance());                                              // 设置前方障碍物的距离
  ExportReferenceLineDebug(ptr_debug);                                                           // 导出参考线的信息

  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();                              // 找到一帧数据中最好的参考线信息(可行驶)
  if (!best_ref_info) {                                                                          // 如果参考线不为空的话，才会进行后面的操作
    std::string msg("planner failed to make a driving plan");
    AERROR << msg;                                                                               // 进行容错处理
    if (last_publishable_trajectory_) {
      last_publishable_trajectory_->Clear();                                                     // 如果最新的发布的轨迹还存在的话， 就直接删除
    }
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // 数据merger到trajectory_pb中
  ptr_debug->MergeFrom(best_ref_info->debug());                                                  // 追加上最好参考线的debug信息
  trajectory_pb->mutable_latency_stats()->MergeFrom(                                             // 延迟状态
      best_ref_info->latency_stats());
  // set right of way status
  trajectory_pb->set_right_of_way_status(best_ref_info->GetRightOfWayStatus());                  // 设置自动驾驶轨迹右边的路况, 从最好的轨迹中获取way的状态， 然后复制给自动驾驶车辆的轨迹
  for (const auto& id : best_ref_info->TargetLaneId()) {                                         // 迭代最好的参考线轨迹上的车道id
    trajectory_pb->add_lane_id()->CopyFrom(id);                                                  // 把每个id值复制给轨迹的ID
  }

  trajectory_pb->set_trajectory_type(best_ref_info->trajectory_type());                          // 设置自动驾驶车辆轨迹的类型, 有四种类型

  best_ref_info->ExportDecision(trajectory_pb->mutable_decision());                              // 导出自动驾驶车辆轨迹的决策类型

  // Add debug information.
  if (FLAGS_enable_record_debug) {                                                               // 是否使能了debug的信息
    auto* reference_line = ptr_debug->mutable_planning_data()->add_path();                       // FLAGS_enable_record_debug被设置为true
    reference_line->set_name("planning_reference_line");                                         // 设置参考线的名字
    const auto& reference_points =
        best_ref_info->reference_line().reference_points();                                      // 设置参考线的离散点
    double s = 0.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
    bool empty_path = true;
    for (const auto& reference_point : reference_points) {                                       // 迭代轨迹中的所有点
      auto* path_point = reference_line->add_path_point();
      path_point->set_x(reference_point.x());                                                    // 设置path point点的值
      path_point->set_y(reference_point.y());
      path_point->set_theta(reference_point.heading());
      path_point->set_kappa(reference_point.kappa());
      path_point->set_dkappa(reference_point.dkappa());
      if (empty_path) {                                                                          // 设置path的起点
        path_point->set_s(0.0);
        empty_path = false;
      } else {
        double dx = reference_point.x() - prev_x;                                                // 不断计算s的值， 连个点的距离
        double dy = reference_point.y() - prev_y;
        s += std::hypot(dx, dy);
        path_point->set_s(s);                                                                    // s就是所有线段长度的累加和
      }
      prev_x = reference_point.x();                                                              // 获取参考点的x,y的坐标
      prev_y = reference_point.y();
    }
  }

  last_publishable_trajectory_.reset(new PublishableTrajectory(                                  // 最新发布的轨迹, 通过最好的轨迹进行赋值
      current_time_stamp, best_ref_info->trajectory()));

  ADEBUG << "current_time_stamp: " << std::to_string(current_time_stamp);                        // 当前的时间戳
  // 记录last_tractory
  // Navi Panner doesn't need to stitch the last path planning                                   // 导航的规划期不用合并轨迹， 但是会导致dreamview的bug
  // trajectory.Otherwise, it will cause the Dremview planning track to display
  // flashing or bouncing
  if (FLAGS_enable_stitch_last_trajectory) {                                                     // 设置为true
    last_publishable_trajectory_->PrependTrajectoryPoints(                                       // 追加新的轨迹点
        stitching_trajectory.begin(), stitching_trajectory.end() - 1);
  }

  for (size_t i = 0; i < last_publishable_trajectory_->NumOfPoints(); ++i) {                     // 迭代最新发布的轨迹中的所有点
    if (last_publishable_trajectory_->TrajectoryPointAt(i).relative_time() >
        FLAGS_trajectory_time_high_density_period) {                                             // 最大的采样周期是1秒
      break;
    }
    ADEBUG << last_publishable_trajectory_->TrajectoryPointAt(i)                                 // debug的信心
                  .ShortDebugString();
  }

  last_publishable_trajectory_->PopulateTrajectoryProtobuf(trajectory_pb);                       // 构造最新的轨迹
  // 设置能开启自动驾驶 ： ExportEngageAdvice， 选择最好的reference information
  best_ref_info->ExportEngageAdvice(trajectory_pb->mutable_engage_advice());                     // 导出engage的建议行驶策略

  return status;                                                                                 // 最终返回planner的状态
}

void StdPlanning::Stop() {                                                                    // 停止标准的规划器
  AWARN << "Planning Stop is called";
  common::util::ThreadPool::Stop();                                                           // 停止线程池
  reference_line_provider_->Stop();                                                           // 停止中心参考线的提供者
  last_publishable_trajectory_.reset(nullptr);                                                // 最新的可发布的轨迹为空指针
  frame_.reset(nullptr);                                                                      // 一阵数据为空指针
  planner_.reset(nullptr);                                                                    // planner规划器为空指针
  FrameHistory::instance()->Clear();                                                          // 历史数据清空
  GetPlanningStatus()->Clear();                                                               // 获取planning的状态
  last_routing_.Clear();                                                                      // 上次planning的结果清空
  EgoInfo::instance()->Clear();                                                               // 自身的
}

}  // namespace planning EMPlanner
}  // namespace apollo
