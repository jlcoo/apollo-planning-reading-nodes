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
 * @file
 **/

#include "modules/planning/common/reference_line_info.h"

#include <algorithm>
#include <functional>
#include <utility>

#include "modules/planning/proto/sl_boundary.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/thread_pool.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::canbus::Chassis;                                                      // canbus中底盘的信息(在chassis.proto文件中定义)
using apollo::common::EngageAdvice;                                                 // 紧急措施
using apollo::common::SLPoint;                                                      // sl的坐标
using apollo::common::TrajectoryPoint;                                              // 轨迹点
using apollo::common::VehicleConfigHelper;                                          // 车辆配置项的帮助者
using apollo::common::VehicleSignal;                                                // 车辆的信号灯信息
using apollo::common::adapter::AdapterManager;                                      // 适配器的管理者, 所有相关的模块都要向这个适配器注册适配器adapter
using apollo::common::math::Box2d;                                                  // 二维的box, Box2d会利用aabox进行构造
using apollo::common::math::Vec2d;                                                  // 二维的向量点, 就是x, y坐标
using apollo::common::util::ThreadPool;                                             // 线程池
                                                                                    // ReferenceLineInfo(中心参考线信息类的构造函数)
ReferenceLineInfo::ReferenceLineInfo(const common::VehicleState& vehicle_state,     // 会通过车辆的状态
                                     const TrajectoryPoint& adc_planning_point,     // 自动驾驶车辆的轨迹点
                                     const ReferenceLine& reference_line,           // 当前车道中心参考线
                                     const hdmap::RouteSegments& segments)          // routing的一个小片段, 这四个信息进行构造, RouteSegments是LaneSegment的数组
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      reference_line_(reference_line),                                              // LaneSegment
      lanes_(segments) {}                                                           // LaneSegment有LaneInfoConstPtr的指针, s方向上的起点, 和终点信息

bool ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles) {       // 通过放障碍物的数组初始化ReferenceLineInfo类
  const auto& param = VehicleConfigHelper::GetConfig().vehicle_param();             // 获取车身参数
  // stitching point       // TrajectoryPoint
  const auto& path_point = adc_planning_point_.path_point();                        // 自动驾驶车辆在path上的一点
  Vec2d position(path_point.x(), path_point.y());                                   // 将一个点转换成一个二维向量
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,           // 车辆的中心点
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);
  Vec2d center(position + vec_to_center.rotate(path_point.theta()));                // 车的中心点
  Box2d box(center, path_point.theta(), param.length(), param.width());             // 新建一个车的边框的box
  // realtime vehicle position
  Vec2d vehicle_position(vehicle_state_.x(), vehicle_state_.y());                   // 车辆的位置
  Vec2d vehicle_center(vehicle_position +
                       vec_to_center.rotate(vehicle_state_.heading()));
  Box2d vehicle_box(vehicle_center, vehicle_state_.heading(), param.length(),       // 车辆的box
                    param.width());

  if (!reference_line_.GetSLBoundary(vehicle_box,                                   // 参考线的sl坐标系中的boundary
                                     &sl_boundary_info_.vehicle_sl_boundary_)) {    // sl_boundary_info_(是自动驾驶车的boundary和vehicle(车辆)的boundary)
    AERROR << "Failed to get ADC boundary from vehicle_box(realtime position "
              "of the car): "
           << box.DebugString();
    return false;
  }

  if (!reference_line_.GetSLBoundary(box,                                           // 车的sl的boundary
                                     &sl_boundary_info_.adc_sl_boundary_)) {
    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
    return false;
  }
                                                                                    // 车辆的boundary不在中心参考线上
  if (sl_boundary_info_.adc_sl_boundary_.end_s() < 0 ||
      sl_boundary_info_.adc_sl_boundary_.start_s() > reference_line_.Length()) {
    AWARN << "Vehicle SL "
          << sl_boundary_info_.adc_sl_boundary_.ShortDebugString()
          << " is not on reference line:[0, " << reference_line_.Length()
          << "]";
  }
  constexpr double kOutOfReferenceLineL = 10.0;  // in meters                       // 横向 10米就算完全偏离了中心参考线
  if (sl_boundary_info_.adc_sl_boundary_.start_l() > kOutOfReferenceLineL ||
      sl_boundary_info_.adc_sl_boundary_.end_l() < -kOutOfReferenceLineL) {
    AERROR << "Ego vehicle is too far away from reference line.";
    return false;
  }
  is_on_reference_line_ =
      reference_line_.IsOnLane(sl_boundary_info_.adc_sl_boundary_);                 // 自动驾驶车辆是否在中心参考线上
  if (!AddObstacles(obstacles)) {                                                   // 障碍物到中心参考线上
    AERROR << "Failed to add obstacles to reference line";
    return false;
  }

  if (hdmap::GetSpeedControls()) {                                                  // 速度控制
    auto* speed_controls = hdmap::GetSpeedControls();
    for (const auto& speed_control : speed_controls->speed_control()) {
      reference_line_.AddSpeedLimit(speed_control);                                 // 添加参考线上每个点的速度
    }
  }

  // set lattice planning target speed limit;                                       // FLAGS_default_cruise_speed被设置为5m/s
  SetCruiseSpeed(FLAGS_default_cruise_speed);                                       // 设置巡航的速度限制
  is_safe_to_change_lane_ = CheckChangeLane();                                      // 变道是否是安全的
  is_inited_ = true;                                                                // 设置初始化完成
  return true;
}

bool ReferenceLineInfo::IsInited() const { return is_inited_; }                     // 返回是否初始化的内部成员变量

bool WithinOverlap(const hdmap::PathOverlap& overlap, double s) {                   // PathOverlap是一个数据结构,在path.h中定义
  constexpr double kEpsilon = 1e-2;                                                 // 无限小量
  return overlap.start_s - kEpsilon <= s && s <= overlap.end_s + kEpsilon;          // 给定的s是否在overlap的起点和终点范围内
}

void ReferenceLineInfo::SetJunctionRightOfWay(double junction_s,                    // 设置右边的交汇点
                                              bool is_protected) {                  // 是否已经进行了投影
  auto* right_of_way = GetPlanningStatus()->mutable_right_of_way();                 // 从planning状态中获取右边的道路
  auto* junction_right_of_way = right_of_way->mutable_junction();                   // PlanningStatus在planning_status.proto文件中定义
  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {      // overlap是什么意思? 一直不知道呢
    if (WithinOverlap(overlap, junction_s)) {                                       // 判断overlap是否在交叉处
      (*junction_right_of_way)[overlap.object_id] = is_protected;                   // 是的话就设置为是受保护的
    }
  }
}

ADCTrajectory::RightOfWayStatus ReferenceLineInfo::GetRightOfWayStatus() const {    // 道路右边的状态
  auto* right_of_way = GetPlanningStatus()->mutable_right_of_way();                 // 在planning_status.proto文件中定义
  auto* junction_right_of_way = right_of_way->mutable_junction();                   // 右边道路的junction
  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {      // 迭代junction处的所有overlap
    if (overlap.end_s < sl_boundary_info_.adc_sl_boundary_.start_s()) {             // overlap和自动驾驶车辆的没有重叠的话
      junction_right_of_way->erase(overlap.object_id);                              // 就把这个物体的id号删除
    } else if (WithinOverlap(overlap,
                             sl_boundary_info_.adc_sl_boundary_.end_s())) {         // 如果在自动驾驶车辆的sl坐标的boundary中的话
      auto is_protected = (*junction_right_of_way)[overlap.object_id];              // 看看overlap对应的物体是否是受保护的
      if (is_protected) {                                                           // 如果是的话
        return ADCTrajectory::PROTECTED;                                            // 返回自动驾驶车辆的轨迹是受保护的
      } else {                                                                      // 如果不是受保护的话
        const auto lane_segments =                                                  // 获取overlap起点到终点(在s方向上)的车道段
            reference_line_.GetLaneSegments(overlap.start_s, overlap.end_s);        // 在path.cc中定义了GetLaneSegments的实现
        for (const auto& segment : lane_segments) {                                 // 迭代lane的所有的片段
          if (segment.lane->lane().turn() != hdmap::Lane::NO_TURN) {                // 如果该lane片段中的转向信息是要进行转向
            return ADCTrajectory::UNPROTECTED;                                      // 那么就返回unprotected的自动驾驶轨迹的状态
          }
        }
        return ADCTrajectory::PROTECTED;                                            // 否则返回protected的状态
      }
    }
  }
  return ADCTrajectory::UNPROTECTED;                                                // 超出轨迹的终点的话就返回unprotected的状态
}

bool ReferenceLineInfo::CheckChangeLane() const {                                   // 变道是否是安全的
  if (!IsChangeLanePath()) {                                                        // 是否有可以变道的path
    ADEBUG << "Not a change lane path.";                                            // 检查是否有可以变道的车道(path)
    return false;
  }

  for (const auto* path_obstacle : path_decision_.path_obstacles().Items()) {       // 会迭代path决策里面的所有path上的障碍物
    const auto& sl_boundary = path_obstacle->PerceptionSLBoundary();                // 每个障碍物会设置感知到的sl坐标下的边框, 和st坐标下的边框

    constexpr float kLateralShift = 2.5;                                            // 横向的移动距离常数为2.5米
    if (sl_boundary.start_l() < -kLateralShift ||                                   // 如果sl的边框boundary离中心参考线太远了就不考虑这个障碍物
        sl_boundary.end_l() > kLateralShift) {
      continue;
    }

    constexpr float kSafeTime = 3.0;                                                // 安全时间为3秒
    constexpr float kForwardMinSafeDistance = 6.0;                                  // 前向的安全距离为6米
    constexpr float kBackwardMinSafeDistance = 8.0;                                 // 后向的安全距离为8米

    const float kForwardSafeDistance =                                              // 最终的前向安全距离设置为3冗余时间和6米的最大值
        std::max(kForwardMinSafeDistance,
                 static_cast<float>((adc_planning_point_.v() -
                                     path_obstacle->obstacle()->Speed()) *
                                    kSafeTime));
    const float kBackwardSafeDistance =                                             // 后向安全距离设置为3秒的相对速度和8米的最大值
        std::max(kBackwardMinSafeDistance,
                 static_cast<float>((path_obstacle->obstacle()->Speed() -
                                     adc_planning_point_.v()) *
                                    kSafeTime));
    if (sl_boundary.end_s() > sl_boundary_info_.adc_sl_boundary_.start_s() -        // 如果最终障碍物的sl边框堵在我自动驾驶车辆的安全区域类的话, 就不能变道
                                  kBackwardSafeDistance &&
        sl_boundary.start_s() <
            sl_boundary_info_.adc_sl_boundary_.end_s() + kForwardSafeDistance) {
      return false;
    }
  }
  return true;                                                                      // 如果前方一切畅通无阻, 那么就变道啥
}

const hdmap::RouteSegments& ReferenceLineInfo::Lanes() const { return lanes_; }     // lane直接返回内部的lanes_成员对象

const std::list<hdmap::Id> ReferenceLineInfo::TargetLaneId() const {                // 获取
  std::list<hdmap::Id> lane_ids;                                                    // 用一个双链表来保存车道的id索引值
  for (const auto& lane_seg : lanes_) {                                             // 迭代私有的成员变量
    lane_ids.push_back(lane_seg.lane->id());                                        // 把lane_中的id值全部取出来, 然后返回作为输出
  }
  return lane_ids;
}

const SLBoundary& ReferenceLineInfo::AdcSlBoundary() const {                        // 获取自动驾驶车辆的boundary边框
  return sl_boundary_info_.adc_sl_boundary_;                                        // 从sl边框中获取, sl_boundary_info_封装了adc的边框和vechile的边框
}

const SLBoundary& ReferenceLineInfo::VehicleSlBoundary() const {                    // 获取vechile的边框
  return sl_boundary_info_.vehicle_sl_boundary_;
}

PathDecision* ReferenceLineInfo::path_decision() { return &path_decision_; }        // 返回path上的决策

const PathDecision& ReferenceLineInfo::path_decision() const {                      // 只读的path决策
  return path_decision_;
}

const ReferenceLine& ReferenceLineInfo::reference_line() const {                    // 返回车道中心参考线的信息
  return reference_line_;
}

void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {    // 返回离散的轨迹
  discretized_trajectory_ = trajectory;
}

bool ReferenceLineInfo::AddObstacleHelper(const Obstacle* obstacle) {               // 向障碍物的数组中增加一个新的障碍物
  return AddObstacle(obstacle) != nullptr;
}

// AddObstacle is thread safe
PathObstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {            // 线程安全的增加障碍物
  if (!obstacle) {                                                                  // 错误检查
    AERROR << "The provided obstacle is empty";                                     // 障碍物的指针为空
    return nullptr;
  }
  auto* path_obstacle = path_decision_.AddPathObstacle(PathObstacle(obstacle));     // 通过道路的决策添加一个障碍物
  if (!path_obstacle) {                                                             // 添加障碍物失败了
    AERROR << "failed to add obstacle " << obstacle->Id();
    return nullptr;
  }

  SLBoundary perception_sl;                                                         // 感知到障碍物的sl边框
  if (!reference_line_.GetSLBoundary(obstacle->PerceptionBoundingBox(),             // boundingbox是什么意思?边界框
                                     &perception_sl)) {
    AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();         // 感知获取边界框出错了
    return path_obstacle;
  }
  path_obstacle->SetPerceptionSlBoundary(perception_sl);                            // 从中心参考线中获得的边界框, 然后马上复制为path上的障碍物

  if (IsUnrelaventObstacle(path_obstacle)) {                                        // 是否是不相关的障碍物
    ObjectDecisionType ignore;                                                      // 一个物体的决策类型, 是否可以忽略
    ignore.mutable_ignore();
    path_decision_.AddLateralDecision("reference_line_filter", obstacle->Id(),      // 添加一个横向决策策略
                                      ignore);
    path_decision_.AddLongitudinalDecision("reference_line_filter",                 // 添加一个纵向决策策略
                                           obstacle->Id(), ignore);                 // 将障碍物忽略
    ADEBUG << "NO build reference line st boundary. id:" << obstacle->Id();
  } else {                                                                          // 不可忽略的障碍物
    ADEBUG << "build reference line st boundary. id:" << obstacle->Id();            // debug出这个障碍物的id值
    path_obstacle->BuildReferenceLineStBoundary(                                    // 建立障碍物在中心参考线中的st坐标系下的边框boundary
        reference_line_, sl_boundary_info_.adc_sl_boundary_.start_s());

    ADEBUG << "reference line st boundary: "                                        // debug出中心参考线boundary的相关信息
           << path_obstacle->reference_line_st_boundary().min_t() << ", "
           << path_obstacle->reference_line_st_boundary().max_t()
           << ", s_max: " << path_obstacle->reference_line_st_boundary().max_s()
           << ", s_min: "
           << path_obstacle->reference_line_st_boundary().min_s();
  }
  return path_obstacle;                                                             // 返回path上的障碍物
}

bool ReferenceLineInfo::AddObstacles(                                               // 要一次添加很多个障碍物, 咋办?　那就封装一层AddObstacle就好了
    const std::vector<const Obstacle*>& obstacles) {　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　// 存放obstacle指针的数组
  if (FLAGS_use_multi_thread_to_add_obstacles) {                                    // 利用多线程来添加障碍物
    // std::future可用于异步任务中获取任务结果，但是它只是获取结果而已，真正的异步调用需要配合std::async, std::promise, std::packaged_task
    std::vector<std::future<bool>> futures;                                         // 一种同步机制
    for (const auto* obstacle : obstacles) {                                        // 迭代整个数组
      futures.push_back(ThreadPool::pool()->push(
          std::bind(&ReferenceLineInfo::AddObstacleHelper, this, obstacle)));       // 通过AddObstacleHelper函数进行添加障碍物
    }

    for (const auto& f : futures) {                                                 // 等待进行同步
      f.wait();
    }

    for (auto& f : futures) {
      if (!f.get()) {
        return false;
      }
    }
  } else {                                                                          // 单线程进行添加障碍物
    for (const auto* obstacle : obstacles) {
      if (!AddObstacle(obstacle)) {
        AERROR << "Failed to add obstacle " << obstacle->Id();
        return false;
      }
    }
  }

  return true;
}

bool ReferenceLineInfo::IsUnrelaventObstacle(PathObstacle* path_obstacle) {       // 判断一个障碍物是否是一个不相关的障碍物
  // if adc is on the road, and obstacle behind adc, ignore                       // 如果自动驾驶车辆在道路上, 就会忽略adc背后的障碍物
  if (path_obstacle->PerceptionSLBoundary().end_s() >                             // 就是通过障碍物的边框长来决定的
      reference_line_.Length()) {                                                 // 超出了参考线的位置
    return true;                                                                  // 确定为是可以忽略的障碍物
  }
  if (is_on_reference_line_ &&                                                    // 虽然在车道上, 但是在自动驾驶车辆的后面
      path_obstacle->PerceptionSLBoundary().end_s() <
          sl_boundary_info_.adc_sl_boundary_.end_s() &&
      reference_line_.IsOnLane(path_obstacle->PerceptionSLBoundary())) {
    return true;                                                                  // 也是不相干的障碍物
  }
  return false;                                                                   // 需要考虑的障碍物
}

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {              // 返回离散的轨迹点
  return discretized_trajectory_;
}

double ReferenceLineInfo::TrajectoryLength() const {                              // 返回轨迹的长度
  const auto& tps = discretized_trajectory_.trajectory_points();                  // 轨迹点
  if (tps.empty()) {                                                              // 如果是空的话就返回0
    return 0.0;
  }
  return tps.back().path_point().s();                                             // 否则返回最后的一个点的s(积累的距离)
}

void ReferenceLineInfo::SetStopPoint(const StopPoint& stop_point) {               // 通过给点的一点, 设置参考路径的终点
  planning_target_.mutable_stop_point()->CopyFrom(stop_point);
}

void ReferenceLineInfo::SetCruiseSpeed(double speed) {                            // 设置巡航的速度
  planning_target_.set_cruise_speed(speed);
}

bool ReferenceLineInfo::IsStartFrom(
    const ReferenceLineInfo& previous_reference_line_info) const {                // 是否从上一个参考线的地方开始执行
  if (reference_line_.reference_points().empty()) {                               // 当前的参考线的点的个数为0
    return false;                                                                 // 则返回false
  }
  auto start_point = reference_line_.reference_points().front();                  // 开始点就是数组中的第一个点
  const auto& prev_reference_line =                                               // 获取上一条车道中心参考线
      previous_reference_line_info.reference_line();
  common::SLPoint sl_point;                                                       // 将起点转换为sl坐标下的点
  prev_reference_line.XYToSL(start_point, &sl_point);                             // 调用reference line中的坐标转换函数
  return previous_reference_line_info.reference_line_.IsOnLane(sl_point);         // 判断当前中心参考线的起点是否在另一条参考线上
}

const PathData& ReferenceLineInfo::path_data() const { return path_data_; }       // 返回path相关的数据

const SpeedData& ReferenceLineInfo::speed_data() const { return speed_data_; }    // 返回speed相关的数据

PathData* ReferenceLineInfo::mutable_path_data() { return &path_data_; }          // 返回可修改的数据

SpeedData* ReferenceLineInfo::mutable_speed_data() { return &speed_data_; }       // 返回可次改的数据

bool ReferenceLineInfo::CombinePathAndSpeedProfile(                               // 合并path和speed的配置
    const double relative_time, const double start_s,                             // 相对时间(时间戳), 起点(start_s)---在s方向上
    DiscretizedTrajectory* ptr_discretized_trajectory) {                          // 输出是一个离散的轨迹信息, 通过一个指针指向
  CHECK(ptr_discretized_trajectory != nullptr);                                   // 容错检查
  // use varied resolution to reduce data load but also provide enough data
  // point for control module                                                     // 使用不同的分辨率来减少数据负载，但也为控制模块提供足够的数据点
  const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;         // 轨迹点的最小时间间隔为0.02秒
  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;        // 最大时间间隔为0.1秒
  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;         // 保持高密度的时间为1秒
  if (path_data_.discretized_path().NumOfPoints() == 0) {                         // 离散path的个数
    AWARN << "path data is empty";                                                // 如果个数为零, 则个数为空
    return false;
  }
  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();         // 根据总时间选择是利用0.02的最小单位还是0.1的最小单位
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                     : kSparseTimeResolution)) {
    common::SpeedPoint speed_point;                                               // 速度点, 用于保存线性插值的速度(根据当前的时间点)
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data_.discretized_path().Length()) {               // 如果速度点的s超出了离散路径的长度的话, 就直接退出
      break;
    }
    common::PathPoint path_point;                                                 // path点, 通过速度点上的s, 线性插值一个path上的一个点
    if (!path_data_.GetPathPointWithPathS(speed_point.s(), &path_point)) {
      AERROR << "Fail to get path data with s " << speed_point.s()                // 容错处理
             << "path total length " << path_data_.discretized_path().Length();
      return false;
    }
    path_point.set_s(path_point.s() + start_s);                                   // 要加上起始点

    common::TrajectoryPoint trajectory_point;                                     // 最终会把路程(path)和速度(speed)相关的信息融合到轨迹点中
    trajectory_point.mutable_path_point()->CopyFrom(path_point);                  // 设置轨迹点中的path数据
    trajectory_point.set_v(speed_point.v());                                      // 设置轨迹点中的速度
    trajectory_point.set_a(speed_point.a());                                      // 设置轨迹点中的加速度
    trajectory_point.set_relative_time(speed_point.t() + relative_time);          // 是设置轨迹点的相对时间, 还要加上一个relative_time
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);          // 将这个轨迹点追加到
  }
  return true;
}

void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }   // 设置是否能够开车过去

bool ReferenceLineInfo::IsDrivable() const { return is_drivable_; }               // 返回是否能够开车通过

bool ReferenceLineInfo::IsChangeLanePath() const {                                // 判断是否有变道的路径
  return !Lanes().IsOnSegment();                                                  // 就是看lanes是否在segment上
}

bool ReferenceLineInfo::IsNeighborLanePath() const {
  return Lanes().IsNeighborSegment();                                             // 是否有相邻的lane path
}

std::string ReferenceLineInfo::PathSpeedDebugString() const {                     // debug path的数据信息和speed的数据信息
  return apollo::common::util::StrCat("path_data:", path_data_.DebugString(),
                                      "speed_data:", speed_data_.DebugString());
}

void ReferenceLineInfo::ExportTurnSignal(VehicleSignal* signal) const {           // 导入车辆的信号信息
  // set vehicle change lane signal                                               // 设置车辆变换车道信号
  CHECK_NOTNULL(signal);                                                          // 容错检查, signal这个指针不能为空

  signal->Clear();                                                                // 先把信号灯清空
  signal->set_turn_signal(VehicleSignal::TURN_NONE);                              // 先设置没有转向信息
  if (IsChangeLanePath()) {                                                       // 是否有可以变道的lane
    if (Lanes().PreviousAction() == routing::ChangeLaneType::LEFT) {              // 上个动作是左转的话
      signal->set_turn_signal(VehicleSignal::TURN_LEFT);                          // 那么下一个动作就为左转
    } else if (Lanes().PreviousAction() == routing::ChangeLaneType::RIGHT) {      // 右转也一样, 不能让转向变为突变的类型
      signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
    }
    return;
  }
  // check lane's turn type
  double route_s = 0.0;                                                           // route基类的距离(s)
  const double adc_s = sl_boundary_info_.adc_sl_boundary_.end_s();                // 自动驾驶车辆sl坐标坐标中结束点的s(路程)
  for (const auto& seg : Lanes()) {                                               // 迭代所有路径的每个片段
    if (route_s > adc_s + FLAGS_turn_signal_distance) {                           // according to DMV's rule, turn signal should be on within 200 ft from intersection.
      break;                                                                      // 根据DMV的规则，转弯信号应在距交叉口200英尺的范围内。
    }
    route_s += seg.end_s - seg.start_s;                                           // 叠加每个片段的距离
    if (route_s < adc_s) {                                                        // 直到route到车辆的前方
      continue;
    }
    const auto& turn = seg.lane->lane().turn();                                   // 获取到专线的信息
    if (turn == hdmap::Lane::LEFT_TURN) {                                         // 如果为左转, 那就设置为左转
      signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      break;
    } else if (turn == hdmap::Lane::RIGHT_TURN) {                                 // 如果为右转, 那就设置为右转
      signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      break;
    } else if (turn == hdmap::Lane::U_TURN) {                                     //如果是掉头的话就会要求动作多一些
      // check left or right by geometry.                                         // 检查左右两边的集合点
      auto start_xy =                                                             // 从起点处获得一个平滑的xy作弊中的一个点
          common::util::MakeVec2d(seg.lane->GetSmoothPoint(seg.start_s));
      auto middle_xy = common::util::MakeVec2d(                                   // 获取一个中点
          seg.lane->GetSmoothPoint((seg.start_s + seg.end_s) / 2.0));
      auto end_xy =
          common::util::MakeVec2d(seg.lane->GetSmoothPoint(seg.end_s));           // 获取中点对应到lane中的点
      auto start_to_middle = middle_xy - start_xy;                                // 起点到中点的距离
      auto start_to_end = end_xy - start_xy;                                      // 中点到终点的距离
      if (start_to_middle.CrossProd(start_to_end) < 0) {                          // 根据两个点的叉积进行判断左转还是右转, 因为逆时针为正, 所以所以右转的话, 叉积要小于0
        signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      } else {
        signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      }
      break;
    }
  }
}

bool ReferenceLineInfo::IsRightTurnPath() const {                                 // 在path上是否进行右转
  double route_s = 0.0;                                                           // route的s(距离)
  const double adc_s = sl_boundary_info_.adc_sl_boundary_.end_s();                // 自动驾驶车辆sl坐标系中的结束点
  constexpr double kRightTurnStartBuff = 1.0;                                     // 给右转添加一个buff
  for (const auto& seg : Lanes()) {                                               // 迭代每个lane segment(车道段)
    if (route_s > adc_s + kRightTurnStartBuff) {
      break;
    }
    route_s += seg.end_s - seg.start_s;                                           // 叠加route的s方向的路程
    if (route_s < adc_s) {
      continue;
    }
    const auto& turn = seg.lane->lane().turn();                                   // 是否是在右转
    if (turn == hdmap::Lane::RIGHT_TURN) {                                        // 是右转的话就返回true, 不是右转的话就返回false
      return true;
    }
  }
  return false;
}

bool ReferenceLineInfo::ReachedDestination() const {                              // 是否到达了终点
  constexpr double kDestinationDeltaS = 0.05;                                     // 到终点的最小单位(s方向上的最小单位)     
  const auto* dest_ptr = path_decision_.Find(FLAGS_destination_obstacle_id);      // 终点障碍物的id号为"DEST", 将终点转换为一个障碍物
  if (!dest_ptr) {                                                                // dest_ptr是一个空指针
    return false;                                                                 // 就返回一个false
  }
  if (!dest_ptr->LongitudinalDecision().has_stop()) {                             // 纵向的决策是否已经停止了
    return false;
  }
  if (!reference_line_.IsOnLane(                                                  // 终点抽象出来的障碍物的中心点是否是在车道上
          dest_ptr->obstacle()->PerceptionBoundingBox().center())) {              // 如果不在证明没有到终点
    return false;
  }
  const double stop_s = dest_ptr->PerceptionSLBoundary().start_s() +              //停止的距离点(s), 是感知到的sl边框下的起点加上纵向决策的距离点s
                        dest_ptr->LongitudinalDecision().stop().distance_s();
  return sl_boundary_info_.adc_sl_boundary_.end_s() + kDestinationDeltaS >        // 终点障碍物的终点应该比自动驾驶车辆的终点稍稍大一点点
         stop_s;
}

void ReferenceLineInfo::ExportDecision(DecisionResult* decision_result) const {   // 导入决策的结果
  MakeDecision(decision_result);                                                  // 通过输入的指针构造一个决策, 主要的决策(自动驾驶车辆的决策), 对象的决策(障碍物的决策)
  ExportTurnSignal(decision_result->mutable_vehicle_signal());                    // 导入转向信号的信息
  auto* main_decision = decision_result->mutable_main_decision();                 // 获取自动驾驶车辆的决策信息
  if (main_decision->has_stop()) {                                                // 如果有停车的决策的话
    main_decision->mutable_stop()->set_change_lane_type(                          // 那就修改为之前的动作??还是说将前一个动作改为停止
        Lanes().PreviousAction());
  } else if (main_decision->has_cruise()) {                                       // 将上一个动作改为巡航
    main_decision->mutable_cruise()->set_change_lane_type(
        Lanes().PreviousAction());
  }
}

void ReferenceLineInfo::MakeDecision(DecisionResult* decision_result) const {     // 执行一个决策??还是生成一个决策
  CHECK_NOTNULL(decision_result);                                                 // 检查指针是否是空指针
  decision_result->Clear();                                                       // 清除决策的所有值

  // cruise by default
  decision_result->mutable_main_decision()->mutable_cruise();                     // 默认的主要决策是巡航

  // check stop decision
  int error_code = MakeMainStopDecision(decision_result);                         // 迭代所有障碍物, 看看有没有必要停车
  if (error_code < 0) {                                                           // 如果错误码小于0的话, 就会紧急停车           
    MakeEStopDecision(decision_result);                                           // 紧急停止
  }
  MakeMainMissionCompleteDecision(decision_result);                               // 主要任务已经完成
  SetObjectDecisions(decision_result->mutable_object_decision());                 // 设置对象障碍物的决策
}

void ReferenceLineInfo::MakeMainMissionCompleteDecision(                          // 主要任务已经完成
    DecisionResult* decision_result) const {                                      // 
  if (!decision_result->main_decision().has_stop()) {                             // 如果主要的任务中都没有停止点, 肯定是不行的
    return;                                                                       // 直接返回
  }
  auto main_stop = decision_result->main_decision().stop();                       // 主要停止点
  if (main_stop.reason_code() != STOP_REASON_DESTINATION) {                       // 停止不是因为到达终点的话, 也会直接返回
    return;
  }
  const auto& adc_pos = adc_planning_point_.path_point();                         // 获得自动驾驶车辆的位置
  if (common::util::DistanceXY(adc_pos, main_stop.stop_point()) >                 // 检查自动驾驶车辆到终点的距离
      FLAGS_destination_check_distance) {                                         // 在5米之内就认为已经快达到终点了, FLAGS_destination_check_distance被设置为5
    return;
  }

  auto mission_complete =
      decision_result->mutable_main_decision()->mutable_mission_complete();       // 看看主要的任务是否已经完成
  if (ReachedDestination()) {                                                     // 已经到达终点了的话, 就把对应的标志为就设置为true
    GetPlanningStatus()->mutable_destination()->set_has_passed_destination(
        true);
  } else {                                                                        // 没有到达终点的话
    mission_complete->mutable_stop_point()->CopyFrom(main_stop.stop_point());     // 就设置停止点
    mission_complete->set_stop_heading(main_stop.stop_heading());                 // 然后设置停止的航向角
  }
}

int ReferenceLineInfo::MakeMainStopDecision(                                          // 设置主要的停止决策
    DecisionResult* decision_result) const {
  double min_stop_line_s = std::numeric_limits<double>::infinity();                   // 最短的停止距离s的大小, 初始化为无穷大
  const Obstacle* stop_obstacle = nullptr;                                            // 停止的障碍物
  const ObjectStop* stop_decision = nullptr;                                          // 停止的决策

  for (const auto path_obstacle : path_decision_.path_obstacles().Items()) {          // 迭代路径上的每一个障碍物
    const auto& obstacle = path_obstacle->obstacle();                                 // 获取障碍物的引用
    const auto& object_decision = path_obstacle->LongitudinalDecision();              // 获取纵向的决策
    if (!object_decision.has_stop()) {                                                // 该障碍物没有停车的选项
      continue;                                                                       // 继续迭代          
    }

    apollo::common::PointENU stop_point = object_decision.stop().stop_point();        // 设置障碍物的停止点(UTM坐标系)
    common::SLPoint stop_line_sl;
    reference_line_.XYToSL({stop_point.x(), stop_point.y()}, &stop_line_sl);          // 将停止点的坐标转换为sl坐标

    double stop_line_s = stop_line_sl.s();                                            // 停止点在s轴上的值            
    if (stop_line_s < 0 || stop_line_s > reference_line_.Length()) {                  // 如果停止点是一个负数或者比车道参考中心线还长的话, 这个肯定有错的
      AERROR << "Ignore object:" << obstacle->Id() << " fence route_s["
             << stop_line_s << "] not in range[0, " << reference_line_.Length()
             << "]";
      continue;                                                                       // 那就跳过这个障碍物
    }

    // check stop_line_s vs adc_s
    if (stop_line_s < min_stop_line_s) {                                              // 比较停止线的s大小和自动驾驶车辆s的大小
      min_stop_line_s = stop_line_s;                                                  // 获取所有障碍物中最小的停车点
      stop_obstacle = obstacle;
      stop_decision = &(object_decision.stop());
    }
  }

  if (stop_obstacle != nullptr) {                                                     // 如果停止的障碍物不为空
    MainStop* main_stop =
        decision_result->mutable_main_decision()->mutable_stop();                     // 将主要的决策设置为停止
    main_stop->set_reason_code(stop_decision->reason_code());                         // 设置停止的原因
    main_stop->set_reason("stop by " + stop_obstacle->Id());
    main_stop->mutable_stop_point()->set_x(stop_decision->stop_point().x());          // 设置停止的坐标
    main_stop->mutable_stop_point()->set_y(stop_decision->stop_point().y());
    main_stop->set_stop_heading(stop_decision->stop_heading());                       // 设置停止的航向角                                 

    ADEBUG << " main stop obstacle id:" << stop_obstacle->Id()                        // debug出相应的string信息
           << " stop_line_s:" << min_stop_line_s << " stop_point: ("
           << stop_decision->stop_point().x() << stop_decision->stop_point().y()
           << " ) stop_heading: " << stop_decision->stop_heading();

    return 1;
  }

  return 0;
}

void ReferenceLineInfo::SetObjectDecisions(                                           // 设置对象障碍物的决策
    ObjectDecisions* object_decisions) const {                                        // 输入的是对象的决策
  for (const auto path_obstacle : path_decision_.path_obstacles().Items()) {          // 迭代道路上的所有障碍物
    if (!path_obstacle->HasNonIgnoreDecision()) {                                     // 如果有不能忽略的决策, 就继续执行
      continue;
    }
    auto* object_decision = object_decisions->add_decision();                         // 添加一个新的决策

    const auto& obstacle = path_obstacle->obstacle();                                 // 获取path中的障碍物
    object_decision->set_id(obstacle->Id());                                          // 设置障碍物的ID值
    object_decision->set_perception_id(obstacle->PerceptionId());                     // 设置感知的ID值
    if (path_obstacle->HasLateralDecision() &&                                        // 如果横向的决策不能忽略的话
        !path_obstacle->IsLateralIgnore()) {
      object_decision->add_object_decision()->CopyFrom(                               // 拷贝障碍物的横向决策
          path_obstacle->LateralDecision());
    }
    if (path_obstacle->HasLongitudinalDecision() &&
        !path_obstacle->IsLongitudinalIgnore()) {
      object_decision->add_object_decision()->CopyFrom(                               // 添加纵向的决策    
          path_obstacle->LongitudinalDecision());
    }
  }
}

void ReferenceLineInfo::ExportEngageAdvice(EngageAdvice* engage_advice) const {       // 导入engage的建议
  constexpr double kMaxAngleDiff = M_PI / 6.0;                                        // 最大的角度偏差为六分之pi, 即30度
  auto* prev_advice = GetPlanningStatus()->mutable_engage_advice();                   // 获取上一次执行的engage的advice
  if (!prev_advice->has_advice()) {                                                   // 如果上一次没有建议的动作
    prev_advice->set_advice(EngageAdvice::DISALLOW_ENGAGE);                           // 就设置为不允许engage
  }
  if (!IsDrivable()) {                                                                // 如果不是可以行驶的车辆
    if (prev_advice->advice() == EngageAdvice::DISALLOW_ENGAGE) {
      prev_advice->set_advice(EngageAdvice::DISALLOW_ENGAGE);
    } else {
      prev_advice->set_advice(EngageAdvice::PREPARE_DISENGAGE);
    }
    prev_advice->set_reason("Reference line not drivable");
  } else if (!is_on_reference_line_) {                                                // 不在车道中心线上 
    if (prev_advice->advice() == EngageAdvice::DISALLOW_ENGAGE) {
      prev_advice->set_advice(EngageAdvice::DISALLOW_ENGAGE);
    } else {
      prev_advice->set_advice(EngageAdvice::PREPARE_DISENGAGE);
    }
    prev_advice->set_reason("Not on reference line");
  } else {
    // check heading
    auto ref_point = reference_line_.GetReferencePoint(                               // 获得自动驾驶车辆结束点投影到车道中心参考线上(reference line)
        sl_boundary_info_.adc_sl_boundary_.end_s());
    if (common::math::AngleDiff(vehicle_state_.heading(), ref_point.heading()) >      // 航向家偏差太大
        kMaxAngleDiff) {
      if (prev_advice->advice() == EngageAdvice::DISALLOW_ENGAGE) {
        prev_advice->set_advice(EngageAdvice::DISALLOW_ENGAGE);
      } else {
        prev_advice->set_advice(EngageAdvice::PREPARE_DISENGAGE);
      }
      prev_advice->set_reason("Vehicle heading is not aligned");                      // 航向角偏差不能超过30度
    } else {
      if (vehicle_state_.driving_mode() !=
          Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
        prev_advice->set_advice(EngageAdvice::READY_TO_ENGAGE);
      } else {
        prev_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
      }
      prev_advice->clear_reason();
    }
  }
  engage_advice->CopyFrom(*prev_advice);                                              // 拷贝之前的建议
}

void ReferenceLineInfo::MakeEStopDecision(                                            // 做紧急停车的处理
    DecisionResult* decision_result) const {                                          // 一旦调用这个函数, 就会清理到所有的决策信息
  decision_result->Clear();

  MainEmergencyStop* main_estop =                                                     // 获取紧急停车的状态
      decision_result->mutable_main_decision()->mutable_estop();
  main_estop->set_reason_code(MainEmergencyStop::ESTOP_REASON_INTERNAL_ERR);          // 记录下紧急停车的原因
  main_estop->set_reason("estop reason to be added");
  main_estop->mutable_cruise_to_stop();                                               // 然后从巡航马上停止

  // set object decisions
  ObjectDecisions* object_decisions =                                                 // 设置对象的决策
      decision_result->mutable_object_decision();
  for (const auto path_obstacle : path_decision_.path_obstacles().Items()) {          // 迭代path上所有的障碍物
    auto* object_decision = object_decisions->add_decision();                         // 添加一个对象的决策
    const auto& obstacle = path_obstacle->obstacle();                                 // 获取path上的障碍物索引
    object_decision->set_id(obstacle->Id());                                          // 设置障碍物的决策(先把对象的决策的ID设置为障碍物的ID)
    object_decision->set_perception_id(obstacle->PerceptionId());                     // 设置感知到的ID
    object_decision->add_object_decision()->mutable_avoid();                          // 所有障碍物都设置为自动驾驶车辆应该躲避的障碍物
  }
}
}  // namespace planning
}  // namespace apollo
