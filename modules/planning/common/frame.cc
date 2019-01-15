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
 * @file frame.cc
 **/
#include "modules/planning/common/frame.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <list>
#include <string>
#include <utility>

#include "modules/routing/proto/routing.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line_provider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;                    // 错误码
using apollo::common::Status;                       // 状态
using apollo::common::VehicleStateProvider;         // 为了更新车辆的状态, VehicleStateProvider是一个单例
using apollo::common::adapter::AdapterManager;      // 适配器管理者, 主要是为了接口兼容性
using apollo::common::math::Box2d;                  // 二维的矩形框
using apollo::common::math::Vec2d;                  // 二维的向量
using apollo::common::monitor::MonitorLogBuffer;    // 日志监控类
using apollo::prediction::PredictionObstacles;      // 感知障碍物

constexpr double kMathEpsilon = 1e-8;   // 无穷小

FrameHistory::FrameHistory()   // IndexedQueue 底层用vector和hash设计的
    : IndexedQueue<uint32_t, Frame>(FLAGS_max_history_frame_num) {}                    // FLAGS_max_history_frame_num设置为1, 最大的历史帧数为1

Frame::Frame(uint32_t sequence_num,                                                    // frame的构造函数. 第几帧
             const common::TrajectoryPoint &planning_start_point,                      // 轨迹的起点
             const double start_time, const common::VehicleState &vehicle_state,       // 开始的时间, 车辆的状态, 中心参考线的提供者
             ReferenceLineProvider *reference_line_provider)
    : sequence_num_(sequence_num),
      planning_start_point_(planning_start_point),
      start_time_(start_time),
      vehicle_state_(vehicle_state),
      reference_line_provider_(reference_line_provider),
      monitor_logger_(common::monitor::MonitorMessageItem::PLANNING) {                 // 日志内容默认是planning的信息
  if (FLAGS_enable_lag_prediction) {                                                   // FLAGS_enable_lag_prediction设置为true
    lag_predictor_.reset(     // 如果侧向预测, 那就新建一个LagPrediction对象
        new LagPrediction(FLAGS_lag_prediction_min_appear_num,                         // 新建一个延迟预测器, FLAGS_lag_prediction_min_appear_num设置为5
                          FLAGS_lag_prediction_max_disappear_num));                    // FLAGS_lag_prediction_max_disappear_num设置为3
  }
}

const common::TrajectoryPoint &Frame::PlanningStartPoint() const {                     // 返回做planning的起点
  return planning_start_point_;
}

const common::VehicleState &Frame::vehicle_state() const {                             // 返回车辆的状态
  return vehicle_state_;
}

bool Frame::Rerouting() {                                                              // 在frame类中重新做routing
  if (FLAGS_use_navigation_mode) {  // 导航模式不支持重新routing                          // 在config_gflags.cc文件中, FLAGS_use_navigation_mode设置为false
    AERROR << "Rerouting not supported in navigation mode";
    return false;
  }
  auto *adapter_manager = AdapterManager::instance();   // 在frame中创建单例
  if (adapter_manager->GetRoutingResponse()->Empty()) { // 获得routing的结果
    AERROR << "No previous routing available";
    return false;
  }
  if (!hdmap_) {                                        // 指向高精地图的指针
    AERROR << "Invalid HD Map.";
    return false;
  }
  auto request = adapter_manager->GetRoutingResponse()                                 // 获取routing的response(反应值)
                     ->GetLatestObserved()              // 最近的障碍物
                     .routing_request();                // 返回最新的消息
  request.clear_header();                               // 得到的请求, 清除header
  AdapterManager::FillRoutingRequestHeader("planning", &request);
  auto point = common::util::MakePointENU(
      vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z());   // UTM 地图中的一点
  double s = 0.0;
  double l = 0.0;
  hdmap::LaneInfoConstPtr lane;        // lane就是车行道
  // 根据车辆状态和位置，从hdmap中找到最近的车道：GetNearestLaneWithHeading
  if (hdmap_->GetNearestLaneWithHeading(point, 5.0, vehicle_state_.heading(),         // 获取最近的lane
                                        M_PI / 3.0, &lane, &s, &l) != 0) {
    AERROR << "Failed to find nearest lane from map at position: "
           << point.DebugString() << ", heading:" << vehicle_state_.heading();
    return false;                                                                     // 不需要重新routing
  }
  request.clear_waypoint();                                                           // 再将request的结果清除掉
  auto *start_point = request.add_waypoint();    // 设置起点                           // 请求的起点
  start_point->set_id(lane->id().id());                                               // 设置起点的lane的id地址
  start_point->set_s(s);                                                              // 设置起点的s(起点)
  start_point->mutable_pose()->CopyFrom(point);                                       // 起点的位置
  for (const auto &waypoint :
       reference_line_provider_->FutureRouteWaypoints()) {                            // 线程安全地获取路网点
    request.add_waypoint()->CopyFrom(waypoint);                                       // 一个一个的点加到请求的路网中
  }
  if (request.waypoint_size() <= 1) {                                                 // 容错处理
    AERROR << "Failed to find future waypoints";
    return false;
  }                                                                                   // 将请求到的数据计算更新后再发布出去
  // 根据当前的车道等信息，发送新的RoutingRequest: PublishRoutingRequest
  AdapterManager::PublishRoutingRequest(request);     // 发布ronting的请求
  apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("Planning send Rerouting request");   
  return true;
}

std::list<ReferenceLineInfo> &Frame::reference_line_info() {                          // 返回一帧数据中中心参考线的信息
  return reference_line_info_;
}
// 更新参考线的优先级
void Frame::UpdateReferenceLinePriority(
    const std::map<std::string, uint32_t> &id_to_priority) {                          // 将id地址(reference line的名字)和对应的优先级通过一个红黑树进行映射
  for (const auto &pair : id_to_priority) {                                           // 迭代红黑树中的每个结点
    const auto id = pair.first;
    const auto priority = pair.second;
    auto ref_line_info_itr =
        std::find_if(reference_line_info_.begin(), reference_line_info_.end(),
                     [&id](const ReferenceLineInfo &ref_line_info) {
                       return ref_line_info.Lanes().Id() == id;                       // 找到id地址相等的那个点
                     });
    if (ref_line_info_itr != reference_line_info_.end()) {                            // 如果优先级没有
      ref_line_info_itr->SetPriority(priority);                                       // 设置参考线的新的优先级
    }
  }
}
// 根据所有的参考线, 高精地图的segments, 创建ReferenceLineInfo的值
bool Frame::CreateReferenceLineInfo() {
  // 从 reference_line_provider_ 拿到所有的lines 和 segments
  std::list<ReferenceLine> reference_lines;                                           // 中心参考线的双链表
  std::list<hdmap::RouteSegments> segments;                                           // route segment的信息
  // 创建Line，检测segments和line相等                                                    从中心参考线的provider中获得中心参考线和segments
  if (!reference_line_provider_->GetReferenceLines(&reference_lines,                  // 拿到所有的参考线
                                                   &segments)) {                      // 所有的segments
    AERROR << "Failed to create reference line";
    return false;
  }
  DCHECK_EQ(reference_lines.size(), segments.size());                                 // 检查参考线reference lines和segments的大小是否相等

  auto forword_limit =
      hdmap::PncMap::LookForwardDistance(vehicle_state_.linear_velocity());           // 前向的安全距离
  // Shrink是什么意思?
  // Shrink ref_line 和 segment：裁剪掉没用的，并生成新的refline
  for (auto &ref_line : reference_lines) {                                            // 迭代每一条reference line
    if (!ref_line.Shrink(Vec2d(vehicle_state_.x(), vehicle_state_.y()),               // 每条参考中心线, 必须shink一下, 删掉不必要的点
                         FLAGS_look_backward_distance, forword_limit)) {
      AERROR << "Fail to shrink reference line.";
      return false;
    }
  }
  for (auto &seg : segments) {                                                        // 除了reference line需要shrink, route segment也需要进行shink
    if (!seg.Shrink(Vec2d(vehicle_state_.x(), vehicle_state_.y()),
                    FLAGS_look_backward_distance, forword_limit)) {
      AERROR << "Fail to shrink routing segments.";
      return false;
    }
  }

  reference_line_info_.clear();                                                       // reference line的info需要重新清理掉, 重新赋值
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  // 多条线的话，计算不同线的距离
  while (ref_line_iter != reference_lines.end()) {                                    // 迭代所有的参考线
    if (segments_iter->StopForDestination()) {     // 到达终点                         检查是否到了终点的route segments(片段), routing 是一截一截的
      is_near_destination_ = true;
    }
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,          // 构造ReferenceLineInfo的信息, 添加到双链表中
                                      *ref_line_iter, *segments_iter);
    ++ref_line_iter;
    ++segments_iter;
  }
  // 计算换道逻辑，进行Init
  if (FLAGS_enable_change_lane_decider &&                                             // 默认是不进行变道的
      !change_lane_decider_.Apply(&reference_line_info_)) {    // 是否变道
    AERROR << "Failed to apply change lane decider";
    return false;
  }

  if (reference_line_info_.size() == 2) {                                             // 就只有两条中心参考线
    common::math::Vec2d xy_point(vehicle_state_.x(), vehicle_state_.y());             // 向量
    common::SLPoint first_sl;                                                         // 缓存到sl坐标系
    if (!reference_line_info_.front().reference_line().XYToSL(xy_point,
                                                              &first_sl)) {
      return false;
    }
    common::SLPoint second_sl;                                                        // 将车的位置投影到这两条reference line(sl坐标系下)中
    if (!reference_line_info_.back().reference_line().XYToSL(xy_point,
                                                             &second_sl)) {
      return false;
    }
    const double offset = first_sl.l() - second_sl.l();                               // 两个参考点的偏差值
    reference_line_info_.front().SetOffsetToOtherReferenceLine(offset);               // 两条中心参考线之间的偏差值
    reference_line_info_.back().SetOffsetToOtherReferenceLine(-offset);
  }
  // Init函数：ref_info.Init的主要逻辑
  bool has_valid_reference_line = false;                                              // 是否有合法的中心参考线
  for (auto &ref_info : reference_line_info_) {                                       // 迭代双链表(里面的元素是ReferenceLineInfo)
    if (!ref_info.Init(obstacles())) {                                                // 通过障碍物进行初始化ReferenceLineInfo对象
      AERROR << "Failed to init reference line";
      continue;
    } else {
      has_valid_reference_line = true;                                                // 初始化成功, 至少有一条成功的ReferenceLineInfo
    }
  }
  return has_valid_reference_line;
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
const Obstacle *Frame::CreateStopObstacle(                                            // 创建一个停止的障碍物
    ReferenceLineInfo *const reference_line_info,                                     // 输入一个中心参考线
    const std::string &obstacle_id, const double obstacle_s) {                        // 障碍物的id号, 障碍物的s
  if (reference_line_info == nullptr) {                                               // 容错检查
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();                 // 获取中心参考线
  const double box_center_s = obstacle_s + FLAGS_virtual_stop_wall_length / 2.0;      // 虚拟停止墙的长度为0.1米
  auto box_center = reference_line.GetReferencePoint(box_center_s);                   // 通过中心点构造一个box
  double heading = reference_line.GetReferencePoint(obstacle_s).heading();            // 获取航向角
  double lane_left_width = 0.0;                                                       // 车道左边的宽度
  double lane_right_width = 0.0;                                                      // 到车道右边的宽度
  reference_line.GetLaneWidth(obstacle_s, &lane_left_width, &lane_right_width);
  Box2d stop_wall_box{box_center, heading, FLAGS_virtual_stop_wall_length,            // 构造一个停止墙的box
                      lane_left_width + lane_right_width};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);                     // 然后把停止墙构造成虚拟的障碍物
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */                                                                                   // 通过障碍物的id, 车道的id, 车道的s创建一个停止的障碍物
const Obstacle *Frame::CreateStopObstacle(const std::string &obstacle_id,
                                          const std::string &lane_id,
                                          const double lane_s) {
  if (!hdmap_) {                                                                      // 高精地图是否存在
    AERROR << "Invalid HD Map.";
    return nullptr;
  }
  const auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));                   // 从高精地图中获取对应的lane(车道)
  if (!lane) {
    AERROR << "Failed to find lane[" << lane_id << "]";
    return nullptr;
  }

  double dest_lane_s = std::max(0.0, lane_s);                                         // 目的地
  auto dest_point = lane->GetSmoothPoint(dest_lane_s);                                // 将目的地的点做一次平滑处理???

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  lane->GetWidth(dest_lane_s, &lane_left_width, &lane_right_width);                   // 获取左右的宽度

  Box2d stop_wall_box{{dest_point.x(), dest_point.y()},                               // 构造一个停止墙
                      lane->Heading(dest_lane_s),
                      FLAGS_virtual_stop_wall_length,
                      lane_left_width + lane_right_width};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);                     // 创建一个静止的障碍物
}

/**
 * @brief: create static virtual object with lane width,
 */
const Obstacle *Frame::CreateStaticObstacle(                                          // 创建具有通道宽度的静态虚拟对象
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_start_s,
    const double obstacle_end_s) {
  if (reference_line_info == nullptr) {                                              // 容错检查
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();                // 从reference line的info信息中获取对应的中心参考线reference line

  // start_xy
  common::SLPoint sl_point;                                                          // 起点
  sl_point.set_s(obstacle_start_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_start_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_start_xy)) {                        // 将障碍物的起点转换为xy坐标系
    AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  // end_xy
  sl_point.set_s(obstacle_end_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_end_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_end_xy)) {                          // 获得障碍物的终点的xy坐标
    AERROR << "Failed to get end_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  if (!reference_line.GetLaneWidth(obstacle_start_s, &left_lane_width,               // 获得车道的宽度
                                   &right_lane_width)) {
    AERROR << "Failed to get lane width at s[" << obstacle_start_s << "]";
    return nullptr;
  }

  common::math::Box2d obstacle_box{
      common::math::LineSegment2d(obstacle_start_xy, obstacle_end_xy),               // 起点和终点构成一条线段
      left_lane_width + right_lane_width};                                           // 车道的宽度就是障碍物的宽度

  return CreateStaticVirtualObstacle(obstacle_id, obstacle_box);                     // 通过障碍物的id号, 和障碍物的box创建一个障碍物, 并返回其指针
}

const Obstacle *Frame::CreateStaticVirtualObstacle(const std::string &id,            // 创建静止的虚拟的障碍物
                                                   const Box2d &box) {
  const auto *object = obstacles_.Find(id);                                          // 找到对应的障碍物
  if (object) {    // 存在直接返回                                                     // 如果障碍物已经存在了就直接返回
    AWARN << "obstacle " << id << " already exist.";
    return object;
  }
  auto *ptr =                                                                        // 否者就新建一个障碍物
      obstacles_.Add(id, *Obstacle::CreateStaticVirtualObstacles(id, box));
  if (!ptr) {                                                                        // 创建成功的话就直接返回
    AERROR << "Failed to create virtual obstacle " << id;
  }
  return ptr;
}

Status Frame::Init() {                                                               // 一个frame框的初始化函数
  // 获取高精地图和车辆状态： 检车车辆位置是否合理
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();   // 获得地图                               // 先初始化高精度地图
  CHECK_NOTNULL(hdmap_);                                                             // 检查获取高精地图是否成功
  vehicle_state_ = common::VehicleStateProvider::instance()->vehicle_state();        // 在Frame中创建一个车辆状态的单例模式
  const auto &point = common::util::MakePointENU(
      vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z());
  if (std::isnan(point.x()) || std::isnan(point.y())) {
    AERROR << "init point is not set";
    return Status(ErrorCode::PLANNING_ERROR, "init point is not set");
  }
  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha
         << FLAGS_align_prediction_time;

  // 拿到prediction结果： 原始或者LaggedPrediction，删除历史点，提取出来其中的障碍物
  // prediction
  if (AdapterManager::GetPrediction() &&                                            // 获取预测值
      !AdapterManager::GetPrediction()->Empty()) {
    if (FLAGS_enable_lag_prediction && lag_predictor_) {                            // 获得滞后预测的结果
      lag_predictor_->GetLaggedPrediction(&prediction_);
    } else {
      prediction_.CopyFrom(
          AdapterManager::GetPrediction()->GetLatestObserved());
    }
    if (FLAGS_align_prediction_time) {                                              // 根据planning的时间启用对齐预测数据
      AlignPredictionTime(vehicle_state_.timestamp(), &prediction_);
    }
    for (auto &ptr : Obstacle::CreateObstacles(prediction_)) {                      // 通过预测的结果创建对应的障碍物
      AddObstacle(*ptr);
    }
  }
  // 检测是否碰撞
  if (FLAGS_enable_collision_detection) {   // 碰撞检测                              FLAGS_enable_collision_detection设置为false
    const auto *collision_obstacle = FindCollisionObstacle();                       // 找到碰撞的障碍物
    if (collision_obstacle) {                                                       // 容错处理
      std::string err_str =
          "Found collision with obstacle: " + collision_obstacle->Id();             // 报错
      apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);           // 写到log中
      buffer.ERROR(err_str);
      return Status(ErrorCode::PLANNING_ERROR, err_str);
    }
  }
  // 创建 reference_lines
  if (!CreateReferenceLineInfo()) {                                                 // 穿件参考线的信息(ReferenceLineInfo)
    AERROR << "Failed to init reference line info";
    return Status(ErrorCode::PLANNING_ERROR,
                  "failed to init reference line info");
  }

  return Status::OK();                                                              // 一切顺利的话就返回OK
}
// 看是否有碰撞的障碍物
const Obstacle *Frame::FindCollisionObstacle() const {                              // 发现碰撞的障碍物
  if (obstacles_.Items().empty()) {
    return nullptr;
  }
  const auto &param =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();         // 启动车辆配置帮助VehicleConfigHelper
  Vec2d position(vehicle_state_.x(), vehicle_state_.y());                           // 车的位置
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,           // 获得车辆的中心点
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);
  Vec2d center(position + vec_to_center.rotate(vehicle_state_.heading()));          // 加上方向的信息
  Box2d adc_box(center, vehicle_state_.heading(), param.length(),                   // 将车辆抽象成一个box
                param.width());
  const double adc_half_diagnal = adc_box.diagonal() / 2.0;                         // 对角线的一半
  for (const auto &obstacle : obstacles_.Items()) {                                 // obstacles_就是ThreadSafeIndexedList<std::string, Obstacle> 
    if (obstacle->IsVirtual()) {                                                    // 如果是虚拟障碍物的话, 是始终不会碰撞上
      continue;
    }

    double center_dist =                                                            // 自动驾驶车辆到中心点的距离
        adc_box.center().DistanceTo(obstacle->PerceptionBoundingBox().center());
    if (center_dist > obstacle->PerceptionBoundingBox().diagonal() / 2.0 +
                          adc_half_diagnal + FLAGS_max_collision_distance) {
      ADEBUG << "Obstacle : " << obstacle->Id() << " is too far to collide";
      continue;
    }
    double distance = obstacle->PerceptionPolygon().DistanceTo(adc_box);            // 感知到的多边形到自动驾驶车辆的距离
    if (FLAGS_ignore_overlapped_obstacle && distance < kMathEpsilon) {              // 是否忽略重叠的障碍物FLAGS_ignore_overlapped_obstacle设置为false
      bool all_points_in = true;                                                    // 所有的点都在碰撞
      for (const auto &point : obstacle->PerceptionPolygon().points()) {
        if (!adc_box.IsPointIn(point)) {
          all_points_in = false;                                                    // 不是所有的点都在碰撞
          break;
        }
      }
      if (all_points_in) {                                                          // 可能是雷达标定的错误
        ADEBUG << "Skip overlapped obstacle, which is often caused by lidar "
                  "calibration error";
        continue;
      }
    }           // FLAGS_max_collision_distance设置为0.1, 碰撞的距离为10厘米  
    if (distance < FLAGS_max_collision_distance) {                                  // 找到了碰撞的障碍物
      AERROR << "Found collision with obstacle " << obstacle->Id();
      return obstacle;
    }
  }
  return nullptr;
}

uint32_t Frame::SequenceNum() const { return sequence_num_; }                       // 返回第几帧数据

std::string Frame::DebugString() const {                                            // debug的字符串
  return "Frame: " + std::to_string(sequence_num_);
}

void Frame::RecordInputDebug(planning_internal::Debug *debug) {                     // 一帧数据, 记录输入的debug信息(主要是一个字符串)
  if (!debug) {
    ADEBUG << "Skip record input into debug";
    return;
  }
  auto *planning_data = debug->mutable_planning_data();                             // planning 的数据信心
  auto *adc_position = planning_data->mutable_adc_position();                       // 自动驾驶车辆的位置
  const auto &localization =
      AdapterManager::GetLocalization()->GetLatestObserved();                       // 最近观测到的位置
  adc_position->CopyFrom(localization);

  const auto &chassis = AdapterManager::GetChassis()->GetLatestObserved();          // 底盘的信息
  auto debug_chassis = planning_data->mutable_chassis();                            // 底盘的debug信息
  debug_chassis->CopyFrom(chassis);                                                 // 直接拷贝到debug信息中

  if (!FLAGS_use_navigation_mode) {                                                 // 如果使用的不是导航模式
    auto debug_routing = planning_data->mutable_routing();                          // 获得routing的数据
    debug_routing->CopyFrom(                                                        // 将routing的信息拷贝到debug routing中
        AdapterManager::GetRoutingResponse()->GetLatestObserved());
  }

  planning_data->mutable_prediction_header()->CopyFrom(prediction_.header());       // planning的头部数据

  auto relative_map = AdapterManager::GetRelativeMap();                             // 获得相对地图
  if (!relative_map->Empty()) {
    planning_data->mutable_relative_map()->mutable_header()->CopyFrom(              // 获得相对地图最新的头部信息
        relative_map->GetLatestObserved().header());
  }
}
// 对齐预测时间
void Frame::AlignPredictionTime(const double planning_start_time,                   // planning的开始时间
                                PredictionObstacles *prediction_obstacles) {        // 预测得到的障碍物
  if (!prediction_obstacles || !prediction_obstacles->has_header() ||
      !prediction_obstacles->header().has_timestamp_sec()) {
    return;
  }
  double prediction_header_time =
      prediction_obstacles->header().timestamp_sec();
  for (auto &obstacle : *prediction_obstacles->mutable_prediction_obstacle()) {   // 轮寻每个障碍物
    for (auto &trajectory : *obstacle.mutable_trajectory()) {                     // 每个轨迹
      for (auto &point : *trajectory.mutable_trajectory_point()) {                // 每个点
        point.set_relative_time(prediction_header_time + point.relative_time() -
                                planning_start_time);                             // 设置相对时间
      }
      if (!trajectory.trajectory_point().empty() &&
          trajectory.trajectory_point().begin()->relative_time() < 0) {
        auto it = trajectory.trajectory_point().begin();
        while (it != trajectory.trajectory_point().end() &&
               it->relative_time() < 0) {
          ++it;                                                                   // 跳过
        }
        trajectory.mutable_trajectory_point()->erase(                             // 移除不存在的点或是相对时间小于0的点
            trajectory.trajectory_point().begin(), it);
      }
    }
  }
}

Obstacle *Frame::Find(const std::string &id) { return obstacles_.Find(id); }     // 直接返回障碍物的指针

void Frame::AddObstacle(const Obstacle &obstacle) {
  obstacles_.Add(obstacle.Id(), obstacle);                                       // 添加一个障碍物
}
// dp的思想体现在这里? 找到cost 最小的line， 在std_planning.cc文件中进行调用
const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo() {
  double min_cost = std::numeric_limits<double>::infinity();
  drive_reference_line_info_ = nullptr;
  for (const auto &reference_line_info : reference_line_info_) {   // 迭代所有的reference_line
    if (reference_line_info.IsDrivable() &&                        // 是否是可行驶区域
        reference_line_info.Cost() < min_cost) {                   // 参考线的代价函数比min_cost还小
      drive_reference_line_info_ = &reference_line_info;           // 那就按最小的
      min_cost = reference_line_info.Cost();                       // 更新最小的代价
    }
  }
  return drive_reference_line_info_;                               // 返回可以正常驾驶的中心参考线
}

const ReferenceLineInfo *Frame::DriveReferenceLineInfo() const {   // 返回可以驾驶的中心参考线
  return drive_reference_line_info_;
}

const std::vector<const Obstacle *> Frame::obstacles() const {     // 返回障碍物数组
  return obstacles_.Items();
}

}  // namespace planning
}  // namespace apollo
