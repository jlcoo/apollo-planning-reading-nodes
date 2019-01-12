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

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/st_boundary.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;
using apollo::common::util::FindOrDie;
// 具有internal属性, 仅作用于当前文件
namespace {                                  // 编译器会在内部为匿名的命名空间生成一个唯一的名字, 还会为这个匿名的命名空间生成一条using指令
const double kStBoundaryDeltaS = 0.2;        // meters  , 采样点的距离?     
const double kStBoundarySparseDeltaS = 1.0;  // meters  稀疏的DeltaS
const double kStBoundaryDeltaT = 0.05;       // seconds 0.05秒一次
}  // namespace

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         PathObstacle::ObjectTagCaseHash>
    PathObstacle::s_longitudinal_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0},
        {ObjectDecisionType::kOvertake, 100},
        {ObjectDecisionType::kFollow, 300},
        {ObjectDecisionType::kYield, 400},
        {ObjectDecisionType::kStop, 500}};

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         PathObstacle::ObjectTagCaseHash>
    PathObstacle::s_lateral_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0},
        {ObjectDecisionType::kNudge, 100},
        {ObjectDecisionType::kSidepass, 200}};   // 旁边超过

const std::string& PathObstacle::Id() const { return id_; }

PathObstacle::PathObstacle(const Obstacle* obstacle) : obstacle_(obstacle) {            // 通过感知到的障碍物构造在道路上的障碍物
  CHECK_NOTNULL(obstacle);
  id_ = obstacle_->Id();
}

void PathObstacle::SetPerceptionSlBoundary(const SLBoundary& sl_boundary) {             // 设置感知到的sl的边框(boundary)
  perception_sl_boundary_ = sl_boundary;   // sl坐标系下的boundary
}

double PathObstacle::MinRadiusStopDistance(
    const common::VehicleParam& vehicle_param) const {                                  // 最小停止距离的半径
  if (min_radius_stop_distance_ > 0) {
    return min_radius_stop_distance_;
  }
  constexpr double stop_distance_buffer = 0.5;                                          // 停止距离为0.5米
  const double min_turn_radius = VehicleConfigHelper::MinSafeTurnRadius();
  double lateral_diff = vehicle_param.width() / 2.0 +                                   // 横向差异
                        std::max(std::fabs(perception_sl_boundary_.start_l()),
                                 std::fabs(perception_sl_boundary_.end_l()));
  const double kEpison = 1e-5;
  lateral_diff = std::min(lateral_diff, min_turn_radius - kEpison);
  double stop_distance =
      std::sqrt(std::fabs(min_turn_radius * min_turn_radius -
                          (min_turn_radius - lateral_diff) *
                              (min_turn_radius - lateral_diff))) +
      stop_distance_buffer;                                                              // 停止距离的buffer(缓冲区)
  stop_distance -= vehicle_param.front_edge_to_center();
  stop_distance = std::min(stop_distance, FLAGS_max_stop_distance_obstacle);
  stop_distance = std::max(stop_distance, FLAGS_min_stop_distance_obstacle);
  return stop_distance;    // 获得最小安全距离
}

void PathObstacle::BuildReferenceLineStBoundary(
    const ReferenceLine& reference_line, const double adc_start_s) {                     // 通过中心参考线和adc(自动驾驶车辆)在s方向上的起点
  const auto& adc_param =
      VehicleConfigHelper::instance()->GetConfig().vehicle_param();                      // 获得车辆参数
  const double adc_width = adc_param.width();                                            // 车的宽度
  if (obstacle_->IsStatic() ||                                                           // 如果是静态障碍物或者障碍物没有轨迹点
      obstacle_->Trajectory().trajectory_point().empty()) {
    std::vector<std::pair<STPoint, STPoint>> point_pairs;                                // ST的点对
    double start_s = perception_sl_boundary_.start_s();                                  // 起点
    double end_s = perception_sl_boundary_.end_s();                                      // 终点
    if (end_s - start_s < kStBoundaryDeltaS) {                                           // 障碍物的最小单位为0.2米
      end_s = start_s + kStBoundaryDeltaS;
    }
    if (!reference_line.IsBlockRoad(obstacle_->PerceptionBoundingBox(),                  // 看看障碍物是否堵在路上
                                    adc_width)) {
      return;                                                                            // 如果不在路中间就直接返回, 不用考虑
    }
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, 0.0),                        // 如果堵在路中间, 就构造点对, 然后构造参考线的st坐标系下的boundary(边框)
                             STPoint(end_s - adc_start_s, 0.0));
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, FLAGS_st_max_t),
                             STPoint(end_s - adc_start_s, FLAGS_st_max_t));
    reference_line_st_boundary_ = StBoundary(point_pairs);                              // 中心参考线在st坐标系下的边框(boundary)
  } else {                                                                              // 如果是动态障碍物
    if (BuildTrajectoryStBoundary(reference_line, adc_start_s,                          // 动态障碍物必须通过起点, 参考线, 生成一个参考线的boundary(边框)
                                  &reference_line_st_boundary_)) {                      // 调用下面的函数(进行动态障碍物的边框生成)
      ADEBUG << "Found st_boundary for obstacle " << id_;                               // 输出障碍物的id好, 最大,最小的t(时间)和s(路程)
      ADEBUG << "st_boundary: min_t = " << reference_line_st_boundary_.min_t()
             << ", max_t = " << reference_line_st_boundary_.max_t()
             << ", min_s = " << reference_line_st_boundary_.min_s()
             << ", max_s = " << reference_line_st_boundary_.max_s();
    } else {
      ADEBUG << "No st_boundary for obstacle " << id_;
    }
  }
}
// 生成轨迹的st坐标下的边框
bool PathObstacle::BuildTrajectoryStBoundary(
    const ReferenceLine& reference_line, const double adc_start_s,                      // 参考线, 起点
    StBoundary* const st_boundary) {                                                    // 生成一个st坐标系下的边界(doundary)
  const auto& object_id = obstacle_->Id();                                              // 获取障碍物的id号
  const auto& perception = obstacle_->Perception();                                     // 感知的障碍物
  if (!IsValidObstacle(perception)) {                                                   // 感知到的障碍物是否已经合法了
    AERROR << "Fail to build trajectory st boundary because object is not "
              "valid. PerceptionObstacle: "
           << perception.DebugString();
    return false;
  }
  const double object_width = perception.width();                                       // 感知到障碍物的宽度
  const double object_length = perception.length();                                     // 感知到障碍物的长度
  const auto& trajectory_points = obstacle_->Trajectory().trajectory_point();           // 获取动态障碍物的轨迹点
  if (trajectory_points.empty()) {                                                      // 错误检查, 需要障碍物需要有轨迹(因为是动态障碍物)
    AWARN << "object " << object_id << " has no trajectory points";
    return false;                                                                       // 动态障碍物没有轨迹的话, 就会生成轨迹的st坐标下的boundary(边框)失败
  }
  const auto& adc_param =                                                               // 从VehicleConfigHelper的单例对象中获取车辆的参数
      VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  const double adc_length = adc_param.length();                                         // 自动驾驶车辆的长度
  const double adc_half_length = adc_length / 2.0;                                      // 取长度的一半
  const double adc_width = adc_param.width();                                           // 自动驾驶车辆的宽度
  common::math::Box2d min_box({0, 0}, 1.0, 1.0, 1.0);                                   // 初始化一个最小的box
  common::math::Box2d max_box({0, 0}, 1.0, 1.0, 1.0);                                   // 初始化一个最大的box
  std::vector<std::pair<STPoint, STPoint>> polygon_points;                              // 一个多边形的点(在st坐标系中的一个点对组成的多表现的点)

  SLBoundary last_sl_boundary;                                                          // 上一个sl的边框点
  int last_index = 0;                                                                   // 上一次的索引值

  for (int i = 1; i < trajectory_points.size(); ++i) {                                  // 迭代轨迹的所有点
    ADEBUG << "last_sl_boundary: " << last_sl_boundary.ShortDebugString();

    const auto& first_traj_point = trajectory_points[i - 1];                            // 第一个轨迹点
    const auto& second_traj_point = trajectory_points[i];                               // 第二个轨迹点
    const auto& first_point = first_traj_point.path_point();
    const auto& second_point = second_traj_point.path_point();

    double total_length =                                                               // 获得两个点的总距离
        object_length + common::util::DistanceXY(first_point, second_point);

    common::math::Vec2d center((first_point.x() + second_point.x()) / 2.0,              // 障碍物的中心点
                               (first_point.y() + second_point.y()) / 2.0);
    common::math::Box2d object_moving_box(center, first_point.theta(),                  // 构造一个新的box
                                          total_length, object_width);
    SLBoundary object_boundary;                                                         // sl的边框
    // NOTICE: this method will have errors when the reference line is not
    // straight. Need double loop to cover all corner cases.
    const double distance_xy =
        common::util::DistanceXY(trajectory_points[last_index].path_point(),            // 到最后轨迹点的距离
                                 trajectory_points[i].path_point());
    if (last_sl_boundary.start_l() > distance_xy ||
        last_sl_boundary.end_l() < -distance_xy) {
      continue;
    }

    const double mid_s =
        (last_sl_boundary.start_s() + last_sl_boundary.end_s()) / 2.0;                  // boundary的中点
    const double start_s = std::fmax(0.0, mid_s - 2.0 * distance_xy);                   // 起点
    const double end_s = (i == 1) ? reference_line.Length()                             // 终点
                                  : std::fmin(reference_line.Length(),
                                              mid_s + 2.0 * distance_xy);

    if (!reference_line.GetApproximateSLBoundary(object_moving_box, start_s,
                                                 end_s, &object_boundary)) {
      AERROR << "failed to calculate boundary";
      return false;
    }

    // update history record
    last_sl_boundary = object_boundary;                                                 // 更新历史点
    last_index = i;

    // skip if object is entirely on one side of reference line.
    constexpr double kSkipLDistanceFactor = 0.4;                                        // 如果对象完全位于参考线的一侧就跳过
    const double skip_l_distance =                                                      // 跳过的距离公式是boundary的距离差的0.4倍再加上自动驾驶车辆的一半
        (object_boundary.end_s() - object_boundary.start_s()) *
            kSkipLDistanceFactor +
        adc_width / 2.0;

    if (std::fmin(object_boundary.start_l(), object_boundary.end_l()) >
            skip_l_distance ||
        std::fmax(object_boundary.start_l(), object_boundary.end_l()) <
            -skip_l_distance) {
      continue;
    }

    if (object_boundary.end_s() < 0) {  // skip if behind reference line
      continue;
    }
    constexpr double kSparseMappingS = 20.0;                                            // 稀疏地图的距离是20米
    const double st_boundary_delta_s =
        (std::fabs(object_boundary.start_s() - adc_start_s) > kSparseMappingS)
            ? kStBoundarySparseDeltaS                                                   // 稀疏地图的delta值为1.0米
            : kStBoundaryDeltaS;
    const double object_s_diff =
        object_boundary.end_s() - object_boundary.start_s();                            // 物体在s轴上的差值
    if (object_s_diff < st_boundary_delta_s) {
      continue;
    }
    const double delta_t =
        second_traj_point.relative_time() - first_traj_point.relative_time();           // 时间戳
    double low_s = std::max(object_boundary.start_s() - adc_half_length, 0.0);
    bool has_low = false;                                                               // 初始化下界为false
    double high_s =
        std::min(object_boundary.end_s() + adc_half_length, FLAGS_st_max_s);
    bool has_high = false;                                                              // 初始化上界为false
    while (low_s + st_boundary_delta_s < high_s && !(has_low && has_high)) {            // 这里使用的是什么算法???
      if (!has_low) {
        auto low_ref = reference_line.GetReferencePoint(low_s);
        has_low = object_moving_box.HasOverlap(
            {low_ref, low_ref.heading(), adc_length, adc_width});
        low_s += st_boundary_delta_s;
      }
      if (!has_high) {
        auto high_ref = reference_line.GetReferencePoint(high_s);
        has_high = object_moving_box.HasOverlap(
            {high_ref, high_ref.heading(), adc_length, adc_width});
        high_s -= st_boundary_delta_s;
      }
    }
    if (has_low && has_high) {                                                          // 上界和下界都存在的情况下
      low_s -= st_boundary_delta_s;
      high_s += st_boundary_delta_s;
      double low_t =
          (first_traj_point.relative_time() +
           std::fabs((low_s - object_boundary.start_s()) / object_s_diff) *
               delta_t);
      polygon_points.emplace_back(
          std::make_pair(STPoint{low_s - adc_start_s, low_t},
                         STPoint{high_s - adc_start_s, low_t}));
      double high_t =
          (first_traj_point.relative_time() +
           std::fabs((high_s - object_boundary.start_s()) / object_s_diff) *
               delta_t);
      if (high_t - low_t > 0.05) {
        polygon_points.emplace_back(
            std::make_pair(STPoint{low_s - adc_start_s, high_t},
                           STPoint{high_s - adc_start_s, high_t}));
      }
    }
  }
  if (!polygon_points.empty()) {
    std::sort(polygon_points.begin(), polygon_points.end(),
              [](const std::pair<STPoint, STPoint>& a,
                 const std::pair<STPoint, STPoint>& b) {
                return a.first.t() < b.first.t();
              });
    auto last = std::unique(polygon_points.begin(), polygon_points.end(),
                            [](const std::pair<STPoint, STPoint>& a,
                               const std::pair<STPoint, STPoint>& b) {
                              return std::fabs(a.first.t() - b.first.t()) <
                                     kStBoundaryDeltaT;                          // 时间的采样的最小单位值
                            });
    polygon_points.erase(last, polygon_points.end());
    if (polygon_points.size() > 2) {
      *st_boundary = StBoundary(polygon_points);
    }
  } else {
    return false;
  }
  return true;
}

const StBoundary& PathObstacle::reference_line_st_boundary() const {              // 直接返回参考线的boundary
  return reference_line_st_boundary_;
}

const StBoundary& PathObstacle::st_boundary() const { return st_boundary_; }      // 返回该障碍物的st坐标系下的boundary

const std::vector<std::string>& PathObstacle::decider_tags() const {              // 返回决策的便签
  return decider_tags_;
}

const std::vector<ObjectDecisionType>& PathObstacle::decisions() const {          // 障碍物的决策类型
  return decisions_;
}

bool PathObstacle::IsLateralDecision(const ObjectDecisionType& decision) {        // 横向决策
  return decision.has_ignore() || decision.has_nudge() ||
         decision.has_sidepass();
}

bool PathObstacle::IsLongitudinalDecision(const ObjectDecisionType& decision) {   // 是否是纵向决策
  return decision.has_ignore() || decision.has_stop() || decision.has_yield() ||
         decision.has_follow() || decision.has_overtake();
}

ObjectDecisionType PathObstacle::MergeLongitudinalDecision(                       // 合并纵向的决策
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {          // 物体的tag没有被设置
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, lhs.object_tag_case());   // 纵向的策略按照安全性排序
  const auto rhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_stop()) {                                                  // 停止
      return lhs.stop().distance_s() < rhs.stop().distance_s() ? lhs : rhs;
    } else if (lhs.has_yield()) {                                                 // 让行
      return lhs.yield().distance_s() < rhs.yield().distance_s() ? lhs : rhs;
    } else if (lhs.has_follow()) {                                                // 跟行
      return lhs.follow().distance_s() < rhs.follow().distance_s() ? lhs : rhs;
    } else if (lhs.has_overtake()) {                                              // 超车
      return lhs.overtake().distance_s() > rhs.overtake().distance_s() ? lhs
                                                                       : rhs;
    } else {
      DCHECK(false) << "Unknown decision";
    }
  }
  return lhs;  // stop compiler complaining
}

const ObjectDecisionType& PathObstacle::LongitudinalDecision() const {            // 返回纵向的决策
  return longitudinal_decision_;
}

const ObjectDecisionType& PathObstacle::LateralDecision() const {                 // 返回横向的决策
  return lateral_decision_;
}

bool PathObstacle::IsIgnore() const {                                             // 一个障碍物的决策能不能被忽略
  return IsLongitudinalIgnore() && IsLateralIgnore();                             // 横向和纵向决策都能被忽略的话就能都被忽略
}

bool PathObstacle::IsLongitudinalIgnore() const {                                 // 纵向的决策
  return longitudinal_decision_.has_ignore();
}

bool PathObstacle::IsLateralIgnore() const {                                      // 横向的决策
  return lateral_decision_.has_ignore();
}

ObjectDecisionType PathObstacle::MergeLateralDecision(                            // 两个不同的决策综合考虑再做决策
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {          // 如果有一个决策的tag没有设置的话, 就选择另外一个
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, lhs.object_tag_case());        // 从字典中找一个值, 表示的是决策的优先级
  const auto rhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {                                                 // 返回一个决策优先级更高的一个
    return lhs;
  } else {
    if (lhs.has_ignore() || lhs.has_sidepass()) {                                 // 如果左边的决策可以忽略或者是绕行
      return rhs;                                                                 // 那就返回右边的决策
    } else if (lhs.has_nudge()) {                                                 // 如果左边的决策是微调
      DCHECK(lhs.nudge().type() == rhs.nudge().type())                            // 同时向左边微调和同时向右边微调, 这是不可能的呀
          << "could not merge left nudge and right nudge";
      return std::fabs(lhs.nudge().distance_l()) >                                // 那边的微调距离比较大, 就像那边走
                     std::fabs(rhs.nudge().distance_l())
                 ? lhs
                 : rhs;
    }
  }
  DCHECK(false) << "Does not have rule to merge decision: "                       // 还是返回左边的策略
                << lhs.ShortDebugString()
                << " and decision: " << rhs.ShortDebugString();
  return lhs;
}

bool PathObstacle::HasLateralDecision() const {                                   // 检查看到一个障碍物的横向决策
  return lateral_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool PathObstacle::HasLongitudinalDecision() const {                              // 检查看到一个障碍物的纵向决策
  return longitudinal_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool PathObstacle::HasNonIgnoreDecision() const {                                 // 是否有不能忽略的决策
  return (HasLateralDecision() && !IsLateralIgnore()) ||
         (HasLongitudinalDecision() && !IsLongitudinalIgnore());
}

const Obstacle* PathObstacle::obstacle() const { return obstacle_; }              // 返回道路上的障碍物

void PathObstacle::AddLongitudinalDecision(const std::string& decider_tag,        // 添加一个纵向的决策
                                           const ObjectDecisionType& decision) {
  DCHECK(IsLongitudinalDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a longitudinal decision";
  longitudinal_decision_ =
      MergeLongitudinalDecision(longitudinal_decision_, decision);                // 通过合并纵向的决策来获得纵向的策略
  ADEBUG << decider_tag << " added obstacle " << Id()
         << " longitudinal decision: " << decision.ShortDebugString()
         << ". The merged decision is: "
         << longitudinal_decision_.ShortDebugString();
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

void PathObstacle::AddLateralDecision(const std::string& decider_tag,            // 通过tag和具体的决策向横向中添加一个策略
                                      const ObjectDecisionType& decision) {
  DCHECK(IsLateralDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a lateral decision";
  lateral_decision_ = MergeLateralDecision(lateral_decision_, decision);
  ADEBUG << decider_tag << " added obstacle " << Id()
         << " a lateral decision: " << decision.ShortDebugString()
         << ". The merged decision is: "
         << lateral_decision_.ShortDebugString();
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

const std::string PathObstacle::DebugString() const {
  std::stringstream ss;
  ss << "PathObstacle id: " << id_;
  for (std::size_t i = 0; i < decisions_.size(); ++i) {
    ss << " decision: " << decisions_[i].DebugString() << ", made by "
       << decider_tags_[i];
  }
  if (lateral_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    ss << "lateral decision: " << lateral_decision_.ShortDebugString();
  }
  if (longitudinal_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    ss << "longitutional decision: "
       << longitudinal_decision_.ShortDebugString();
  }
  return ss.str();
}

const SLBoundary& PathObstacle::PerceptionSLBoundary() const {                  // 返回感知到障碍物的sl坐标下的边框(boundary)
  return perception_sl_boundary_;
}

void PathObstacle::SetStBoundary(const StBoundary& boundary) {                  // 设置st坐标下的边框(boundary)
  st_boundary_ = boundary;
}

void PathObstacle::SetStBoundaryType(const StBoundary::BoundaryType type) {     // 设置st边框的类型
  st_boundary_.SetBoundaryType(type);
}

void PathObstacle::EraseStBoundary() { st_boundary_ = StBoundary(); }           // 删除st坐标系下的boundary(边框)

void PathObstacle::SetReferenceLineStBoundary(const StBoundary& boundary) {     // 设置中心参考线的st坐标系下的boundary
  reference_line_st_boundary_ = boundary;
}

void PathObstacle::SetReferenceLineStBoundaryType(                              // 设置中心参考线的st坐标系下boundary的类型
    const StBoundary::BoundaryType type) {
  reference_line_st_boundary_.SetBoundaryType(type);
}

void PathObstacle::EraseReferenceLineStBoundary() {                             // 删除中心参考线的boundary
  reference_line_st_boundary_ = StBoundary();                                   // 默认构造函数构造一个对象返回(空对象)
}

bool PathObstacle::IsValidObstacle(
    const perception::PerceptionObstacle& perception_obstacle) {
  const double object_width = perception_obstacle.width();                       // 感知障碍物的宽度和长度
  const double object_length = perception_obstacle.length();

  const double kMinObjectDimension = 1.0e-6;                                     // 障碍物的最小维度
  return !std::isnan(object_width) && !std::isnan(object_length) &&
         object_width > kMinObjectDimension &&                                   // 宽度和长度至少要大于1.0e-6
         object_length > kMinObjectDimension;                                    // 长度的限制
}

}  // namespace planning
}  // namespace apollo
