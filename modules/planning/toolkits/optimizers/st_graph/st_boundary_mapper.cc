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

#include "modules/planning/toolkits/optimizers/st_graph/st_boundary_mapper.h"

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;                                                      // 错误码
using apollo::common::PathPoint;                                                      // path上的离散点(x,y,z)的 坐标
using apollo::common::SLPoint;                                                        // sl坐标点
using apollo::common::Status;                                                         // 状态
using apollo::common::TrajectoryPoint;                                                // 轨迹点
using apollo::common::VehicleParam;                                                   // 车辆的参数
using apollo::common::math::Box2d;                                                    // 二维的box
using apollo::common::math::Vec2d;                                                    // 二维的向量
using apollo::common::util::StrCat;                                                   // 拷贝字符串

namespace {
constexpr double boundary_t_buffer = 0.1;                                             // 时间的buff设置为0.1秒
constexpr double boundary_s_buffer = 1.0;                                             // s方向的buff设置为1米
}  // namespace

StBoundaryMapper::StBoundaryMapper(const SLBoundary& adc_sl_boundary,                 // 构造st边框的映射器
                                   const StBoundaryConfig& config,
                                   const ReferenceLine& reference_line,
                                   const PathData& path_data,
                                   const double planning_distance,
                                   const double planning_time,
                                   bool is_change_lane)
    : adc_sl_boundary_(adc_sl_boundary),
      st_boundary_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()),
      planning_distance_(planning_distance),
      planning_time_(planning_time),
      is_change_lane_(is_change_lane) {}                                              // 初始化列表进行构造

Status StBoundaryMapper::CreateStBoundary(PathDecision* path_decision) const {
  const auto& path_obstacles = path_decision->path_obstacles();                       // 获得path上的障碍物

  if (planning_time_ < 0.0) {                                                         // 做planning的时间为负数, 肯定有问题
    const std::string msg = "Fail to get params since planning_time_ < 0.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (path_data_.discretized_path().NumOfPoints() < 2) {                              // 离散点的个数小于2
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data_.discretized_path().NumOfPoints() << ".";
    return Status(ErrorCode::PLANNING_ERROR,                                          // 离散的点太少
                  "Fail to get params because of too few path points");
  }

  PathObstacle* stop_obstacle = nullptr;                                              // 停止的障碍物
  ObjectDecisionType stop_decision;                                                   // 决策的类型
  double min_stop_s = std::numeric_limits<double>::max();                             // 最近的停止点

  for (const auto* const_path_obstacle : path_obstacles.Items()) {                    // 迭代所有的path上的障碍物
    auto* path_obstacle = path_decision->Find(const_path_obstacle->Id());             // 通过id地址找到对应的障碍物
    if (!path_obstacle->HasLongitudinalDecision()) {                                  // 是否有纵向的决策
      if (!MapWithoutDecision(path_obstacle).ok()) {                                  // 将障碍物和决策者映射到一个map中
        std::string msg = StrCat("Fail to map obstacle ", path_obstacle->Id(),
                                 " without decision.");                               // 如果出错了就返回
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      continue;                                                                       // 映射进来一个决策就继续迭代障碍物
    }
    const auto& decision = path_obstacle->LongitudinalDecision();                     // 获取纵向的决策器
    if (decision.has_stop()) {                                                        // 决策是停止
      const double stop_s = path_obstacle->PerceptionSLBoundary().start_s() +         // 停止点
                            decision.stop().distance_s();
      // this is a rough estimation based on reference line s, so that a large
      // buffer is used.
      constexpr double stop_buff = 15.0;                                              // 基于参考线粗略计算估计, 所以使用15的大缓冲器
      if (stop_s + stop_buff < adc_sl_boundary_.end_s()) {                            // 停止点在后面, 肯定不合理, 所以必须报错
        AERROR << "Invalid stop decision. not stop at behind of current "
                  "position. stop_s : "
               << stop_s << ", and current adc_s is; "
               << adc_sl_boundary_.end_s();
        return Status(ErrorCode::PLANNING_ERROR, "invalid decision");
      }
      if (stop_s < min_stop_s) {                                                      // 一直迭代找到最小的停止距离(s)
        stop_obstacle = path_obstacle;                                                // 迭代停止障碍物
        min_stop_s = stop_s;                                                          // 获得最小停止的距离
        stop_decision = decision;                                                     // 获得停止点的决策
      }
    } else if (decision.has_follow() || decision.has_overtake() ||                    // 否者是跟车， 超车， 避让等决策
               decision.has_yield()) {
      if (!MapWithDecision(path_obstacle, decision).ok()) {                           // 将path上的障碍物和一个决策绑定到一起
        AERROR << "Fail to map obstacle " << path_obstacle->Id()                      // 绑定出错
               << " with decision: " << decision.DebugString();
        return Status(ErrorCode::PLANNING_ERROR,
                      "Fail to map overtake/yield decision");
      }
    } else if (!decision.has_ignore()) {                                              // 忽略的障碍物就没有什么的决策
      AWARN << "No mapping for decision: " << decision.DebugString();
    }
  }

  if (stop_obstacle) {                                                                // 如果是停止障碍物
    bool success = MapStopDecision(stop_obstacle, stop_decision);                     // 将停止决策和停止障碍物捆绑在一起
    if (!success) {
      std::string msg = "Fail to MapStopDecision.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  return Status::OK();                                                                // 直接返回正常
}

Status StBoundaryMapper::CreateStBoundaryWithHistory(                                 // 利用历史的决策者, 来创建一个st的boundary
    const ObjectDecisions& history_decisions,                                         // 历史的决策者
    PathDecision* path_decision) const {                                              // path上面针对障碍物的决策者
  const auto& path_obstacles = path_decision->path_obstacles();                       // 获得path上的障碍物
  if (planning_time_ < 0.0) {                                                         // 如果planning的时间是负数的话， 肯定是有问题的
    const std::string msg = "Fail to get params since planning_time_ < 0.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);                                    // 直接返回错误的debug信息
  }

  if (path_data_.discretized_path().NumOfPoints() < 2) {                              // 如果离散的点少于两个也是有问题的
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data_.discretized_path().NumOfPoints() << ".";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to get params because of too few path points");               // 重复就是罪恶
  }

  std::unordered_map<std::string, ObjectDecisionType> prev_decision_map;              // 前一个决策的map(映射, 红黑树)
  for (const auto& history_decision : history_decisions.decision()) {                 // 轮寻历史的决策者
    for (const auto& decision : history_decision.object_decision()) {                 // 获得历史决策者的决策
      if (PathObstacle::IsLongitudinalDecision(decision) &&                           // 找出没有被忽略的决策
          !decision.has_ignore()) {
        prev_decision_map[history_decision.id()] = decision;                          // 然后将id号和决策放到具体的map中, 会按id号自动排序
        break;
      }
    }
  }

  PathObstacle* stop_obstacle = nullptr;                                              // 停止障碍物
  ObjectDecisionType stop_decision;                                                   // 停止障碍物的决策
  double min_stop_s = std::numeric_limits<double>::max();                             // 最小的停止点

  for (const auto* const_path_obstacle : path_obstacles.Items()) {                    // 迭代path上所有的障碍物
    auto* path_obstacle = path_decision->Find(const_path_obstacle->Id());             // 通过障碍物的id找到决策者的id号
    auto iter = prev_decision_map.find(path_obstacle->Id());                          // 再通过决策者的id号找到具体的决策
    ObjectDecisionType decision;
    if (iter == prev_decision_map.end()) {                                            // 如果没有对应的决策者的id
      decision.mutable_ignore();                                                      // 就将决策忽略
    } else {
      decision = iter->second;                                                        // 否则取出对应的决策
    }

    if (!path_obstacle->HasLongitudinalDecision()) {                                  // 如果障碍物没有纵向上的决策
      if (!MapWithoutDecision(path_obstacle).ok()) {                                  // 就会映射到一个决策上去
        std::string msg = StrCat("Fail to map obstacle ", path_obstacle->Id(),        // 在MapWithoutDecision中就是复制了一个boundary
                                 " without decision.");
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      if (path_obstacle->st_boundary().IsEmpty() || decision.has_ignore()) {          // 可以忽略的决策就继续
        continue;
      }
    }
    if (path_obstacle->HasLongitudinalDecision()) {                                   // 如果有纵向的决策就返回决策者
      decision = path_obstacle->LongitudinalDecision();
    }
    if (decision.has_stop()) {                                                        // 如果决策者是停止
      const double stop_s = path_obstacle->PerceptionSLBoundary().start_s() +         // 那么就先获取到停止点的s值
                            decision.stop().distance_s();
      // this is a rough estimation based on reference line s, so that a large
      // buffer is used.
      constexpr double stop_buff = 15.0;                                              // 粗略估计, 所以需要加一个大的buff(15.0)
      if (stop_s + stop_buff < adc_sl_boundary_.end_s()) {                            // 起点太小
        AERROR << "Invalid stop decision. not stop at behind of current "
                  "position. stop_s : "
               << stop_s << ", and current adc_s is; "
               << adc_sl_boundary_.end_s();
        return Status(ErrorCode::PLANNING_ERROR, "invalid decision");                 // 直接返回
      }
      if (stop_s < min_stop_s) {                                                      // 找到最小的停止点的s(路程)
        stop_obstacle = path_obstacle;
        min_stop_s = stop_s;
        stop_decision = decision;
      }
    } else if (decision.has_follow() || decision.has_overtake() ||                   // 如果是其他决策
               decision.has_yield()) {
      if (!MapWithDecision(path_obstacle, decision).ok()) {                          // 就重新将path上的障碍物和决策捆绑到一起
        AERROR << "Fail to map obstacle " << path_obstacle->Id()
               << " with decision: " << decision.DebugString();
        return Status(ErrorCode::PLANNING_ERROR,
                      "Fail to map overtake/yield decision");
      }
    } else {                                                                         // 没有决策
      AWARN << "No mapping for decision: " << decision.DebugString();
    }
  }

  if (stop_obstacle) {                                                               // 如果是停止障碍物， 就马上进行映射
    bool success = MapStopDecision(stop_obstacle, stop_decision);
    if (!success) {
      std::string msg = "Fail to MapStopDecision.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  return Status::OK();
}

bool StBoundaryMapper::MapStopDecision(                                              // 停止障碍物和停止决策两个相互映射
    PathObstacle* stop_obstacle,                                                     // 停止障碍物
    const ObjectDecisionType& stop_decision) const {                                 // 停止决策
  DCHECK(stop_decision.has_stop()) << "Must have stop decision";                     // 检查停止决策是否存在

  if (stop_obstacle->PerceptionSLBoundary().start_s() >                              // 预测的sl的boundary的起点, 大于自动驾驶的加速点+一个值
      adc_sl_boundary_.end_s() + planning_distance_) {
    return true;
  }

  double st_stop_s = 0.0;                                                            // 停止障碍物的s(st坐标系中的s)
  const double stop_ref_s = stop_obstacle->PerceptionSLBoundary().start_s() +
                            stop_decision.stop().distance_s() -
                            vehicle_param_.front_edge_to_center();                   // 中心参考线中的s

  if (stop_ref_s > path_data_.frenet_frame_path().points().back().s()) {             // 超出了当前的path
    st_stop_s =
        path_data_.discretized_path().EndPoint().s() +
        (stop_ref_s - path_data_.frenet_frame_path().points().back().s());
  } else {
    PathPoint stop_point;                                                            // 获得参考点
    if (!path_data_.GetPathPointWithRefS(stop_ref_s, &stop_point)) {
      AERROR << "Fail to get path point from reference s. The sl boundary of "
                "stop obstacle "
             << stop_obstacle->Id()
             << " is: " << stop_obstacle->PerceptionSLBoundary().DebugString();
      return false;
    }

    st_stop_s = stop_point.s();
  }

  constexpr double kStopEpsilon = 1e-2;                                              // 无穷小
  const double s_min = std::max(0.0, st_stop_s - kStopEpsilon);
  const double s_max =
      std::fmax(s_min, std::fmax(planning_distance_, reference_line_.Length()));     // s最大的值

  std::vector<std::pair<STPoint, STPoint>> point_pairs;                              // 点对组成的数组
  point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));                // 获得最小，最大的值
  point_pairs.emplace_back(
      STPoint(s_min, planning_time_),
      STPoint(s_max + st_boundary_config_.boundary_buffer(), planning_time_));
  auto boundary = StBoundary(point_pairs);
  boundary.SetBoundaryType(StBoundary::BoundaryType::STOP);                          // 停止的类型
  boundary.SetCharacteristicLength(st_boundary_config_.boundary_buffer());
  boundary.SetId(stop_obstacle->Id());                                               // 设置停止障碍物的boundary
  stop_obstacle->SetStBoundary(boundary);
  return true;
}

Status StBoundaryMapper::MapWithoutDecision(PathObstacle* path_obstacle) const {              // 输出是障碍物的st的边框
  std::vector<STPoint> lower_points;                                                          // 上界和下界
  std::vector<STPoint> upper_points;

  if (!GetOverlapBoundaryPoints(path_data_.discretized_path().path_points(),
                                *(path_obstacle->obstacle()), &upper_points,
                                &lower_points)) {                                             // 获取边界重叠的点
    return Status::OK();
  }

  auto boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)                  // 通过上下界构造一个st的边框
                      .ExpandByS(boundary_s_buffer)                                           // 然后扩展s和扩展t
                      .ExpandByT(boundary_t_buffer);
  boundary.SetId(path_obstacle->Id());                                                        // 设置障碍物的id地址
  const auto& prev_st_boundary = path_obstacle->st_boundary();                                // 设置前一个点的st边框
  const auto& ref_line_st_boundary =                                                          // 参考线的st边框
      path_obstacle->reference_line_st_boundary();
  if (!prev_st_boundary.IsEmpty()) {                                                          // 如果不为空
    boundary.SetBoundaryType(prev_st_boundary.boundary_type());                               // 就设置为st boundary的类型
  } else if (!ref_line_st_boundary.IsEmpty()) {                                               // 如果前向的参考线的st boundary不为空
    boundary.SetBoundaryType(ref_line_st_boundary.boundary_type());                           // 将设置新建boundary的类型
  }

  path_obstacle->SetStBoundary(boundary);                                                     // 最后更新st的类型
  return Status::OK();                                                                        // 返回正常ok
}

bool StBoundaryMapper::GetOverlapBoundaryPoints(                                            // 获取重叠的点
    const std::vector<PathPoint>& path_points, const Obstacle& obstacle,                    // path的轨迹点, 障碍物， 输出为上界和下界
    std::vector<STPoint>* upper_points,                                                     // 上界点
    std::vector<STPoint>* lower_points) const {                                             // 下界点
  DCHECK_NOTNULL(upper_points);                                                             // 有效性检查
  DCHECK_NOTNULL(lower_points);
  DCHECK(upper_points->empty());
  DCHECK(lower_points->empty());
  DCHECK_GT(path_points.size(), 0);                                                         // path中的点的个数要大于0

  if (path_points.empty()) {                                                                // 没有离散的点
    AERROR << "No points in path_data_.discretized_path().";
    return false;
  }

  const auto& trajectory = obstacle.Trajectory();                                           // 获得障碍物的轨迹
  if (trajectory.trajectory_point_size() == 0) {                                            // 轨迹的点不存在， 肯定是静态障碍物
    if (!obstacle.IsStatic()) {                                                             // 障碍物不是静态障碍物
      AWARN << "Non-static obstacle[" << obstacle.Id()
            << "] has NO prediction trajectory."
            << obstacle.Perception().ShortDebugString();
    }
    for (const auto& curr_point_on_path : path_points) {                                    // 迭代path点中的所有采样点
      if (curr_point_on_path.s() > planning_distance_) {                                    // 大于做planning的距离是不可能的
        break;
      }
      const Box2d obs_box = obstacle.PerceptionBoundingBox();                               // 感知到bounding的对象box

      if (CheckOverlap(curr_point_on_path, obs_box,                                         // 检查是否有重叠部分
                       st_boundary_config_.boundary_buffer())) {
        const double backward_distance = -vehicle_param_.front_edge_to_center();            // 向后的距离
        const double forward_distance = vehicle_param_.length() +                           // 向前的距离
                                        vehicle_param_.width() +
                                        obs_box.length() + obs_box.width();
        double low_s =
            std::fmax(0.0, curr_point_on_path.s() + backward_distance);                     // s较小的值
        double high_s = std::fmin(planning_distance_,
                                  curr_point_on_path.s() + forward_distance);               // s较大的值
        lower_points->emplace_back(low_s, 0.0);                                             // 对角线上的四个点
        lower_points->emplace_back(low_s, planning_time_);
        upper_points->emplace_back(high_s, 0.0);
        upper_points->emplace_back(high_s, planning_time_);
        break;
      }
    }
  } else {                                                                                  // 是动态障碍物
    const int default_num_point = 50;                                                       // 默认使用50个点进行采样
    DiscretizedPath discretized_path;                                                       // 离散的轨迹
    if (path_points.size() > 2 * default_num_point) {                                       // path中如果有100个点
      const int ratio = path_points.size() / default_num_point;                             // 计算一个比例尺
      std::vector<PathPoint> sampled_path_points;                                           // 将path重新采样
      for (size_t i = 0; i < path_points.size(); ++i) {                                     // 原来的点太多了, 就重新获取更稀疏的点
        if (i % ratio == 0) {
          sampled_path_points.push_back(path_points[i]);
        }
      }
      discretized_path.set_path_points(sampled_path_points);                                // 将更稀疏的点放到离散的轨迹中
    } else {
      discretized_path.set_path_points(path_points);                                        // 如果path中的点没有超过100的话就直接放到离散的path中
    }
    for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {                          // 迭代动态障碍物的轨迹
      const auto& trajectory_point = trajectory.trajectory_point(i);                        // 取出每一个轨迹点
      const Box2d obs_box = obstacle.GetBoundingBox(trajectory_point);                      // 通过一个轨迹点获得一个新的boundary box

      double trajectory_point_time = trajectory_point.relative_time();                      // 相对时间戳
      constexpr double kNegtiveTimeThreshold = -1.0;                                        // 时间的阈值最多为-1.0
      if (trajectory_point_time < kNegtiveTimeThreshold) {                                  // 如果太小， 就跳过这个轨迹点
        continue;
      }

      const double step_length = vehicle_param_.front_edge_to_center();                     // 步长设置为车长度的一半
      for (double path_s = 0.0; path_s < discretized_path.Length();                         // 迭代所有的离散path
           path_s += step_length) {
        const auto curr_adc_path_point = discretized_path.Evaluate(
            path_s + discretized_path.StartPoint().s());                                    // 自动驾驶车辆的path位置
        if (CheckOverlap(curr_adc_path_point, obs_box,
                         st_boundary_config_.boundary_buffer())) {                          // 检查和障碍物的stboundary有没有重叠
          // found overlap, start searching with higher resolution
          const double backward_distance = -step_length;                                    // 如果有重叠就搜索更高的分辨率
          const double forward_distance = vehicle_param_.length() +
                                          vehicle_param_.width() +                          // 前向的距离
                                          obs_box.length() + obs_box.width();
          const double default_min_step = 0.1;  // in meters                                // 默认最小的步长为0.1米
          const double fine_tuning_step_length = std::fmin(
              default_min_step, discretized_path.Length() / default_num_point);             // 最小的转向步长

          bool find_low = false;                                                            // 找到下界
          bool find_high = false;                                                           // 找到了上界        
          double low_s = std::fmax(0.0, path_s + backward_distance);                        // 0和后向距离最大的值
          double high_s =
              std::fmin(discretized_path.Length(), path_s + forward_distance);              // 最小的s

          while (low_s < high_s) {                                                          // 如果下界的s比上界的s小的话， 就会一直的迭代
            if (find_low && find_high) {                                                    // 上下界都找到的话就退出
              break;
            }
            if (!find_low) {
              const auto& point_low = discretized_path.Evaluate(
                  low_s + discretized_path.StartPoint().s());                               // 计算下界点
              if (!CheckOverlap(point_low, obs_box,
                                st_boundary_config_.boundary_buffer())) {                   // 检查是否有重叠
                low_s += fine_tuning_step_length;                                           // 加上转向的步长
              } else {
                find_low = true;                                                            // 找到下界
              }
            }
            if (!find_high) {
              const auto& point_high = discretized_path.Evaluate(
                  high_s + discretized_path.StartPoint().s());
              if (!CheckOverlap(point_high, obs_box,
                                st_boundary_config_.boundary_buffer())) {
                high_s -= fine_tuning_step_length;
              } else {
                find_high = true;                                                            // 找到上界           
              }
            }
          }
          if (find_high && find_low) {                                                       // 上下界都找到了就将s和t放到点对中
            lower_points->emplace_back(
                low_s - st_boundary_config_.point_extension(),
                trajectory_point_time);
            upper_points->emplace_back(
                high_s + st_boundary_config_.point_extension(),
                trajectory_point_time);
          }
          break;
        }
      }
    }
  }
  DCHECK_EQ(lower_points->size(), upper_points->size());                                    // 上界和下界的个数要相等
  return (lower_points->size() > 1 && upper_points->size() > 1);                            // 下界两个点, 上界两个点
}

Status StBoundaryMapper::MapWithDecision(
    PathObstacle* path_obstacle, const ObjectDecisionType& decision) const {                // 将一个障碍物和面对障碍物的决策映射到一个里面
  DCHECK(decision.has_follow() || decision.has_yield() ||                                   // 这里只映射三种类型的决策(跟车, 超车，避让)
         decision.has_overtake())
      << "decision is " << decision.DebugString()
      << ", but it must be follow or yield or overtake.";

  std::vector<STPoint> lower_points;                                                        // 下界点和上界点
  std::vector<STPoint> upper_points;

  if (!GetOverlapBoundaryPoints(path_data_.discretized_path().path_points(),                // 获取重叠boundary中的点
                                *(path_obstacle->obstacle()), &upper_points,
                                &lower_points)) {
    return Status::OK();                                                                    // 正常的话就直接返回
  }

  if (decision.has_follow() && lower_points.back().t() < planning_time_) {                  // 跟车的场景
    const double diff_s = lower_points.back().s() - lower_points.front().s();               // s方向的增量
    const double diff_t = lower_points.back().t() - lower_points.front().t();               // t方向的增量
    double extend_lower_s =
        diff_s / diff_t * (planning_time_ - lower_points.front().t()) +                     // 扩展下界
        lower_points.front().s();
    const double extend_upper_s =
        extend_lower_s + (upper_points.back().s() - lower_points.back().s()) +              // 扩展上界
        1.0;
    upper_points.emplace_back(extend_upper_s, planning_time_);
    lower_points.emplace_back(extend_lower_s, planning_time_);
  }

  auto boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)                // 通过上界和下界的点产生一个st的boundary(边框)
                      .ExpandByS(boundary_s_buffer)
                      .ExpandByT(boundary_t_buffer);

  // get characteristic_length and boundary_type.
  StBoundary::BoundaryType b_type = StBoundary::BoundaryType::UNKNOWN;                      // 获取特征的长度
  double characteristic_length = 0.0;                                                       // 特征的长度
  if (decision.has_follow()) {                                                              // 决定跟车
    characteristic_length = std::fabs(decision.follow().distance_s());                      // 跟车的长度就是决策的距离
    b_type = StBoundary::BoundaryType::FOLLOW;                                              // 将boundary的类型设置为跟车
  } else if (decision.has_yield()) {                                                        // 必然的决策
    characteristic_length = std::fabs(decision.yield().distance_s());
    boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = StBoundary::BoundaryType::YIELD;
  } else if (decision.has_overtake()) {                                                     // 超车的决策
    characteristic_length = std::fabs(decision.overtake().distance_s());
    b_type = StBoundary::BoundaryType::OVERTAKE;
  } else {                                                                                  // 都不是就进行报错
    DCHECK(false) << "Obj decision should be either yield or overtake: "
                  << decision.DebugString();
  }
  boundary.SetBoundaryType(b_type);                                                         // 设置障碍物的boundary的类型
  boundary.SetId(path_obstacle->obstacle()->Id());                                          // 设置障碍物的id
  boundary.SetCharacteristicLength(characteristic_length);                                  // 设置特征的长度
  path_obstacle->SetStBoundary(boundary);                                                   // 将已经弄好的boundary赋值

  return Status::OK();
}

bool StBoundaryMapper::CheckOverlap(const PathPoint& path_point,                            // 检车重叠点
                                    const Box2d& obs_box,                                   // path上的点, 二维的box
                                    const double buffer) const {                            // buffer的大小
  double left_delta_l = 0.0;                                                                // 左边的偏差值
  double right_delta_l = 0.0;                                                               // 右边的偏差值
  if (is_change_lane_) {                                                                    // 是否已经变道
    if ((adc_sl_boundary_.start_l() + adc_sl_boundary_.end_l()) / 2.0 > 0.0) {
      // change to right
      left_delta_l = 1.0;                                                                   // 左边偏差(右拐就添加左边的delta值)
    } else {
      // change to left
      right_delta_l = 1.0;                                                                  // 左拐就添加右边的delta值
    }
  }
  Vec2d vec_to_center =
      Vec2d((vehicle_param_.front_edge_to_center() -
             vehicle_param_.back_edge_to_center()) /
                2.0,
            (vehicle_param_.left_edge_to_center() + left_delta_l -
             vehicle_param_.right_edge_to_center() + right_delta_l) /
                2.0)
          .rotate(path_point.theta());
  Vec2d center = Vec2d(path_point.x(), path_point.y()) + vec_to_center;

  const Box2d adc_box =
      Box2d(center, path_point.theta(), vehicle_param_.length() + 2 * buffer,
            vehicle_param_.width() + 2 * buffer);
  return obs_box.HasOverlap(adc_box);                                                      // 检查障碍物和自动驾驶车辆是否有重叠点
}

}  // namespace planning
}  // namespace apollo
