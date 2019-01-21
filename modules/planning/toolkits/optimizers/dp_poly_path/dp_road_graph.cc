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
 * @file dp_road_graph.h
 **/

#include "modules/planning/toolkits/optimizers/dp_poly_path/dp_road_graph.h"

#include <algorithm>
#include <utility>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/util/thread_pool.h"
#include "modules/common/util/util.h"

#include "modules/map/hdmap/hdmap_util.h"

#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;                                                        // 错误码
using apollo::common::SLPoint;                                                          // sl坐标
using apollo::common::Status;                                                           // 车辆的状态
using apollo::common::math::CartesianFrenetConverter;                                   // 笛卡尔坐标系转换为Frenet坐标系
using apollo::common::util::MakeSLPoint;                                                // 产生一个SL坐标系的点
using apollo::common::util::ThreadPool;                                                 // 线程池
// 构造函数
DPRoadGraph::DPRoadGraph(const DpPolyPathConfig &config,                                // dp的多项式路径配置
                         const ReferenceLineInfo &reference_line_info,                  // 参考线的信息
                         const SpeedData &speed_data)                                   // 速度相关的数据
    : config_(config),
      reference_line_info_(reference_line_info),
      reference_line_(reference_line_info.reference_line()),
      speed_data_(speed_data) {}
// 
bool DPRoadGraph::FindPathTunnel(                                                       // 找一条通路
    const common::TrajectoryPoint &init_point,                                          // 轨迹点
    const std::vector<const PathObstacle *> &obstacles,                                 // 存放障碍物的vector
    PathData *const path_data) {                                                        // 路径的数据
  CHECK_NOTNULL(path_data);                                                             // 检查path的数据是否存在

  init_point_ = init_point;                                                             // 起点
  if (!reference_line_.XYToSL(
          {init_point_.path_point().x(), init_point_.path_point().y()},                 // 转换为sl坐标系
          &init_sl_point_)) {
    AERROR << "Fail to create init_sl_point from : "
           << init_point.DebugString();
    return false;
  }
  // 转换为frenet坐标系, 主要是有个frenet的frame点
  if (!CalculateFrenetPoint(init_point_, &init_frenet_frame_point_)) {                  // 将起点转换为frenet的坐标点(l的一次和二次微分)
    AERROR << "Fail to create init_frenet_frame_point_ from : "                         // 容错处理
           << init_point_.DebugString();
    return false;
  }

  std::vector<DPRoadGraphNode> min_cost_path;                                           // 最小路径的代价函数
  if (!GenerateMinCostPath(obstacles, &min_cost_path)) {                                // 产生最小代价的路径, 是不是这个函数就是特别关键的那个函数呢?
    AERROR << "Fail to generate graph!";                                                // 代价越小越好, 路网中的点保存到一个数组中vector
    return false;                                                                       // 没有找到一个最小代价的path的话, 就直接返回错误
  }
  std::vector<common::FrenetFramePoint> frenet_path;                                    // 将路网图中的数据转换为frenet的坐标点
  float accumulated_s = init_sl_point_.s();                                             // 起点的路程(s)
  const float path_resolution = config_.path_resolution();                              // path的分辨率

  for (std::size_t i = 1; i < min_cost_path.size(); ++i) {                              // 迭代最小路径的所有点(从1开始迭代)
    const auto &prev_node = min_cost_path[i - 1];                                       // 前向点
    const auto &cur_node = min_cost_path[i];                                            // 当前点

    const float path_length = cur_node.sl_point.s() - prev_node.sl_point.s();           // 获得图中两个点的长度
    float current_s = 0.0;
    const auto &curve = cur_node.min_cost_curve;                                        // 最小代价的5次多项式曲线
    while (current_s + path_resolution / 2.0 < path_length) {                           // 不断迭代
      const float l = curve.Evaluate(0, current_s);                                     // 通过s计算l
      const float dl = curve.Evaluate(1, current_s);
      const float ddl = curve.Evaluate(2, current_s);                                   // 计算二次微分
      common::FrenetFramePoint frenet_frame_point;                                      // 然后将计算的值转换为frenet的坐标, 即path point点
      frenet_frame_point.set_s(accumulated_s + current_s);
      frenet_frame_point.set_l(l);
      frenet_frame_point.set_dl(dl);
      frenet_frame_point.set_ddl(ddl);
      frenet_path.push_back(std::move(frenet_frame_point));
      current_s += path_resolution;                                                     // 在两个图中的点之间还会进行采样，通过s进行计算l
    }
    if (i == min_cost_path.size() - 1) {                                                // 已经到最后的一个点
      accumulated_s += current_s;                                                       // 加上那点点尾巴
    } else {
      accumulated_s += path_length;                                                     // 否则加上两个节点的长度
    }
  }
  FrenetFramePath tunnel(frenet_path);                                                  // 将frenet的轨迹转换为一个frenet一帧的path(sl坐标系下)
  path_data->SetReferenceLine(&reference_line_);                                        // 设置path data中的参考线
  path_data->SetFrenetPath(tunnel);                                                     // 设置path data中的通道(通路)
  return true;
}

bool DPRoadGraph::GenerateMinCostPath(                                            // 在DP RoadGraph图中的GenerateMinCostPath函数里通过障碍物生成最小代价的路径
    const std::vector<const PathObstacle *> &obstacles,                           // path上的障碍物(path是指在sl坐标系下的车道?)
    std::vector<DPRoadGraphNode> *min_cost_path) {                                // 用一个数组来保存DPRoadGraphNode(路网中的一点)
  CHECK(min_cost_path != nullptr);                                                // 做边界检查

  std::vector<std::vector<common::SLPoint>> path_waypoints;                       // 二维的路网的点(path_waypoints)
  if (!SamplePathWaypoints(init_point_, &path_waypoints) ||                       // 从起点进行数据的采样, 采样的结果保存到二维数组path_waypoints中
      path_waypoints.size() < 1) {
    AERROR << "Fail to sample path waypoints! reference_line_length = "           // 采样出错
           << reference_line_.Length();
    return false;
  }
  path_waypoints.insert(path_waypoints.begin(),
                        std::vector<common::SLPoint>{init_sl_point_});            // 把起点插入到采样的点中
  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();                       // 获取车的配置
  // 轨迹的代价函数? TrajectoryCost又是一个类
  TrajectoryCost trajectory_cost(                                                 // 构造一个轨迹的代价函数
      config_, reference_line_, reference_line_info_.IsChangeLanePath(),
      obstacles, vehicle_config.vehicle_param(), speed_data_, init_sl_point_);

  std::list<std::list<DPRoadGraphNode>> graph_nodes;                              // 图中的点是用的双链表来设计的?
  graph_nodes.emplace_back();                                                     // 构造一个空对象(一个空的链表)
  graph_nodes.back().emplace_back(init_sl_point_, nullptr, ComparableCost());     // 初始化一个节点
  auto &front = graph_nodes.front().front();                                      // 获得链表的头结点的头结点
  size_t total_level = path_waypoints.size();                                     // 一个有多少个level

  for (std::size_t level = 1; level < path_waypoints.size(); ++level) {           // 迭代每个level
    const auto &prev_dp_nodes = graph_nodes.back();                               // 前向的节点
    const auto &level_points = path_waypoints[level];                             // 本level的所有节点

    graph_nodes.emplace_back();                                                   // 新建一个双链表

    std::vector<std::future<void>> futures;                                       // std::future是什么呢? std::future 可以用来获取异步任务的结果，因此可以把它当成一种简单的线程间同步的手段

    for (size_t i = 0; i < level_points.size(); ++i) {                            // 迭代一个level上的所有点
      const auto &cur_point = level_points[i];                                    // 获得当前的在路网图中的点

      graph_nodes.back().emplace_back(cur_point, nullptr);
      auto &cur_node = graph_nodes.back().back();                                 // 当前的结点
      if (FLAGS_enable_multi_thread_in_dp_poly_path) {                            // 是否使能了多线程, 没有使能多线程
        futures.push_back(ThreadPool::pool()->push(std::bind(
            &DPRoadGraph::UpdateNode, this, std::ref(prev_dp_nodes), level,       // 利用多线程更新结点
            total_level, &trajectory_cost, &(front), &(cur_node))));              // 使用线程中的一个点

      } else {
        UpdateNode(prev_dp_nodes, level, total_level, &trajectory_cost, &front,   // 单线程更新结点
                   &cur_node);                                                    // 更新图中的一个点
      }
    }

    for (const auto &f : futures) {                                               // 多线程进行等待
      f.wait();
    }
  }

  // find best path
  DPRoadGraphNode fake_head;                                                      // 虚拟的头结点
  for (const auto &cur_dp_node : graph_nodes.back()) {                            // 不断迭代图中的点
    fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve,                // 找到当前代价最小的点
                         cur_dp_node.min_cost);                                   // 取得path中的第一个点
  }
  // 动态规划是从后向前找, 不是从前往后找, 这样有什么好处呢????
  const auto *min_cost_node = &fake_head;                                         // 第一个level中代价值最小的结点
  while (min_cost_node->min_cost_prev_node) {                                     // 动态规划迭代后面的下一个结点, 所以应该是局部最优的
    min_cost_node = min_cost_node->min_cost_prev_node;
    min_cost_path->push_back(*min_cost_node);                                     // 然后放到最小费用的路径中
  }
  if (min_cost_node != &graph_nodes.front().front()) {                            // 如果最小结点(最后的结点)不等于路网中的起点
    return false;
  }

  std::reverse(min_cost_path->begin(), min_cost_path->end());                     // 将最小花费的路径点翻转180度

  for (const auto &node : *min_cost_path) {                                       // 迭代图中的最小代价的点
    ADEBUG << "min_cost_path: " << node.sl_point.ShortDebugString();              // debug每个点的信息
    planning_debug_->mutable_planning_data()                                      // 将每个点的信息放到planning的debug信息中
        ->mutable_dp_poly_graph()
        ->add_min_cost_point()
        ->CopyFrom(node.sl_point);
  }
  return true;
}

void DPRoadGraph::UpdateNode(const std::list<DPRoadGraphNode> &prev_nodes,       // 一个链表中保存的是之前的节点
                             const uint32_t level, const uint32_t total_level,   // 当前是那个level, 总共有多少level
                             TrajectoryCost *trajectory_cost,                    // 轨迹的代价
                             DPRoadGraphNode *front,                             // 作为输出, 前一个点
                             DPRoadGraphNode *cur_node) {                        // 后一个点
  DCHECK_NOTNULL(trajectory_cost);                                               // 检查指针的合法性
  DCHECK_NOTNULL(front);
  DCHECK_NOTNULL(cur_node);
  for (const auto &prev_dp_node : prev_nodes) {                                  // 迭代所有的前向点
    const auto &prev_sl_point = prev_dp_node.sl_point;                           // 取出前向点的sl坐标
    const auto &cur_point = cur_node->sl_point;                                  // 取出当前点的sl坐标
    float init_dl = 0.0;                                                         // 起点的一次微分
    float init_ddl = 0.0;                                                        // 起点的二次微分
    if (level == 1) {                                                            // level为1的点就是起点所在的那个level
      init_dl = init_frenet_frame_point_.dl();
      init_ddl = init_frenet_frame_point_.ddl();                                 // 直接复制起点的dl和ddl
    }
    QuinticPolynomialCurve1d curve(prev_sl_point.l(), init_dl, init_ddl,         // 构造一个5次平滑的曲线
                                   cur_point.l(), 0.0, 0.0,
                                   cur_point.s() - prev_sl_point.s());

    if (!IsValidCurve(curve)) {                                                  // 曲线不合法就继续进行
      continue;
    }
    const auto cost =                                                            // 如果合法, 就计算轨迹的代价
        trajectory_cost->Calculate(curve, prev_sl_point.s(), cur_point.s(),
                                   level, total_level) +                         // 当前点的轨迹代价加上前一个点的最小代价
        prev_dp_node.min_cost;

    cur_node->UpdateCost(&prev_dp_node, curve, cost);                            // 然后更新当前节点的代价
  }

  // try to connect the current point with the first point directly
  if (level >= 2) {                                                              // 如果不是第一个点， 就要进行链接
    const float init_dl = init_frenet_frame_point_.dl();                         // 起点的dl和ddl
    const float init_ddl = init_frenet_frame_point_.ddl();
    QuinticPolynomialCurve1d curve(init_sl_point_.l(), init_dl, init_ddl,        // 再构造一个5次多项式的曲线
                                   cur_node->sl_point.l(), 0.0, 0.0,
                                   cur_node->sl_point.s() - init_sl_point_.s());
    if (!IsValidCurve(curve)) {                                                  // 不合法的话就直接返回
      return;
    }
    const auto cost = trajectory_cost->Calculate(
        curve, init_sl_point_.s(), cur_node->sl_point.s(), level, total_level);  // 然后计算链接后的代价值
    cur_node->UpdateCost(front, curve, cost);                                    // 不断更新代价函数
  }
}
// 会得到一个level的采样点, 然后把一个level的采样点放到二维数组中
bool DPRoadGraph::SamplePathWaypoints(                                                       // 路网采样点， 这就是传说中的撒点吧?
    const common::TrajectoryPoint &init_point,                                               // 轨迹中的起点
    std::vector<std::vector<common::SLPoint>> *const points) {                               // 采样的结果放在二维数组中, 每个点是SL的坐标
  CHECK_NOTNULL(points);                                                                     // 做有效性检查

  const float kMinSampleDistance = 40.0;                                                     // 最小采样的距离为40
  const float total_length = std::fmin(                                                      // 获取采样的总长度
      init_sl_point_.s() + std::fmax(init_point.v() * 8.0, kMinSampleDistance),              // 8秒的路程
      reference_line_.Length());                                                             // 起点的距离加上未来8秒的路程, 最小采样距离40米, 车道中心参考线的长度, 三者取最大值
  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();                                  // VehicleConfigHelper这个单例类的作用就是获取车辆配置的参数, 还有一个重要的功能是计算最大转向半径
  const float half_adc_width = vehicle_config.vehicle_param().width() / 2.0;                 // 自动驾驶车辆的一半的宽度
  const size_t num_sample_per_level =                                                        // 不同的模式撒的点也不一样
      FLAGS_use_navigation_mode ? config_.navigator_sample_num_each_level()                  // 导航每个level上的点为3个
                                : config_.sample_points_num_each_level();                    // 每个level上的点为7个

  const bool has_sidepass = HasSidepass();                                                   // 看是否绕行, 从左边绕行还是从右边绕行

  constexpr float kSamplePointLookForwardTime = 4.0;                                         // 向前考虑的时间, 考虑前面的4秒钟的时间
  const float step_length =                                                                  // 步长
      common::math::Clamp(init_point.v() * kSamplePointLookForwardTime,                      // 当前速度乘以4秒, 反应时间为4秒?， 框型滤波
                          config_.step_length_min(), config_.step_length_max());             // 最大步长和最小步长之间, 最小步长为20m, 最大的步长为40m

  const float level_distance =
      (init_point.v() > FLAGS_max_stop_speed) ? step_length : step_length / 2.0;             // 最大的停止速度为0.2m/s, 如果要停止了就用二分之一步长
  float accumulated_s = init_sl_point_.s();                                                  // 走过的距离
  float prev_s = accumulated_s;                                                              // 前面的距离

  auto *status = GetPlanningStatus();                                                        // 调用命名空间中的函数, 会间接调用PlanningContext类里的同名函数
  if (!status->has_pull_over() && status->pull_over().in_pull_over()) {                      // 靠边停车的意思
    status->mutable_pull_over()->set_status(PullOverStatus::IN_OPERATION);                   // 正在靠边停车的过程中
    const auto &start_point = status->pull_over().start_point();                             // 获取靠边停车的起点
    SLPoint start_point_sl;
    if (!reference_line_.XYToSL(start_point, &start_point_sl)) {                             // 转换为sl坐标系
      AERROR << "Fail to change xy to sl.";
      return false;
    }

    if (init_sl_point_.s() > start_point_sl.s()) {                                           // 如果靠边停车的地方比轨迹的起点还要靠前
      const auto &stop_point = status->pull_over().stop_point();                             // 获取靠边停车的终点
      SLPoint stop_point_sl;
      if (!reference_line_.XYToSL(stop_point, &stop_point_sl)) {                             // 并转换为sl坐标系中
        AERROR << "Fail to change xy to sl.";
        return false;
      }
      std::vector<common::SLPoint> level_points(1, stop_point_sl);                           // 一个sl的点
      points->emplace_back(level_points);                                                    // 直接将终点返回
      return true;
    }
  }
                                                                                             // 在正常的轨迹上进行撒点
  for (std::size_t i = 0; accumulated_s < total_length; ++i) {                               // 开始迭代， total_length至少为40米
    accumulated_s += level_distance;                                                         // 一个步长，在20米到40米之间
    if (accumulated_s + level_distance / 2.0 > total_length) {                               // 到了终点, 有可能一般的时候就找到了步长
      accumulated_s = total_length;
    }
    const float s = std::fmin(accumulated_s, total_length);                                  // 找个小的, 步长和总长度中最小的
    constexpr float kMinAllowedSampleStep = 1.0;                                             // 采样的最小距离为1m
    if (std::fabs(s - prev_s) < kMinAllowedSampleStep) {                                     // 距离太小就跳过
      continue;
    }
    prev_s = s;                                                                              // 保存前一段的s(路程长度)

    double left_width = 0.0;                                                                 // 中心参考车道线的左右两边的宽度
    double right_width = 0.0;
    reference_line_.GetLaneWidth(s, &left_width, &right_width);                              // 通过给定的一个s点，获取车道的宽度, 左边和右边的宽度

    constexpr float kBoundaryBuff = 0.20;                                                    // 边界的buff, 为0.20
    const float eff_right_width = right_width - half_adc_width - kBoundaryBuff;              // 右边有效的宽度
    const float eff_left_width = left_width - half_adc_width - kBoundaryBuff;                // 左边有效的宽度， 这里有一个偏差， 应该会更偏右

    // the heuristic shift of L for lane change scenarios
    const double delta_dl = 1.2 / 20.0;                                                      // 启发式的参数delta, 1.2/20的具体含义是什么啊(dl方向的增量)?
    const double kChangeLaneDeltaL = common::math::Clamp(
        level_distance * (std::fabs(init_frenet_frame_point_.dl()) + delta_dl),              // 横向距离至少要为1.2米以上才能进行变道，(可以变道的横向距离)
        1.2, 3.5);   // kChangeLaneDeltaL必须在[1.2, 3.5]之间

    float kDefaultUnitL = kChangeLaneDeltaL / (num_sample_per_level - 1);                    // l默认的单位, num_sample_per_level在巡航模式中是3个点, 在std planning模式中是7个点
    if (reference_line_info_.IsChangeLanePath() &&
        !reference_line_info_.IsSafeToChangeLane()) {                                        // 如果已经确定变道，但是变道不安全
      kDefaultUnitL = 1.0;                                                                   // 将默认的l方向的采样值设置为1米
    }
    const float sample_l_range = kDefaultUnitL * (num_sample_per_level - 1);                 // num_sample_per_level为7
    float sample_right_boundary = -eff_right_width;                                          // 右边为什么是减， 针对中心参考线来说是左正右负
    float sample_left_boundary = eff_left_width;                                             // 左边为什么是加

    const float kLargeDeviationL = 1.75;                                                     // L 大的偏差为1.75
    if (reference_line_info_.IsChangeLanePath() ||                                           // 如果确定可以变道, 并且起点的横向的距离l大于1.75米
        std::fabs(init_sl_point_.l()) > kLargeDeviationL) {
      sample_right_boundary = std::fmin(-eff_right_width, init_sl_point_.l());               // 右boundary的大小， 获得边框最远的点
      sample_left_boundary = std::fmax(eff_left_width, init_sl_point_.l());                  // 左boundary的大小

      if (init_sl_point_.l() > eff_left_width) {                                             // l的值太大
        sample_right_boundary = std::fmax(sample_right_boundary,
                                          init_sl_point_.l() - sample_l_range);              // 重新计算右边采样的宽度
      }
      if (init_sl_point_.l() < eff_right_width) {
        sample_left_boundary = std::fmin(sample_left_boundary,                               // 重新计算左边采样的宽度
                                         init_sl_point_.l() + sample_l_range);
      }
    }

    std::vector<float> sample_l;                                                             // l的采样点放到一个vector里面
    if (reference_line_info_.IsChangeLanePath() &&
        !reference_line_info_.IsSafeToChangeLane()) {                                        // 是不安全的变道
      sample_l.push_back(reference_line_info_.OffsetToOtherReferenceLine());                 // 与另一条参考中心线的偏差
    } else if (has_sidepass) {                                                               // 如果可以进行绕行操作
      // currently only left nudge is supported. Need road hard boundary for                 // 只支持了左边绕行, 需要道路强边框两边绕行的话
      // both sides
      switch (sidepass_.type()) {
        case ObjectSidePass::LEFT: {
          sample_l.push_back(eff_left_width + config_.sidepass_distance());                  // 只会执行这条语句, 有效距离会加上一个绕行的距离
          break;
        }
        case ObjectSidePass::RIGHT: {
          sample_l.push_back(-eff_right_width - config_.sidepass_distance());                // 如果是从左边绕行
          break;
        }
        default:
          break;
      }
    } else {                                                                                 // 否则开始将
      common::util::uniform_slice(sample_right_boundary, sample_left_boundary,               // 等分右边到左边距离的点
                                  num_sample_per_level - 1, &sample_l);                      // 一个level上的点保存在sample_l中
    }
    std::vector<common::SLPoint> level_points;                                               // sl坐标组成的一个level的点                                 
    planning_internal::SampleLayerDebug sample_layer_debug;                                  // 采样层的debug
    for (size_t j = 0; j < sample_l.size(); ++j) {                                           // 迭代采样的l
      common::SLPoint sl = common::util::MakeSLPoint(s, sample_l[j]);                        // s和一个采样的l 组成一个sl坐标
      sample_layer_debug.add_sl_point()->CopyFrom(sl);                                       // 放到debug的对象中
      level_points.push_back(std::move(sl));                                                 // 并放到一个level_points数组中
    }
    if (!reference_line_info_.IsChangeLanePath() && has_sidepass) {                          // 这个是一个比较矛盾的东西
      auto sl_zero = common::util::MakeSLPoint(s, 0.0);
      sample_layer_debug.add_sl_point()->CopyFrom(sl_zero);
      level_points.push_back(std::move(sl_zero));                                            // 直接是参考中心线上
    }

    if (!level_points.empty()) {                                                             // 一个level上的已经有点的存在
      planning_debug_->mutable_planning_data()
          ->mutable_dp_poly_graph()                                                          // 将采样层的debug信息添加到planning的debug信息中
          ->add_sample_layer()
          ->CopyFrom(sample_layer_debug);
      points->emplace_back(level_points);                                                    // 然后把一个level的采样点放到二维数组中
    }
  }
  return true;
}

bool DPRoadGraph::CalculateFrenetPoint(                                               // 路网中计算一个frenet的点
    const common::TrajectoryPoint &traj_point,
    common::FrenetFramePoint *const frenet_frame_point) {
  common::SLPoint sl_point;                                                           // 除了转换为sl坐标以外
  if (!reference_line_.XYToSL(
          {traj_point.path_point().x(), traj_point.path_point().y()},                 // 转换为sl坐标系下的坐标
          &sl_point)) {
    return false;
  }
  frenet_frame_point->set_s(sl_point.s());
  frenet_frame_point->set_l(sl_point.l());

  const float theta = traj_point.path_point().theta();                                // 还会计算theta值
  const float kappa = traj_point.path_point().kappa();                                // 曲率值
  const float l = frenet_frame_point->l();

  ReferencePoint ref_point;                                                           // 获取道路中心参考线
  ref_point = reference_line_.GetReferencePoint(frenet_frame_point->s());             // 通过一个距离s, 获得一个参考线上的一点

  const float theta_ref = ref_point.heading();                                        // 获取道路中心参考线的航向角
  const float kappa_ref = ref_point.kappa();                                          // 曲率
  const float dkappa_ref = ref_point.dkappa();                                        // 曲率的导数

  const float dl = CartesianFrenetConverter::CalculateLateralDerivative(              // 获取横向的一次导数
      theta_ref, theta, l, kappa_ref);                                                // 笛卡尔坐标系转换为frenet坐标系的成员函数, 计算侧轴l的导数
  const float ddl =                                                                   // l的二次导数
      CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(
          theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point->set_dl(dl);                                                     // 复制给一个frenet坐标系中的一个frame点
  frenet_frame_point->set_ddl(ddl);                                                   // 设置l的二次微分
  return true;
}

bool DPRoadGraph::IsValidCurve(const QuinticPolynomialCurve1d &curve) const {         // 判断一个曲线是否合法
  constexpr float kMaxLateralDistance = 20.0;                                         // 横向的最大距离设置为20米
  for (float s = 0.0; s < curve.ParamLength(); s += 2.0) {                            // 2米迭代一次
    const float l = curve.Evaluate(0, s);                                             // 通过曲线中的s, 获取一个l
    if (std::fabs(l) > kMaxLateralDistance) {                                         // l特别长就会不合法， 5次多项式的曲线不合法
      return false;
    }
  }
  return true;
}

void DPRoadGraph::GetCurveCost(TrajectoryCost trajectory_cost,
                               const QuinticPolynomialCurve1d &curve,
                               const float start_s, const float end_s,
                               const uint32_t curr_level,
                               const uint32_t total_level,
                               ComparableCost *cost) {                               // 计算轨迹的代价
  *cost =
      trajectory_cost.Calculate(curve, start_s, end_s, curr_level, total_level);
}

bool DPRoadGraph::HasSidepass() {                                                    // 看是否可以绕行
  const auto &path_decision = reference_line_info_.path_decision();                  // 获得道路中心线的决策者
  for (const auto &obstacle : path_decision.path_obstacles().Items()) {              // 看该道路上的障碍物, 不断地迭代
    if (obstacle->LateralDecision().has_sidepass()) {                                // 如果有旁边绕行的, 就记录下来
      sidepass_ = obstacle->LateralDecision().sidepass();                            // 就是看从左边绕行还是从右边绕行
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
