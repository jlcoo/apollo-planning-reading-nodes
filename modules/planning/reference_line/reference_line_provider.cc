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
 * @brief Implementation of the class ReferenceLineProvider.
 */

#include "modules/planning/reference_line/reference_line_provider.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <utility>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/routing/common/routing_gflags.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;    // 车配置助手
using apollo::common::VehicleState;           // 车的状态
using apollo::common::adapter::AdapterManager;// 适配器的管理者
using apollo::common::math::Vec2d;            // 二维的向量
using apollo::common::time::Clock;            // 时间时钟
using apollo::hdmap::HDMapUtil;               // 高精地图相关的配置项
using apollo::hdmap::LaneWaypoint;            // 车道路网中的点
using apollo::hdmap::MapPathPoint;            // path中的点, 这个是一个类(里面有航向角, vector<LaneWaypoint>(路网点的数组))
using apollo::hdmap::PncMap;                  // pnc的地图
using apollo::hdmap::RouteSegments;           // RouteSegments(routing的片段)

ReferenceLineProvider::~ReferenceLineProvider() {                                    // 参考线提供者的析构函数
  if (thread_ && thread_->joinable()) {                                              // 会等待所有线程都死去
    thread_->join();   // 等死
  }
}
// 在车道线的构造函数中, 生产参考线(车道线)Spline2dSolver
ReferenceLineProvider::ReferenceLineProvider(const hdmap::HDMap *base_map) {         // 通过高精地图的base地图进行构造
  if (!FLAGS_use_navigation_mode) {
    pnc_map_.reset(new hdmap::PncMap(base_map));                                     // 如果使用了导航模式, 就会新建一个base地图然后初始化pnc的地图
  }
  CHECK(common::util::GetProtoFromFile(FLAGS_smoother_config_filename,               // 平滑器的配置文件为modules/planning/conf/qp_spline_smoother_config.pb.txt
                                       &smoother_config_))                           // 获取protocol buf中的内容进行参数配置
      << "Failed to load smoother config file "
      << FLAGS_smoother_config_filename;                                             // 失败的话就把错误信息打到log日志文件中
  if (smoother_config_.has_qp_spline()) {      // 看是用的那个平滑器                    // 配置文件中设置的是qp(二次规划器)
    smoother_.reset(new QpSplineReferenceLineSmoother(smoother_config_));  // QP平滑  // 创建一个新的对象
  } else if (smoother_config_.has_spiral()) { // 螺旋线
    smoother_.reset(new SpiralReferenceLineSmoother(smoother_config_));
  } else if (smoother_config_.has_cos_theta()) { // cos(theta)
    smoother_.reset(new CosThetaReferenceLineSmoother(smoother_config_));
  } else {
    CHECK(false) << "unknown smoother config "
                 << smoother_config_.DebugString();
  }
  is_initialized_ = true;
}  // namespace planning

bool ReferenceLineProvider::UpdateRoutingResponse(                                    // 更新routing获得的值 
    const routing::RoutingResponse &routing) {
  std::lock_guard<std::mutex> routing_lock(routing_mutex_);                           // 给routing加锁                 
  routing_ = routing;                                                                 // 直接复制                                         
  has_routing_ = true;                                                                // 设置标志位                              
  return true;
}

std::vector<routing::LaneWaypoint>
ReferenceLineProvider::FutureRouteWaypoints() {                                       // 这是同步的一种机制
  std::lock_guard<std::mutex> lock(pnc_map_mutex_);                                   // 因为加锁了, 所以是线程安全的
  return pnc_map_->FutureRouteWaypoints();   // 将来路径要行驶的点
}

void ReferenceLineProvider::UpdateVehicleState(                                       // 更新车辆的状态                          
    const VehicleState &vehicle_state) {
  std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
  vehicle_state_ = vehicle_state;
}

bool ReferenceLineProvider::Start() {
  if (FLAGS_use_navigation_mode) {
    return true;
  }
  if (!is_initialized_) {  // 已经被初始化完了
    AERROR << "ReferenceLineProvider has NOT been initiated.";
    return false;
  }  // 创建一个线程GenerateThread, 并且把this(ReferenceLineProvider对象)指针传递给它
  if (FLAGS_enable_reference_line_provider_thread) {
    thread_.reset(
        new std::thread(&ReferenceLineProvider::GenerateThread, this));              // 多线程就要产生多个线程, 所以要调用GenerateThread函数
  }
  return true;
}

void ReferenceLineProvider::Stop() {
  is_stop_ = true;                                                                   // 停止位必须设置为true                            
  if (FLAGS_enable_reference_line_provider_thread && thread_ &&                      // 在停止函数中等死
      thread_->joinable()) {
    thread_->join();
  }
}

void ReferenceLineProvider::UpdateReferenceLine(                                     // 更新中心参考线                      
    const std::list<ReferenceLine> &reference_lines,         // 参考车道线
    const std::list<hdmap::RouteSegments> &route_segments) { // 高精地图片段
  if (reference_lines.size() != route_segments.size()) {                             // 中心参考线的个数要和route_segments的数量完全相等
    AERROR << "The calculated reference line size(" << reference_lines.size()
           << ") and route_segments size(" << route_segments.size()
           << ") are different";
    return;
  }  // 临界区加锁
  std::lock_guard<std::mutex> lock(reference_lines_mutex_);                          // 一上来就要先加一个锁                
  if (reference_lines_.size() != reference_lines.size()) {                           // 如果内部不相等就更新内部的参考线的条数
    reference_lines_ = reference_lines;
    route_segments_ = route_segments;

  } else {                                                                           // 如果相等就进行迭代                         
    auto segment_iter = route_segments.begin();
    auto internal_iter = reference_lines_.begin();
    auto internal_segment_iter = route_segments_.begin();
    for (auto iter = reference_lines.begin(); iter != reference_lines.end();
         ++iter, ++segment_iter, ++internal_iter, ++internal_segment_iter) {
      if (iter->reference_points().empty()) {
        *internal_iter = *iter;
        *internal_segment_iter = *segment_iter;
        continue;
      }
      if (common::util::SamePointXY(
              iter->reference_points().front(),
              internal_iter->reference_points().front()) &&
          common::util::SamePointXY(iter->reference_points().back(),
                                    internal_iter->reference_points().back()) &&
          std::fabs(iter->Length() - internal_iter->Length()) <
              common::math::kMathEpsilon) {
        continue;
      }
      *internal_iter = *iter;                                                        // 进行一个点一个点地更新
      *internal_segment_iter = *segment_iter;
    }
  }
  // update history
  reference_line_history_.push(reference_lines_);                                    // 放到内部的单链表中
  route_segments_history_.push(route_segments_);
  // 关键字 constexpr 是C++11中引入的关键字，声明为constexpr类型的变量，编译器会验证该变量的值是否是一个常量表达式
  // constexpr 函数是在使用需要它的代码时，可以在编译时计算其返回值的函数
  // const 和 constexpr 变量之间的主要区别在于：const 变量的初始化可以延迟到运行时，而 constexpr 变量必须在编译时进行初始化。
  constexpr int kMaxHistoryNum = 3;      // 缓存3个点?
  if (reference_line_history_.size() > kMaxHistoryNum) {                             // 只保存3组不同的中心参考线
    reference_line_history_.pop();       // 删除掉一个顶部的
    route_segments_history_.pop();
  }
}
                                                                                     // reference line的提供者, 用多线程来进行处理
void ReferenceLineProvider::GenerateThread() {
  while (!is_stop_) {                          // 进入一个while死循环
    std::this_thread::yield();     // 当被抢占时, 主动让出时间片
    // boost线程中的yield方法：可以将本线程的CPU时间片放弃，并允许其他线程运行
    // yield方法其实就是::Sleep(0)
    constexpr int32_t kSleepTime = 50;  // milliseconds, 50ms睡一次                   // 20Hz一次
    std::this_thread::sleep_for(
        std::chrono::duration<double, std::milli>(kSleepTime));
    double start_time = Clock::NowInSeconds();
    if (!has_routing_) {    // routing完了才能启动 sample
      AERROR << "Routing is not ready.";
      continue;
    }
    std::list<ReferenceLine> reference_lines;   // 参考线放到的是一个单链表中
    std::list<hdmap::RouteSegments> segments;   // 将高精地图分段也放到一个链表中
    if (!CreateReferenceLine(&reference_lines, &segments)) {  // 生成一个参考路径线
      AERROR << "Fail to get reference line";
      continue;
    }                                                                                // 不断地创建reference line, 然后不断地更新
    UpdateReferenceLine(reference_lines, segments); // 更新参考路径线
    double end_time = Clock::NowInSeconds();    // 得到上面算法的最后时间
    std::lock_guard<std::mutex> lock(reference_lines_mutex_); // guard加锁, 不用自己释放互斥量，在析构函数中自动释放
    last_calculation_time_ = end_time - start_time; // 得到一个时间戳
  }
}

double ReferenceLineProvider::LastTimeDelay() {                                      // 上次时间上的延迟
  if (FLAGS_enable_reference_line_provider_thread &&                                 // 如果是多线程就要加锁
      !FLAGS_use_navigation_mode) {
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    return last_calculation_time_;
  } else {
    return last_calculation_time_;
  }
}

bool ReferenceLineProvider::GetReferenceLines(                                       // 获取参考线
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {                                     // 获取routing的片段                           
  CHECK_NOTNULL(reference_lines);                                                    // 做容错检查              
  CHECK_NOTNULL(segments);

  if (FLAGS_use_navigation_mode) {                                                   // 使用导航模式
    double start_time = Clock::NowInSeconds();                                       // 开始的时间戳
    bool result = GetReferenceLinesFromRelativeMap(
        AdapterManager::GetRelativeMap()->GetLatestObserved(), reference_lines,      // 从相对地图中获取道路中心参考线
        segments);
    if (!result) {                                                                   // 获取失败的容错处理
      AERROR << "Failed to get reference line from relative map";
    }
    double end_time = Clock::NowInSeconds();                                         // 获取中心参考线的时间戳
    last_calculation_time_ = end_time - start_time;
    return result;
  }

  if (FLAGS_enable_reference_line_provider_thread) {                                 // 如果使用的是多线程
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);                        // 加锁
    if (!reference_lines_.empty()) {                                                 // 中心参考线存在
      reference_lines->assign(reference_lines_.begin(), reference_lines_.end());     // 从内部的成员函数中获取中心参考
      segments->assign(route_segments_.begin(), route_segments_.end());              // 从内部的route的片段中复制
      return true;
    } else {                                                                         // 参考线还没有准备好
      AWARN << "Reference line is NOT ready.";
      if (reference_line_history_.empty()) {                                         // 历史的数据中没有数据
        return false;
      }
      reference_lines->assign(reference_line_history_.back().begin(),                // 如果历史数据中还有数据的话, 就从历史
                              reference_line_history_.back().end());
      segments->assign(route_segments_history_.back().begin(),                       // 从历史数据中的route Segments
                       route_segments_history_.back().end());
    }
  } else {
    double start_time = Clock::NowInSeconds();                                       // 开始时间
    if (!CreateReferenceLine(reference_lines, segments)) {                           // 创建中心参考线, route segment的
      AERROR << "Failed to create reference line";
      return false;
    }
    UpdateReferenceLine(*reference_lines, *segments);                                // 更新车道中心参考线
    double end_time = Clock::NowInSeconds();                                         // 结束的时间点
    last_calculation_time_ = end_time - start_time;                                  // 获得创建中心参考线的时间
  }
  return true;
}

void ReferenceLineProvider::PrioritzeChangeLane(                                     // 优先考虑变道
    std::list<hdmap::RouteSegments> *route_segments) {   // list是一个双链表           // RouteSegments, 
  CHECK_NOTNULL(route_segments);                                                     // 检查指针是否为空
  auto iter = route_segments->begin();                                               // 跌代
  while (iter != route_segments->end()) {   // 就是不断地拼接上route_segments
    if (!iter->IsOnSegment()) {                                                      // 不在segment上才进行评接
      route_segments->splice(route_segments->begin(), *route_segments, iter);        // 链表的拼接上
      break;
    }
    ++iter;
  }
}

bool ReferenceLineProvider::GetReferenceLinesFromRelativeMap(                        // 从相对地图中获取中心参考线 
    const relative_map::MapMsg &relative_map,                                        // 输入为相对地图relative map
    std::list<ReferenceLine> *reference_lines,                                       // 输出为中心参考线reference line和route segments
    std::list<hdmap::RouteSegments> *segments) {  // list<vector<LaneSegment>>   
  DCHECK_GE(relative_map.navigation_path_size(), 0);                                 // 容错检查(有效性检查)
  DCHECK_NOTNULL(reference_lines);
  DCHECK_NOTNULL(segments);

  if (relative_map.navigation_path().empty()) {                                      // 从当前的相对地图中获取导航path
    AERROR << "There isn't any navigation path in current relative map.";
    return false;
  }

  auto *hdmap = HDMapUtil::BaseMapPtr();                                             // 获取高精地图的指针
  if (!hdmap) {
    AERROR << "hdmap is null";
    return false;
  }

  // 1.get adc current lane info ,such as lane_id,lane_priority,neighbor lanes
  std::unordered_set<std::string> navigation_lane_ids;                               // 获取当前车道的信息, lane id地址, lane的优先级, 相邻的lanes
  for (const auto &path_pair : relative_map.navigation_path()) {                     // 迭代相对地图中的path点对
    const auto lane_id = path_pair.first;                                            // 第一项是id地址
    navigation_lane_ids.insert(lane_id);
  }
  if (navigation_lane_ids.empty()) {
    AERROR << "navigation path ids is empty";
    return false;
  }
  // get curent adc lane info by vehicle state
  common::VehicleState vehicle_state =                                               // 获取自动驾驶车辆的状态
      common::VehicleStateProvider::instance()->vehicle_state();
  hdmap::LaneWaypoint adc_lane_way_point;                                            // 获取最近的一点
  if (!GetNearestWayPointFromNavigationPath(vehicle_state, navigation_lane_ids,
                                            &adc_lane_way_point)) {
    return false;
  }
  const std::string adc_lane_id = adc_lane_way_point.lane->id().id();                // 自动驾驶车辆所在的车道id号
  auto adc_navigation_path = relative_map.navigation_path().find(adc_lane_id);       // 找到导航地图中的path
  if (adc_navigation_path == relative_map.navigation_path().end()) {                 // 其实相当于导航的path和相对地图中的path建立联系
    AERROR << "adc lane cannot be found in relative_map.navigation_path";
    return false;
  }
  const uint32_t adc_lane_priority =                                                 // 获取优先级
      adc_navigation_path->second.path_priority();
  // get adc left neighbor lanes
  std::vector<std::string> left_neighbor_lane_ids;                                   // 获取相邻车道的id地址值
  auto left_lane_ptr = adc_lane_way_point.lane;
  while (left_lane_ptr != nullptr &&
         left_lane_ptr->lane().left_neighbor_forward_lane_id_size() > 0) {
    auto neighbor_lane_id =
        left_lane_ptr->lane().left_neighbor_forward_lane_id(0);
    left_neighbor_lane_ids.emplace_back(neighbor_lane_id.id());
    left_lane_ptr = hdmap->GetLaneById(neighbor_lane_id);
  }
  ADEBUG << adc_lane_id
         << " left neighbor size : " << left_neighbor_lane_ids.size();               // 左边车道有多少条
  for (const auto &neighbor : left_neighbor_lane_ids) {
    ADEBUG << adc_lane_id << " left neighbor : " << neighbor;
  }
  // get adc right neighbor lanes
  std::vector<std::string> right_neighbor_lane_ids;                                  // 获取右边的车道信息
  auto right_lane_ptr = adc_lane_way_point.lane;
  while (right_lane_ptr != nullptr &&
         right_lane_ptr->lane().right_neighbor_forward_lane_id_size() > 0) {
    auto neighbor_lane_id =
        right_lane_ptr->lane().right_neighbor_forward_lane_id(0);
    right_neighbor_lane_ids.emplace_back(neighbor_lane_id.id());
    right_lane_ptr = hdmap->GetLaneById(neighbor_lane_id);
  }
  ADEBUG << adc_lane_id
         << " right neighbor size : " << right_neighbor_lane_ids.size();
  for (const auto &neighbor : right_neighbor_lane_ids) {
    ADEBUG << adc_lane_id << " right neighbor : " << neighbor;
  }
  // 2.get the higher priority lane info list which priority higher
  // than current lane and get the highest one as the target lane
  using LaneIdPair = std::pair<std::string, uint32_t>;                               // 车道的点对, 名字和32位无符号整型
  std::vector<LaneIdPair> high_priority_lane_pairs;
  ADEBUG << "relative_map.navigation_path_size = "
         << relative_map.navigation_path_size();                                     // 迭代过程中选择优先级最高的车道 
  for (const auto &path_pair : relative_map.navigation_path()) {
    const auto lane_id = path_pair.first;
    const uint32_t priority = path_pair.second.path_priority();
    ADEBUG << "lane_id = " << lane_id << " priority = " << priority
           << " adc_lane_id = " << adc_lane_id
           << " adc_lane_priority = " << adc_lane_priority;
    // the smaller the number, the higher the priority
    if (adc_lane_id != lane_id && priority < adc_lane_priority) {                    // 车道(lane)的优先级是值越小, 优先级越大
      high_priority_lane_pairs.emplace_back(lane_id, priority);
    }
  }
  // get the target lane
  bool is_lane_change_needed = false;                                                // 找到最高优先级的车道, 确定是否需要换到
  LaneIdPair target_lane_pair;
  if (!high_priority_lane_pairs.empty()) {
    std::sort(high_priority_lane_pairs.begin(), high_priority_lane_pairs.end(),      // 按优先级进行排序
              [](const LaneIdPair &left, const LaneIdPair &right) {
                return left.second < right.second;
              });
    ADEBUG << "need to change lane";
    // the higheast priority lane as the target naviagion lane
    target_lane_pair = high_priority_lane_pairs.front();                             // 将高优先级作为目标点对
    is_lane_change_needed = true;
  }
  // 3.get current lane's the neareast neighbor lane to the target lane
  // and make sure it position is left or right on the current lane
  routing::ChangeLaneType lane_change_type = routing::FORWARD;                       // 获取到目标车道最近的车道, 而且确定从左边走还是从右边走
  std::string neareast_neighbor_lane_id;                                             // 到目标车道的最近车道
  if (is_lane_change_needed) {                                                       // 只有会变道的情况下才会执行下面的逻辑
    // target on the left of adc
    if (left_neighbor_lane_ids.end() !=                                              // 如果从左边变道
        std::find(left_neighbor_lane_ids.begin(), left_neighbor_lane_ids.end(),
                  target_lane_pair.first)) {
      // take the id of the first adjacent lane on the left of adc as
      // the neareast_neighbor_lane_id
      neareast_neighbor_lane_id =                                                    // 就获取第一个左相邻车道的id地址
          adc_lane_way_point.lane->lane().left_neighbor_forward_lane_id(0).id();
    } else if (right_neighbor_lane_ids.end() !=                                      // 目标车道在右边就获取第一个右车道相邻的ID地址
               std::find(right_neighbor_lane_ids.begin(),
                         right_neighbor_lane_ids.end(),
                         target_lane_pair.first)) {
      // target lane on the right of adc
      // take the id  of the first adjacent lane on the right of adc as
      // the neareast_neighbor_lane_id
      neareast_neighbor_lane_id = adc_lane_way_point.lane->lane()
                                      .right_neighbor_forward_lane_id(0)
                                      .id();
    }
  }

  for (const auto &path_pair : relative_map.navigation_path()) {                     // 迭代相对地图中的所有点对
    const auto &lane_id = path_pair.first;                                           // 获取点对中的车道id信息
    const auto &path_points = path_pair.second.path().path_point();                  // 获取path中的采样点
    auto lane_ptr = hdmap->GetLaneById(hdmap::MakeMapId(lane_id));                   // 获取车道(lane)对应的指针
    RouteSegments segment;                                                           // routing相关的segment片段
    segment.emplace_back(lane_ptr, 0.0, lane_ptr->total_length());                   // 设置route segment的各种参数
    segment.SetCanExit(true);
    segment.SetId(lane_id);
    segment.SetNextAction(routing::FORWARD);
    segment.SetStopForDestination(false);
    segment.SetPreviousAction(routing::FORWARD);

    if (is_lane_change_needed) {                                                     // 判断是否需要变道, 如果需要变道的话, 就会改变车道的id地址
      if (lane_id == neareast_neighbor_lane_id) {
        ADEBUG << "adc lane_id = " << adc_lane_id
               << " neareast_neighbor_lane_id = " << lane_id;
        segment.SetIsNeighborSegment(true);
        segment.SetPreviousAction(lane_change_type);
      } else if (lane_id == adc_lane_id) {                                           // 如果已经变道完了, 那么就不用改变id地址
        segment.SetIsOnSegment(true);
        segment.SetNextAction(lane_change_type);
      }
    }

    segments->emplace_back(segment);                                                 // 把segment(route相关的信息)放到双链表中
    std::vector<ReferencePoint> ref_points;                                          // 这是将path中的所有点转换为reference中的所有点呀
    for (const auto &path_point : path_points) {                                     // 在迭代所有的path中的点
      ref_points.emplace_back(
          MapPathPoint{Vec2d{path_point.x(), path_point.y()},                        // 将所有点的path中点转换为reference中的点
                       path_point.theta(),
                       LaneWaypoint(lane_ptr, path_point.s())},
          path_point.kappa(), path_point.dkappa());
    }
    reference_lines->emplace_back(ref_points.begin(), ref_points.end());             // 再将所有的reference中点构造成一个reference line(车道中心参考线)
    reference_lines->back().SetPriority(path_pair.second.path_priority());           // 设置车道参考中心线的优先级
  }
  return !segments->empty();                                                         // routing后的信息(判断segment是否为空)
}

bool ReferenceLineProvider::GetNearestWayPointFromNavigationPath(                    // 从导航的path中获取最近的way point
    const common::VehicleState &state,                                               // 输入是自动驾驶车量的状态
    const std::unordered_set<std::string> &navigation_lane_ids,                      // 导航的lane的集合
    hdmap::LaneWaypoint *waypoint) {                                                 // 输出为一个LaneWaypoint点的指针
  const double kMaxDistance = 10.0;
  waypoint->lane = nullptr;                                                          // LaneWaypoint中包含一个sl的坐标和LaneInfoConstPtr(lane info的常量指针)
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  auto point = common::util::MakePointENU(state.x(), state.y(), state.z());          // 将车辆的状态转换为x,y,z坐标系中的一点
  if (std::isnan(point.x()) || std::isnan(point.y())) {                              // 做容错检查
    AERROR << "vehicle state is invalid";
    return false;
  }
  auto *hdmap = HDMapUtil::BaseMapPtr();
  if (!hdmap) {
    AERROR << "hdmap is null";
    return false;
  }

  // get all adc direction lanes from map in kMaxDistance range                      // kMaxDistance等于10
  // by vehicle point in map // 通过地图中的车辆点从10米范围内的地图中获取所有adc(自动驾驶车辆)方向车道
  const int status = hdmap->GetLanesWithHeading(                                     // 获取航向角
      point, kMaxDistance, state.heading(), M_PI / 2.0, &lanes);
  if (status < 0) {
    AERROR << "failed to get lane from point " << point.ShortDebugString();
    return false;
  }

  // get lanes that exist in both map and navigation paths as vallid lanes
  std::vector<hdmap::LaneInfoConstPtr> valid_lanes;                                 // map(高精度地图)和导航地图都存在的话就是合法的lane(车道)
  std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
               [&](hdmap::LaneInfoConstPtr ptr) {
                 return navigation_lane_ids.count(ptr->lane().id().id()) > 0;
               });
  if (valid_lanes.empty()) {
    AERROR << "no valid lane found within " << kMaxDistance
           << " meters with heading " << state.heading();
    return false;
  }

  // get nearest lane wayponints for current adc position                          // 获取最近的车道中的点(需要考虑在那个车道中, 然后转换为sl坐标)
  double min_distance = std::numeric_limits<double>::infinity(); 
  for (const auto &lane : valid_lanes) {                                           // 从合法的id中找一个点
    // project adc point to lane to check if it is out of lane range
    double s = 0.0;
    double l = 0.0;
    if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
      continue;
    }
    constexpr double kEpsilon = 1e-6;
    if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
      continue;
    }

    // get the neareast distance between adc point and lane                        // 获取自动驾驶车辆到lane中的最近的距离
    double distance = 0.0;
    common::PointENU map_point =
        lane->GetNearestPoint({point.x(), point.y()}, &distance);
    // reord the near distance lane                                                // 记录下最近的距离
    if (distance < min_distance) {
      double s = 0.0;
      double l = 0.0;
      if (!lane->GetProjection({map_point.x(), map_point.y()}, &s, &l)) {
        AERROR << "failed to get projection for map_point "
               << map_point.DebugString();
        continue;
      }
      min_distance = distance;
      waypoint->lane = lane;                                                       // way point 保存的就是最近的距离
      waypoint->s = s;
    }
  }

  if (waypoint->lane == nullptr) {
    AERROR << "failed to find nearest point " << point.ShortDebugString();
  }
  return waypoint->lane != nullptr;
}

bool ReferenceLineProvider::CreateRouteSegments(
    const common::VehicleState &vehicle_state,    // 车的状态, 在vehicle_state.proto文件中被定义, 利用的是protocolbuf的功能
    std::list<hdmap::RouteSegments> *segments) {  // 高精地图中RouteSegments的大小, segments是一个list<vector<LaneSegment>>对象
  {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);  // 加锁然后获取routeSegments, 锁住pnc_map
    // pnc_map_是std::unique_ptr<hdmap::PncMap>的智能指针
    if (!pnc_map_->GetRouteSegments(vehicle_state, segments)) {   // segments为GetRouteSegments的输出
      AERROR << "Failed to extract segments from routing";
      return false;
    }
  }

  if (FLAGS_prioritize_change_lane) {     // 被设置为false
    PrioritzeChangeLane(segments);
  }
  return !segments->empty();    // 就是看看list里面是不是为空的
}

bool ReferenceLineProvider::CreateReferenceLine(
    std::list<ReferenceLine> *reference_lines,     // 参考线的列表的指针
    std::list<hdmap::RouteSegments> *segments) {   // 高精地图的列表RouteSegments是一个vector
  CHECK_NOTNULL(reference_lines);   // CHECK_NOTNULL是在logging.h中定义的宏
  CHECK_NOTNULL(segments);
  // 这应该是protobuf的语法知识
  common::VehicleState vehicle_state;   // 这个是一个类， 在vehicle_state.proto中被定义声明
  {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    vehicle_state = vehicle_state_;
  }

  routing::RoutingResponse routing;
  {
    std::lock_guard<std::mutex> lock(routing_mutex_);
    routing = routing_;
  }
  bool is_new_routing = false;   // 每次把标志位致为false
  {
    // Update routing in pnc_map
    if (pnc_map_->IsNewRouting(routing)) {  // 判断是否是新的路径
      is_new_routing = true;  // 如果地图是最新的
      if (!pnc_map_->UpdateRoutingResponse(routing)) {  // 看看pnc地图的更新路径的响应函数是否调用成功
        AERROR << "Failed to update routing in pnc map";    // pnc_map_就会更新里面的内容
        return false;   // RoutingResponse是从rounting返回路网的信息
      }
    }
  }
  // 根据routing的结果创建一个routeSegments, segments是传入的形参
  if (!CreateRouteSegments(vehicle_state, segments)) {   // 这个函数特别重要， ******这里应该有dp算法调用创建routing吧?*******
    AERROR << "Failed to create reference line from routing";   // 就是从pnc_地图中获得了segments
    return false;
  }
  if (is_new_routing || !FLAGS_enable_reference_line_stitching) {    // 合并reference_line的标志为没有打开
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      if (!SmoothRouteSegment(*iter, &reference_lines->back())) {
        AERROR << "Failed to create reference line from route segments";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        ++iter;
      }
    }
    return true;
  } else {  // stitching reference line，  enable_reference_line_stitching被设置为true
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();   // 放个空的进去
      if (!ExtendReferenceLine(vehicle_state, &(*iter),
                               &reference_lines->back())) {   // 再填充进去
        AERROR << "Failed to extend reference line";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        ++iter;   //不断迭代更新下一个segments
      }
    }
  }
  return true;
}
//  扩展参考线, stitching参考线
bool ReferenceLineProvider::ExtendReferenceLine(const VehicleState &state,
                                                RouteSegments *segments,
                                                ReferenceLine *reference_line) {
  RouteSegments segment_properties;      // routeSegments
  segment_properties.SetProperties(*segments);
  auto prev_segment = route_segments_.begin();   // 之前的segments
  auto prev_ref = reference_lines_.begin();
  while (prev_segment != route_segments_.end()) {   // 扩展道路线
    if (prev_segment->IsConnectedSegment(*segments)) {    // 如果连接上了就退出
      break;
    }
    ++prev_segment;
    ++prev_ref;
  }
  if (prev_segment == route_segments_.end()) {
    if (!route_segments_.empty() && segments->IsOnSegment()) {
      AWARN << "Current route segment is not connected with previous route "
               "segment";
    }
    return SmoothRouteSegment(*segments, reference_line);
  }
  common::SLPoint sl_point;
  Vec2d vec2d(state.x(), state.y());
  LaneWaypoint waypoint;    // 将上一个segments投影到路网上, sl_point和waypoint
  if (!prev_segment->GetProjection(vec2d, &sl_point, &waypoint)) {
    AWARN << "Vehicle current point: " << vec2d.DebugString()   // 前面没有参考线就直接返回，然后做smooth
          << " not on previous reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  const double prev_segment_length = RouteSegments::Length(*prev_segment);   // 获取之前的routeSegments的长度
  const double remain_s = prev_segment_length - sl_point.s();                // 剩下的部分
  const double look_forward_required_distance =
      PncMap::LookForwardDistance(state.linear_velocity());                  // 前面行驶的距离
  if (remain_s > look_forward_required_distance) {                           // 剩下的s距离更长
    *segments = *prev_segment;
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Reference line remain " << remain_s
           << ", which is more than required " << look_forward_required_distance
           << " and no need to extend";
    return true;
  }
  double future_start_s =
      std::max(sl_point.s(), prev_segment_length -
                                 FLAGS_reference_line_stitch_overlap_distance);  // 新的起点
  double future_end_s =
      prev_segment_length + FLAGS_look_forward_extend_distance;
  RouteSegments shifted_segments;
  std::unique_lock<std::mutex> lock(pnc_map_mutex_);
  if (!pnc_map_->ExtendSegments(*prev_segment, future_start_s, future_end_s,
                                &shifted_segments)) {       // 扩展segments, 保存在shifted_segments(转移段中)
    lock.unlock();
    AERROR << "Failed to shift route segments forward";
    return SmoothRouteSegment(*segments, reference_line);
  }
  lock.unlock();
  if (prev_segment->IsWaypointOnSegment(shifted_segments.LastWaypoint())) {
    *segments = *prev_segment;
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Could not further extend reference line";
    return true;
  }
  hdmap::Path path;   // 新的path
  hdmap::PncMap::CreatePathFromLaneSegments(shifted_segments, &path);   // 创建新的path
  ReferenceLine new_ref(path);
  if (!SmoothPrefixedReferenceLine(*prev_ref, new_ref, reference_line)) {
    AWARN << "Failed to smooth forward shifted reference line";            // 无法平滑向前移动的参考线
    return SmoothRouteSegment(*segments, reference_line);
  }
  if (!reference_line->Stitch(*prev_ref)) {
    AWARN << "Failed to stitch reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  if (!shifted_segments.Stitch(*prev_segment)) {
    AWARN << "Failed to stitch route segments";
    return SmoothRouteSegment(*segments, reference_line);
  }
  *segments = shifted_segments;
  segments->SetProperties(segment_properties);
  common::SLPoint sl;
  if (!reference_line->XYToSL(vec2d, &sl)) {
    AWARN << "Failed to project point: " << vec2d.DebugString()
          << " to stitched reference line";
  }
  if (sl.s() > FLAGS_look_backward_distance * 1.5) {         // 扩大1.5倍
    ADEBUG << "reference line back side is " << sl.s()
           << ", shrink reference line: origin lenght: "
           << reference_line->Length();
    if (!reference_line->Shrink(vec2d, FLAGS_look_backward_distance,
                                std::numeric_limits<double>::infinity())) {
      AWARN << "Failed to shrink reference line";
    }
    if (!segments->Shrink(vec2d, FLAGS_look_backward_distance,
                          std::numeric_limits<double>::infinity())) {
      AWARN << "Failed to shrink route segment";
    }
  }
  return true;
}

bool ReferenceLineProvider::IsReferenceLineSmoothValid(                          // 中心参考线是否是足够的平滑, 检查10米处的平滑就可以了? 
    const ReferenceLine &raw, const ReferenceLine &smoothed) const {
  constexpr double kReferenceLineDiffCheckStep = 10.0;      // 参考线检查
  for (double s = 0.0; s < smoothed.Length();
       s += kReferenceLineDiffCheckStep) {
    auto xy_new = smoothed.GetReferencePoint(s);                                 // 通过s获取一个xy坐标系中的一点

    common::SLPoint sl_new;
    if (!raw.XYToSL(xy_new, &sl_new)) {                                          // 然后马上转换为一个sl坐标中的一点
      AERROR << "Fail to change xy point on smoothed reference line to sl "
                "point respect to raw reference line.";
      return false;
    }

    const double diff = std::fabs(sl_new.l());    // 变差不大
    if (diff > FLAGS_smoothed_reference_line_max_diff) {                         // FLAGS_smoothed_reference_line_max_diff设置为5米, 最大位置的偏差不能超过5米
      AERROR << "Fail to provide reference line because too large diff "
                "between smoothed and raw reference lines. diff: "
             << diff;
      return false;
    }
  }
  return true;
}
// 生成一个点, 怎么生成的锚点呢?
AnchorPoint ReferenceLineProvider::GetAnchorPoint(
    const ReferenceLine &reference_line, double s) const {                      // 通过一个中心参考线和一个积累的路程s, 获取一个锚点(AnchorPoint, 主要包含PathPoint和sl方向的bound)
  AnchorPoint anchor;   // 作为输出
  anchor.longitudinal_bound = smoother_config_.longitudinal_boundary_bound();   // 纵向的边界
  auto ref_point = reference_line.GetReferencePoint(s);                         // 获得参考点(中心车道线中的一个点), 是一个插值的点
  if (ref_point.lane_waypoints().empty()) {                                     // 如果这个点存在就直接利用中心车道上的一点构造一个锚点(anchor point)
    anchor.path_point = ref_point.ToPathPoint(s);
    anchor.lateral_bound = smoother_config_.lateral_boundary_bound();
    return anchor;
  }                                                                             // 否则需要重新计算
  const double adc_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width();                 // autonomy driving car的宽度
  const double adc_half_width = adc_width / 2.0;                                // 宽度的一半
  const Vec2d left_vec =                                                        // 航向角逆时针旋转90度, 就是左边的单位向量
      Vec2d::CreateUnitVec2d(ref_point.heading() + M_PI / 2.0);                 // 朝向+二分之pi干嘛呢?
  auto waypoint = ref_point.lane_waypoints().front();                           // 车道中最新的采样点
  // shift to center
  double left_width = 0.0;
  double right_width = 0.0;
  waypoint.lane->GetWidth(waypoint.s, &left_width, &right_width);               // 获取自动驾驶车辆(中心点)离左边的宽度和离右边的宽度
  double total_width = left_width + right_width;                                // 左边宽度加上右边的宽度
  // only need to track left side width shift                                   // 让车到车道的正中间
  double shifted_left_width = total_width / 2.0;                                // 只跟踪了左侧宽度的偏移

  // shift to left (or right) on wide lanes        // 在宽的lanes上左转或者右转
  if (smoother_config_.wide_lane_threshold_factor() > 0 &&      // 宽车道阈值系数
      total_width > adc_width * smoother_config_.wide_lane_threshold_factor()) { // 平滑的参数设置, 能够做平滑
    if (smoother_config_.driving_side() == ReferenceLineSmootherConfig::RIGHT) {   // 右转
      shifted_left_width =                                                       // 平滑得太靠右, 就向左偏
          adc_half_width +
          adc_width * smoother_config_.wide_lane_shift_remain_factor();
    } else {
      shifted_left_width = std::fmax(
          adc_half_width,
          total_width -
              (adc_half_width +
               adc_width * smoother_config_.wide_lane_shift_remain_factor()));
    }
  }

  // shift away from curb boundary
  auto left_type = hdmap::LeftBoundaryType(waypoint);                           // 远离路边界
  if (left_type == hdmap::LaneBoundaryType::CURB) {
    shifted_left_width += smoother_config_.curb_shift();
  }
  auto right_type = hdmap::RightBoundaryType(waypoint);
  if (right_type == hdmap::LaneBoundaryType::CURB) {   // cub是控制的意思, curb是抑制的意思
    shifted_left_width -= smoother_config_.curb_shift();
  }

  ref_point += left_vec * (left_width - shifted_left_width);  // 向量乘以左边的宽度   // 先计算中心参考线的一点
  auto shifted_right_width = total_width - shifted_left_width;  // 右边的宽度
  anchor.path_point = ref_point.ToPathPoint(s);                                    // 在把这一点转换为锚点的path point
  double effective_width = std::min(shifted_left_width, shifted_right_width) -
                           adc_half_width - FLAGS_reference_line_lateral_buffer;   // 有效的宽度.
  anchor.lateral_bound =
      std::max(smoother_config_.lateral_boundary_bound(), effective_width);        // 侧向的边界
  // 锚点, 主要设置了path_point和lateral_bound
  return anchor;
}
// 这些锚点用来干什么呢?? AnchorPoint是用来做reference line的平滑的
void ReferenceLineProvider::GetAnchorPoints(
    const ReferenceLine &reference_line,             // 原始的参考线, 作为输入
    std::vector<AnchorPoint> *anchor_points) const { // anchor_points作为输出
  CHECK_NOTNULL(anchor_points);  // 最大常量间隔
  const double interval = smoother_config_.max_constraint_interval();   // 如果是默认的话, 是5.0
  int num_of_anchors =
      std::max(2, static_cast<int>(reference_line.Length() / interval + 0.5));   // 至少要两个锚点, 加0.5是向上取整
  std::vector<double> anchor_s;    // 
  common::util::uniform_slice(0.0, reference_line.Length(), num_of_anchors - 1,
                              &anchor_s);   // 将reference_line均匀切片到anchor_s中, 这是一个std::vector<double>的数组
  for (const double s : anchor_s) {         // 切片完了再一个一个地生成锚点
    AnchorPoint anchor = GetAnchorPoint(reference_line, s);    // 生成一个锚点
    anchor_points->emplace_back(anchor);                       // 获得锚点后放到anchor_points中
  }
  anchor_points->front().longitudinal_bound = 1e-6;            // 纵向的bound
  anchor_points->front().lateral_bound = 1e-6;                 // 侧向的bound
  anchor_points->front().enforced = true;
  anchor_points->back().longitudinal_bound = 1e-6;
  anchor_points->back().lateral_bound = 1e-6;
  anchor_points->back().enforced = true;
}

bool ReferenceLineProvider::SmoothRouteSegment(const RouteSegments &segments,          // 平滑一些route segment
                                               ReferenceLine *reference_line) {
  hdmap::Path path;    // 保存了生成的path， path是高精度的path
  hdmap::PncMap::CreatePathFromLaneSegments(segments, &path);                          // 从segments中创建一个path
  // 通过path生成的referenceline, 然后用reference_line做平滑的referenceline
  return SmoothReferenceLine(ReferenceLine(path), reference_line);                     // 平滑一条车道上的参考中心线(reference line)
}
// Smooth前的预处理
bool ReferenceLineProvider::SmoothPrefixedReferenceLine(
    const ReferenceLine &prefix_ref, const ReferenceLine &raw_ref,                     // 平滑前向的参考线
    ReferenceLine *reference_line) {
  if (!FLAGS_enable_smooth_reference_line) {                                           // FLAGS_enable_smooth_reference_line设置为true
    *reference_line = raw_ref;                                                         // 不做平滑的话就直接返回原始的参考线
    return true;
  }
  // generate anchor points:
  std::vector<AnchorPoint> anchor_points;
  GetAnchorPoints(raw_ref, &anchor_points);                                            // 从原始的中心参考线中获取锚点
  // modify anchor points based on prefix_ref
  for (auto &point : anchor_points) {
    common::SLPoint sl_point;  // sl坐标点
    Vec2d xy{point.path_point.x(), point.path_point.y()};                              // 将path的xy左边点转换为sl坐标系中的点
    if (!prefix_ref.XYToSL(xy, &sl_point)) {
      continue;
    }
    if (sl_point.s() < 0 || sl_point.s() > prefix_ref.Length()) {
      continue;
    }   // 获得最近的参考点
    auto prefix_ref_point = prefix_ref.GetNearestReferencePoint(sl_point.s());         // 获得最近的参考线上的一点
    point.path_point.set_x(prefix_ref_point.x());
    point.path_point.set_y(prefix_ref_point.y());
    point.path_point.set_z(0.0);
    point.path_point.set_theta(prefix_ref_point.heading());
    point.longitudinal_bound = 1e-6;
    point.lateral_bound = 1e-6;
    point.enforced = true;
    break;
  }

  smoother_->SetAnchorPoints(anchor_points);                                          // 将锚点复制给平滑器
  if (!smoother_->Smooth(raw_ref, reference_line)) {    // 在这里调用Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    AERROR << "Failed to smooth prefixed reference line with anchor points";          // 平滑器无法求解
    return false;
  }
  if (!IsReferenceLineSmoothValid(raw_ref, *reference_line)) {                        // 平滑后的线段不合法
    AERROR << "The smoothed reference line error is too large";
    return false;
  }
  return true;
}
// 用原始的reference_line作平滑平滑处理ReferenceLine, 输出到第二个参数ReferenceLine *reference_line中
bool ReferenceLineProvider::SmoothReferenceLine(
    const ReferenceLine &raw_reference_line, ReferenceLine *reference_line) {
  if (!FLAGS_enable_smooth_reference_line) {     // 如果没有使能平滑reference_line这个选项, 就直接返回
    *reference_line = raw_reference_line;
    return true;
  }   // enable smooth the map reference line， 使能平滑map的参考线
  // generate anchor points:   产生一些锚点
  std::vector<AnchorPoint> anchor_points;                // AnchorPoint是一个结构体
  GetAnchorPoints(raw_reference_line, &anchor_points);   // 从原始的参考线中获得锚点, anchor_points作为输出
  smoother_->SetAnchorPoints(anchor_points);                                         // 先要设置平滑器的锚点, 才能进行平滑
  if (!smoother_->Smooth(raw_reference_line, reference_line)) {                      // 平滑后要检查平滑器是否能够求解, 平滑器时候完全合法
    AERROR << "Failed to smooth reference line with anchor points";
    return false;
  }
  if (!IsReferenceLineSmoothValid(raw_reference_line, *reference_line)) {
    AERROR << "The smoothed reference line error is too large";
    return false;
  }
  return true;
}
}  // namespace planning
}  // namespace apollo
