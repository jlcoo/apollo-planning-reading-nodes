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
 * @file reference_line_provider.h
 *
 * @brief Declaration of the class ReferenceLineProvider.
 */

#ifndef MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_
#define MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_

#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/util/factory.h"
#include "modules/common/util/util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/math/smoothing_spline/spline_2d_solver.h"
#include "modules/planning/reference_line/cos_theta_reference_line_smoother.h"
#include "modules/planning/reference_line/qp_spline_reference_line_smoother.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/spiral_reference_line_smoother.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class ReferenceLineProvider
 * @brief The class of ReferenceLineProvider.
 *        It provides smoothed reference line to planning.
 */    // 参考线的生成者
class ReferenceLineProvider {
 public:
  ReferenceLineProvider() = default;                                                      // 默认的构造函数
  explicit ReferenceLineProvider(const hdmap::HDMap* base_map);                           // 禁止隐式转换的构造函数

  /**
   * @brief Default destructor.
   */
  ~ReferenceLineProvider();                                                               // 默认的析构函数

  bool UpdateRoutingResponse(const routing::RoutingResponse& routing);                    // 更新routing的请求

  void UpdateVehicleState(const common::VehicleState& vehicle_state);                     // 更新车辆的状态

  bool Start();                                                                           // 开始

  void Stop();                                                                            // 结束

  bool GetReferenceLines(std::list<ReferenceLine>* reference_lines,                       // 获取中心参考线
                         std::list<hdmap::RouteSegments>* segments);                      // route的片段RouteSegments

  double LastTimeDelay();                                                                 // 上次时间的延迟

  std::vector<routing::LaneWaypoint> FutureRouteWaypoints();                              // 同步的获取lane上的路点

 private:
  /**
   * @brief Use PncMap to create reference line and the corresponding segments
   * based on routing and current position. This is a thread safe function.
   * @return true if !reference_lines.empty() && reference_lines.size() ==
   *                 segments.size();
   **/ // 基于routing和当前的位置(来自pnc的地图)来创建一个中心参考线和相应的片段(route segments)                              
  bool CreateReferenceLine(std::list<ReferenceLine>* reference_lines,
                           std::list<hdmap::RouteSegments>* segments);

  /**
   * @brief store the computed reference line. This function can avoid
   * unnecessary copy if the reference lines are the same.
   */  // 减少不必要的copy, 如果中心参考线一样的话
  void UpdateReferenceLine(
      const std::list<ReferenceLine>& reference_lines,
      const std::list<hdmap::RouteSegments>& route_segments);

  void GenerateThread();                                                                // 产生一个线程
  void IsValidReferenceLine();
  void PrioritzeChangeLane(std::list<hdmap::RouteSegments>* route_segments);            // 优先考虑变道的信息

  bool CreateRouteSegments(const common::VehicleState& vehicle_state,                   // 创建route的片段
                           std::list<hdmap::RouteSegments>* segments);

  bool IsReferenceLineSmoothValid(const ReferenceLine& raw,                             // 中心参考线是否足够平滑
                                  const ReferenceLine& smoothed) const;

  bool SmoothReferenceLine(const ReferenceLine& raw_reference_line,                     // 将原始的中心参考线平滑一下
                           ReferenceLine* reference_line);

  bool SmoothPrefixedReferenceLine(const ReferenceLine& prefix_ref,                     // 平滑中线参考线的前缀
                                   const ReferenceLine& raw_ref,
                                   ReferenceLine* reference_line);

  void GetAnchorPoints(const ReferenceLine& reference_line,                             // 获得中心参考线锚点
                       std::vector<AnchorPoint>* anchor_points) const;

  bool SmoothRouteSegment(const hdmap::RouteSegments& segments,                         // 平滑route的片段
                          ReferenceLine* reference_line);

  /**
   * @brief This function creates a smoothed forward reference line
   * based on the given segments.
   */
  // 把新的reference line, 拼接到旧的上面
  bool ExtendReferenceLine(const common::VehicleState& state,
                           hdmap::RouteSegments* segments,
                           ReferenceLine* reference_line);

  AnchorPoint GetAnchorPoint(const ReferenceLine& reference_line,                      // 获得锚点
                             double s) const;

  bool GetReferenceLinesFromRelativeMap(                                               // 从参考地图中获取参考线(reference line)
      const relative_map::MapMsg& relative_map,                                        // 定位地图(relative map)
      std::list<ReferenceLine>* reference_lines,                                       // 获取很多个中心参考线
      std::list<hdmap::RouteSegments>* segments);                                      // 获取route的片段

  /**
   * @brief This function get adc lane info from navigation path and map
   * by vehicle state.  LatticePlanner
   */  // 从导航path和地图中会获取自动驾驶车辆(adc)的车道信息
  bool GetNearestWayPointFromNavigationPath(
      const common::VehicleState& state,                                               // 先必须要获得车辆的状态
      const std::unordered_set<std::string>& navigation_lane_ids,                      // 用hash表中获取导航的车道index的值
      hdmap::LaneWaypoint* waypoint);                                                  // 返回的是lane way point(包含了sl的坐标和LaneInfoConstPtr的指针)

 private:
  bool is_initialized_ = false;                                                        // 中心参考线是否已经被初始化
  bool is_stop_ = false;                                                               // 是否已经停止了
  std::unique_ptr<std::thread> thread_;                                                // reference line provider使用的线程

  std::unique_ptr<ReferenceLineSmoother> smoother_;                                    // 平滑器()
  ReferenceLineSmootherConfig smoother_config_;   // 里面配置了靠左行驶还是靠右行驶, 最大约束间隔(默认是5), 纵向边界(默认是1米), 横向边界(lateral boundary bound默认是0.1), 总的点数(默认是500), curb_shift(默认是0.2), driving_side(默认是靠右行驶) 
  // 输出线的分辨率是0.02， smootherconfig配置使用qp_spline还是spiral还是cos_theta这三种不同的平滑器. 

  std::mutex pnc_map_mutex_;                                                           // 要改变pnc地图的信息, 必须进行加锁处理
  std::unique_ptr<hdmap::PncMap> pnc_map_;   // pnc的map到底是什么地图呢? 基于hdmap,增加了routing的一些逻辑

  std::mutex vehicle_state_mutex_;
  // common::VehicleState这个是一个类， 在vehicle_state.proto中被定义声明
  common::VehicleState vehicle_state_;  // 车辆的状态

  std::mutex routing_mutex_;
  routing::RoutingResponse routing_;   // routing::RoutingResponse应该也是通过protobuf定义的
  bool has_routing_ = false;
  // ReferenceLine::Stitch: 拼接两个refline,  检测这个点是否Join, 根据join的点，决定插入的位置
  // ReferencePoint：　组成ReferenceLine的基本单位
  // ReferenceLine::Shrink：　基于（look_backward,　当前点，look_forward） 找到有效的refline, 裁剪到外面的line, 并更新　map_path_

  std::mutex reference_lines_mutex_;                                                  // 参考线保护的中心线所利用的互斥量
  std::list<ReferenceLine> reference_lines_;                                          // 保存中心参考线的双链表
  std::list<hdmap::RouteSegments> route_segments_;                                    // 高精地图的route segment(routing利用的片段)
  double last_calculation_time_ = 0.0;                                                // 上次计算reference line的时间

  std::queue<std::list<ReferenceLine>> reference_line_history_;                       // 将reference line的历史数据放到队列中
  std::queue<std::list<hdmap::RouteSegments>> route_segments_history_;                // 将route segments的历史数据也放到队列中
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_
