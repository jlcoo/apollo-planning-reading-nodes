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
 * @file reference_line.h
 **/

#ifndef MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_H_
#define MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_H_

#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/proto/map_geometry.pb.h"
#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {
// ReferenceLine作为的是list的元素, 怎么构造一个ReferenceLine兑现
class ReferenceLine {                                                                   // 道路参考线
 public:
  ReferenceLine() = default;                                                            // 默认的构造函数
  explicit ReferenceLine(const ReferenceLine& reference_line) = default;                // 禁止隐式转换的构造函数
  template <typename Iterator>
  explicit ReferenceLine(const Iterator begin, const Iterator end)
      : reference_points_(begin, end),
        map_path_(std::move(std::vector<hdmap::MapPathPoint>(begin, end))) {}           // 带模板的构造函数
  explicit ReferenceLine(const std::vector<ReferencePoint>& reference_points);          // 由参考线的点集构成一条道路中心线
  explicit ReferenceLine(const hdmap::Path& hdmap_path);                                // 通过高精地图的path进行构造reference line(中心参考线)
  // stitching 拼接
  /** Stitch current reference line with the other reference line
   * The stitching strategy is to use current reference points as much as
   * possible. The following two examples show two successful stitch cases.
   *
   * Example 1
   * this:   |--------A-----x-----B------|
   * other:                 |-----C------x--------D-------|
   * Result: |------A-----x-----B------x--------D-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part B and part C matches, we update current reference
   * line to A-B-D.
   *
   * Example 2
   * this:                  |-----A------x--------B-------|
   * other:  |--------C-----x-----D------|
   * Result: |--------C-----x-----A------x--------B-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part A and part D matches, we update current reference
   * line to C-A-B.
   *
   * @return false if these two reference line cannot be stitched
   */
  bool Stitch(const ReferenceLine& other);                                             // 合并两条reference line

  bool Shrink(const common::math::Vec2d& point, double look_backward,                  // 二维向量的点进行剪切
              double look_forward);

  const hdmap::Path& map_path() const;                                                 // 高精地图中的path
  const std::vector<ReferencePoint>& reference_points() const;                         // reference line相关的所有点集

  ReferencePoint GetReferencePoint(const double s) const;                              // 返回中心参考线上, s(路程)对应的一个点
  std::vector<ReferencePoint> GetReferencePoints(double start_s,
                                                 double end_s) const;

  std::size_t GetNearestReferenceIndex(const double s) const;                          // 返回最近一个点的索引值
  // ReferencePoint继承于MapPathPoint, MapPathPoint是继承于二维的向量common::math::Vec2d(里面还包含了一些航向角, std::vector<LaneWaypoint>, laneWaypoint的数组), 
  // LaneWaypoint就是一个数据结构, 里面包含一个sl的坐标, 和一个LaneInfo的常量指针
  ReferencePoint GetNearestReferencePoint(const common::math::Vec2d& xy) const;        // 返回一个最近的点, 输入是一个二维的向量, 输出是一个 reference point

  std::vector<hdmap::LaneSegment> GetLaneSegments(const double start_s,                // 获取lane的片段的集合(多少米为一个片段呢?)
                                                  const double end_s) const;

  ReferencePoint GetNearestReferencePoint(const double s) const;                       // 获取最近的参考点

  ReferencePoint GetReferencePoint(const double x, const double y) const;              // 将一个x,y坐标转换为一个参考点

  bool GetApproximateSLBoundary(const common::math::Box2d& box,                        // 获取近似的sl坐标系的边界
                                const double start_s, const double end_s,
                                SLBoundary* const sl_boundary) const;
  bool GetSLBoundary(const common::math::Box2d& box,                                   // 讲一个二维的box, 添加一个sl坐标系下的边框
                     SLBoundary* const sl_boundary) const;
  bool GetSLBoundary(const hdmap::Polygon& polygon,
                     SLBoundary* const sl_boundary) const;

  bool SLToXY(const common::SLPoint& sl_point,                                         // 在ReferenceLine类中封装了一个sl坐标和xy坐标相互转换的函数
              common::math::Vec2d* const xy_point) const;
  bool XYToSL(const common::math::Vec2d& xy_point,
              common::SLPoint* const sl_point) const;
  template <class XYPoint>
  bool XYToSL(const XYPoint& xy, common::SLPoint* const sl_point) const {
    return XYToSL(common::math::Vec2d(xy.x(), xy.y()), sl_point);
  }
  // lane车道
  bool GetLaneWidth(const double s, double* const lane_left_width,                     // 通过距离s, lane左边宽度, lane右边宽度获得对应的lane
                    double* const lane_right_width) const;
  // road路面
  bool GetRoadWidth(const double s, double* const road_left_width,                     // 获取路面的宽度
                    double* const road_right_width) const;

  void GetLaneFromS(const double s,                                                    // 获得多条lane(车道)
                    std::vector<hdmap::LaneInfoConstPtr>* lanes) const;

  /**
   * @brief: check if a box/point is on lane along reference line
   */
  bool IsOnLane(const common::SLPoint& sl_point) const;                               // 检查一个sl的坐标是否在车道线上
  bool IsOnLane(const common::math::Vec2d& vec2d_point) const;                        // 检查一个二维的向量点是否在车道上
  template <class XYPoint>
  bool IsOnLane(const XYPoint& xy) const {                                            // 检查一个xy坐标系中的点是否子啊车道上
    return IsOnLane(common::math::Vec2d(xy.x(), xy.y()));
  }
  bool IsOnLane(const SLBoundary& sl_boundary) const;                                 // 检查一个sl的边框是否在车道上

  /**
   * @brief: check if a box/point is on road
   *         (not on sideways/medians) along reference line
   */
  bool IsOnRoad(const common::SLPoint& sl_point) const;                               // 是否在路上
  bool IsOnRoad(const common::math::Vec2d& vec2d_point) const;
  bool IsOnRoad(const SLBoundary& sl_boundary) const;

  /**
   * @brief Check if a box is blocking the road surface. The crieria（标准） is to check  
   * whether the remaining space on the road surface is larger than the provided
   * gap space.
   * @param boxed the provided box
   * @param gap check the gap of the space
   * @return true if the box blocks the road.
   */
  bool IsBlockRoad(const common::math::Box2d& box2d, double gap) const;               // 是否一个box堵塞在路上

  /**
   * @brief check if any part of the box has overlap with the road.
   */
  bool HasOverlap(const common::math::Box2d& box) const;                             // 检查一个box是否与道路有重叠

  double Length() const { return map_path_.length(); }                               // 返回道路的长度

  std::string DebugString() const;                                                   // debug的字符串信息

  double GetSpeedLimitFromS(const double s) const;                                   // 获得s点的速度限制

  void AddSpeedLimit(const hdmap::SpeedControl& speed_control);                      // 添加限速信息
  void AddSpeedLimit(double start_s, double end_s, double speed_limit);              // 在一定范围内添加速度信息

  uint32_t GetPriority() const { return priority_; }                                 // 每条reference line都有一个优先级
  void SetPriority(uint32_t priority) { priority_ = priority; }

 private:
  /**
   * @brief Linearly interpolate p0 and p1 by s0 and s1.
   * The input has to satisfy condition: s0 <= s <= s1
   * p0 and p1 must have lane_waypoint.
   * Note: it requires p0 and p1 are on the same lane, adjacent lanes, or
   * parallel neighboring lanes. Otherwise the interpolated result may not
   * valid.
   * @param p0 the first anchor point for interpolation.
   * @param s0 the longitutial distance (s) of p0 on current reference line.
   * s0 <= s && s0 <= s1
   * @param p1 the second anchor point for interpolation
   * @param s1 the longitutial distance (s) of p1 on current reference line.
   * s1
   * @param s identifies the the middle point that is going to be
   * interpolated.
   * s >= s0 && s <= s1
   * @return The interpolated ReferencePoint.
   */
  static ReferencePoint Interpolate(const ReferencePoint& p0, const double s0,       // 线性插值
                                    const ReferencePoint& p1, const double s1,
                                    const double s);
  ReferencePoint InterpolateWithMatchedIndex(                                        // 插值匹配指数
      const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
      const double s1, const hdmap::InterpolatedIndex& index) const;

  static double FindMinDistancePoint(const ReferencePoint& p0, const double s0,      // 查找最小距离点
                                     const ReferencePoint& p1, const double s1,
                                     const double x, const double y);

 private:
  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;  // unit m/s
    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
  };
  /**
   * This speed limit overrides the lane speed limit
   **/
  std::vector<SpeedLimit> speed_limit_;           // 速度限制
  std::vector<ReferencePoint> reference_points_;  // ReferencePoint继承于MapPathPoint
  hdmap::Path map_path_;                          // 高精地图的Path类
  uint32_t priority_ = 0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_H_
