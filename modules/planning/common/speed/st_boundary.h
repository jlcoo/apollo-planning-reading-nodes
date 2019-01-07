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
 *   @file
 **/

#ifndef MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_BOUNDARY_H_
#define MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_BOUNDARY_H_

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/planning/proto/planning.pb.h"

#include "modules/common/math/box2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/speed/st_point.h"

namespace apollo {
namespace planning {

class StBoundary : public common::math::Polygon2d {                                // st坐标下的边框是继承2维的多边形
 public:
  StBoundary() = default;                                                          // 默认的构造函数

  explicit StBoundary(                                                             // 禁止隐式转换的构造函数
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs);                // ST坐标下的点对进行构造

  explicit StBoundary(const common::math::Box2d& box) = delete;                    // 通过一个二维的box进行构造一个st坐标系下的边框
  explicit StBoundary(std::vector<common::math::Vec2d> points) = delete;           // 通过一系列的点进行构造一个st坐标系下的边框

  ~StBoundary() = default;                                                         // 默认的析构函数

  bool IsEmpty() const { return lower_points_.empty(); }                           // 低点是否为空
  bool IsPointInBoundary(const STPoint& st_point) const;                           // 一个st坐标下的点是否在边界上

  STPoint BottomLeftPoint() const;                                                 // 左下角的点
  STPoint BottomRightPoint() const;                                                // 右下角的点

  StBoundary ExpandByS(const double s) const;                                      // 扩展s的边界
  StBoundary ExpandByT(const double t) const;                                      // 扩展t的边界

  // if you need to add boundary type, make sure you modify
  // GetUnblockSRange accordingly.
  enum class BoundaryType {                                                        // st边框的类型
    UNKNOWN,
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
    KEEP_CLEAR,
  };    // 不知道， 停止， 跟随， 让行， 超车， 保持清醒

  static std::string TypeName(BoundaryType type);                                  // 静态的类函数, 通过枚举类型返回一个字符串

  BoundaryType boundary_type() const;                                              // 返回st坐标系下的边框
  const std::string& id() const;                                                   // 字符串的id
  double characteristic_length() const;                                            // 特征的长度

  void SetId(const std::string& id);                                               // 设置st边框的ID
  void SetBoundaryType(const BoundaryType& boundary_type);                         // 设置st边框的类型
  void SetCharacteristicLength(const double characteristic_length);                // 设置st边框特征的长度

  bool GetUnblockSRange(const double curr_time, double* s_upper,                   // 获取非阻塞的范围(S的范围)
                        double* s_lower) const;

  bool GetBoundarySRange(const double curr_time, double* s_upper,                  // 获取st边框的范围(S的范围)
                         double* s_lower) const;

  double min_s() const;                                                            // 最小, 最大的s
  double min_t() const;                                                            // 最小, 最大的t
  double max_s() const;                                                            // 这四个点可以求得面积
  double max_t() const;

  double Area() const;

  std::vector<STPoint> upper_points() const { return upper_points_; }              // 上边界的点
  std::vector<STPoint> lower_points() const { return lower_points_; }              // 下边界的点

  static StBoundary GenerateStBoundary(                                            // 通过上下边界的点生成一个st的边框
      const std::vector<STPoint>& lower_points,
      const std::vector<STPoint>& upper_points);

  StBoundary CutOffByT(const double t) const;                                      // 切断T

 private:
  bool IsValid(                                                                    // 判断一些点对是否合法
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const;

  bool IsPointNear(const common::math::LineSegment2d& seg,                         // 一个点和一个线段是否在指定的范围内
                   const common::math::Vec2d& point, const double max_dist);

  FRIEND_TEST(StBoundaryTest, remove_redundant_points);
  void RemoveRedundantPoints(                                                      // 移除冗余的点对
      std::vector<std::pair<STPoint, STPoint>>* point_pairs);

  FRIEND_TEST(StBoundaryTest, get_index_range);
  bool GetIndexRange(const std::vector<STPoint>& points, const double t,           // 获取st点索引的范围
                     size_t* left, size_t* right) const;

 private:
  BoundaryType boundary_type_ = BoundaryType::UNKNOWN;                             // 初始化为不明障碍物

  std::vector<STPoint> upper_points_;                                              // 上边界的点
  std::vector<STPoint> lower_points_;                                              // 下边界的点

  double area_ = 0.0;                                                              // ST边框的面积

  std::string id_;                                                                 // st边框的id
  double characteristic_length_ = 1.0;                                             // 特征长度, 初始化为1.0
  double s_high_limit_ = 200.0;                                                    // 最远的路程为200米
  double min_s_ = std::numeric_limits<double>::max();                              // 初始化最小,最大的s和t
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_BOUNDARY_H_
