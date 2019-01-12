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
 * @file path_data.cc
 **/

#include "modules/planning/common/path/path_data.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::SLPoint;                                                         // 一个sl坐标点, 其实就只有(sl的信息)
using apollo::common::math::CartesianFrenetConverter;                                  // 笛卡尔坐标系和frenet坐标系的转换器(这是一个类)
using apollo::common::math::Vec2d;                                                     // 二维的向量

bool PathData::SetDiscretizedPath(const DiscretizedPath &path) {                       // 通过一个离散的path来设置path data
  if (reference_line_ == nullptr) {                                                    // 容错检查
    AERROR << "Should NOT set discretized path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  discretized_path_ = path;      // 离散的坐标是在笛卡尔坐标系下, 规划算法是在SL坐标系下计算
  if (!XYToSL(discretized_path_, &frenet_path_)) {                                     // 把离散的xy坐标系中path映射到frenet坐标系中
    AERROR << "Fail to transfer discretized path to frenet path.";
    return false;
  }
  DCHECK_EQ(discretized_path_.NumOfPoints(), frenet_path_.points().size());            // 两个坐标系中的离散点的个数应该相等
  path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));       // 如果相等就成对的放到path data的历史数据中
  return true;
}

bool PathData::SetFrenetPath(const FrenetFramePath &frenet_path) {                     // 通过frenet坐标系中的数据, 设置path data的数据
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set frenet path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  frenet_path_ = frenet_path;    // 将frenet坐标系转换为xy的笛卡尔坐标系
  if (!SLToXY(frenet_path_, &discretized_path_)) {
    AERROR << "Fail to transfer frenet path to discretized path.";
    return false;
  }
  DCHECK_EQ(discretized_path_.NumOfPoints(), frenet_path_.points().size());
  path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));
  return true;
}

const DiscretizedPath &PathData::discretized_path() const {                            // 返回xy坐标系中path
  return discretized_path_;
}

bool PathData::Empty() const {                                                         // 离散的path和frenet坐标系中的path都没有元素就为空
  return discretized_path_.NumOfPoints() == 0 &&
         frenet_path_.NumOfPoints() == 0;
}

std::list<std::pair<DiscretizedPath, FrenetFramePath>>                                 // 返回历史数据
    &PathData::path_data_history() {
  return path_data_history_;
}

const FrenetFramePath &PathData::frenet_frame_path() const {                           // 返回frenet坐标系中的数据
  return frenet_path_;
}

void PathData::SetReferenceLine(const ReferenceLine *reference_line) {                 // 通过一个车道中心线设置一个车道中心线
  Clear();                                                                             // 再深度拷贝的时候清理数据
  reference_line_ = reference_line;                                                    // 然后将指针赋值
}

bool PathData::GetPathPointWithPathS(                                                  // 通过s(距离)点插值或获取一个path的点
    const double s, common::PathPoint *const path_point) const {
  *path_point = discretized_path_.Evaluate(s);                                         // 调用离散path的方法(这里的path point是三维空间中的一个点)
  return true;
}

bool PathData::GetPathPointWithRefS(const double ref_s,
                                    common::PathPoint *const path_point) const {
  DCHECK_NOTNULL(reference_line_);                                                     // 通过一个中心参考线上的s(路程) 获取对应的一个path point(路径点, 三维空间中的一个点)
  DCHECK_NOTNULL(path_point);
  DCHECK_EQ(discretized_path_.path_points().size(),                                    // 做容错处理
            frenet_path_.points().size());
  if (ref_s < 0) {
    AERROR << "ref_s[" << ref_s << "] should be > 0";
    return false;
  }
  if (ref_s > frenet_path_.points().back().s()) {                                       // 参考线的点必须在path的长度范围内
    AERROR << "ref_s is larger than the length of frenet_path_ length ["
           << frenet_path_.points().back().s() << "].";
    return false;
  }

  uint32_t index = 0;                                                                   // 索引值
  const double kDistanceEpsilon = 1e-3;                                                 // 最小的距离常数
  for (uint32_t i = 0; i + 1 < frenet_path_.points().size(); ++i) {                     // 迭代frenet坐标中path的所有点
    if (fabs(ref_s - frenet_path_.points().at(i).s()) < kDistanceEpsilon) {             // 两个特别近的话, 就直接返回第i个点就好
      path_point->CopyFrom(discretized_path_.path_points().at(i));
      return true;
    }
    if (frenet_path_.points().at(i).s() < ref_s &&                                      // 找到离ref_s最近的两个点
        ref_s <= frenet_path_.points().at(i + 1).s()) {
      index = i;
      break;
    }
  }
  double r = (ref_s - frenet_path_.points().at(index).s()) /                            // 然后进行线性插值(这里的r应该是ratio的意思)
             (frenet_path_.points().at(index + 1).s() -
              frenet_path_.points().at(index).s());

  const double discretized_path_s =                                                     // 把要插的值映射到离散(x,y)坐标系的path中
      discretized_path_.path_points().at(index).s() +
      r * (discretized_path_.path_points().at(index + 1).s() -
           discretized_path_.path_points().at(index).s());
  path_point->CopyFrom(discretized_path_.Evaluate(discretized_path_s));                 // 通过path的s再找一个合适的path point(路径点)

  return true;
}

void PathData::Clear() {
  discretized_path_.Clear();                                                         // 将xy坐标系中的path清空
  frenet_path_.Clear();                                                              // 将frenet坐标系中的path清空
  reference_line_ = nullptr;                                                         // 参考线的指针设置为空指针
}

std::string PathData::DebugString() const {                                             // debug的信息
  const auto &path_points = discretized_path_.path_points();                            // 获取所有的path点
  const auto limit =
      std::min(path_points.size(),
               static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));              // FLAGS_trajectory_point_num_for_debug设置为10

  return apollo::common::util::StrCat(
      "[\n",
      apollo::common::util::PrintDebugStringIter(
          path_points.begin(), path_points.begin() + limit, ",\n"),
      "]\n");
}
// 将转换函数封装成了成员函数
bool PathData::SLToXY(const FrenetFramePath &frenet_path,                           // 将frenet坐标系中的path映射到离散(x,y)的坐标系中
                      DiscretizedPath *const discretized_path) {
  DCHECK_NOTNULL(discretized_path);
  std::vector<common::PathPoint> path_points;
  for (const common::FrenetFramePoint &frenet_point : frenet_path.points()) {  // 遍历所有frenet的点
    common::SLPoint sl_point;
    common::math::Vec2d cartesian_point;
    sl_point.set_s(frenet_point.s());
    sl_point.set_l(frenet_point.l());
    if (!reference_line_->SLToXY(sl_point, &cartesian_point)) {
      AERROR << "Fail to convert sl point to xy point";
      return false;
    }
    ReferencePoint ref_point =
        reference_line_->GetReferencePoint(frenet_point.s());
    double theta = CartesianFrenetConverter::CalculateTheta(
        ref_point.heading(), ref_point.kappa(), frenet_point.l(),
        frenet_point.dl());
    double kappa = CartesianFrenetConverter::CalculateKappa(
        ref_point.kappa(), ref_point.dkappa(), frenet_point.l(),
        frenet_point.dl(), frenet_point.ddl());

    common::PathPoint path_point = common::util::MakePathPoint(
        cartesian_point.x(), cartesian_point.y(), 0.0, theta, kappa, 0.0, 0.0);

    if (path_points.empty()) {
      path_point.set_s(0.0);
      path_point.set_dkappa(0.0);
    } else {
      common::math::Vec2d last(path_points.back().x(), path_points.back().y());
      common::math::Vec2d current(path_point.x(), path_point.y());
      double distance = (last - current).Length();
      path_point.set_s(path_points.back().s() + distance);
      path_point.set_dkappa((path_point.kappa() - path_points.back().kappa()) /
                            distance);
    }
    path_points.push_back(std::move(path_point));
  }
  *discretized_path = DiscretizedPath(std::move(path_points));

  return true;
}

bool PathData::XYToSL(const DiscretizedPath &discretized_path,                    // 同理, 这里是把离散(x,y)的path映射到frenet坐标系中
                      FrenetFramePath *const frenet_path) {
  CHECK_NOTNULL(frenet_path);
  CHECK_NOTNULL(reference_line_);
  std::vector<common::FrenetFramePoint> frenet_frame_points;
  const double max_len = reference_line_->Length();
  for (const auto &path_point : discretized_path.path_points()) {
    SLPoint sl_point;
    if (!reference_line_->XYToSL({path_point.x(), path_point.y()}, &sl_point)) {
      AERROR << "Fail to transfer cartesian point to frenet point.";
      return false;
    }
    common::FrenetFramePoint frenet_point;
    // NOTICE: does not set dl and ddl here. Add if needed.
    frenet_point.set_s(std::max(0.0, std::min(sl_point.s(), max_len)));
    frenet_point.set_l(sl_point.l());
    frenet_frame_points.push_back(std::move(frenet_point));
  }
  *frenet_path = FrenetFramePath(std::move(frenet_frame_points));
  return true;
}

}  // namespace planning
}  // namespace apollo
