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
 * @file path_data.h
 **/

#ifndef MODULES_PLANNING_COMMON_PATH_PATH_DATA_H_
#define MODULES_PLANNING_COMMON_PATH_PATH_DATA_H_

#include <list>
#include <string>
#include <utility>

#include "modules/planning/common/path/discretized_path.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class PathData {    // 解析地图的类
 public:
  PathData() = default;                                                              // 默认的构造函数

  bool SetDiscretizedPath(const DiscretizedPath &path);                              // 设置path 打他的离散的路径

  bool SetFrenetPath(const FrenetFramePath &frenet_path);                            // 设置frenet坐标系下的path

  void SetReferenceLine(const ReferenceLine *reference_line);                        // 设置中心参考线

  const DiscretizedPath &discretized_path() const;                                   // 获取离散的path(在xy坐标系中的轨迹), path point

  const FrenetFramePath &frenet_frame_path() const;                                  // 获取sl坐标系中的path路径

  bool GetPathPointWithPathS(const double s,                                         // 通过路程s获取一个path point(三维的坐标(x,y,z), 曲率)
                             common::PathPoint *const path_point) const;

  std::list<std::pair<DiscretizedPath, FrenetFramePath>> &path_data_history();       // 一个path data会存在两个坐标的信息, 而且这些信息是一一对应的关系

  /*
   * brief: this function will find the path_point in discretized_path whose
   * projection to reference line has s value closest to ref_s.
   */
  bool GetPathPointWithRefS(const double ref_s,                                      // 通过中心参考线中的一个距离点ref_s, 获得一个path point
                            common::PathPoint *const path_point) const;

  void Clear();                                                                      // 清空所有的path data

  bool Empty() const;                                                                // path data 是否为空

  std::string DebugString() const;                                                   // dedug需要的字符串信息

 private:
  /*
   * convert frenet path to cartesian path by reference line
   */
  bool SLToXY(const FrenetFramePath &frenet_path,                                    // ReferenceLine中会封装sl和xy坐标相互转换的函数(涉及到的是点)
              DiscretizedPath *const discretized_path);                              // 这里直接升级到了path的角度(是线的角度)
  bool XYToSL(const DiscretizedPath &discretized_path,                               // 一个path要映射到xy坐标和sl坐标中
              FrenetFramePath *const frenet_path);                                   // 坐标转换
  const ReferenceLine *reference_line_ = nullptr;                                    // 参考线, path data中会有包含一个道路中心线 reference line
  DiscretizedPath discretized_path_;                                                 // 离散的path, 就是在xyz坐标中的path 点组成
  FrenetFramePath frenet_path_;                                                      // frenet坐标中的path, 由sl坐标和l的一次,二次微分组成
  std::list<std::pair<DiscretizedPath, FrenetFramePath>> path_data_history_;         // 用单链表存过去的时间, 所有的path, 都会映射到两个坐标中
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_PATH_PATH_DATA_H_
