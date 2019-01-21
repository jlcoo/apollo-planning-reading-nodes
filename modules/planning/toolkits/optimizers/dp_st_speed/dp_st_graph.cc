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
 * @file dp_st_graph.cc
 **/

#include "modules/planning/toolkits/optimizers/dp_st_speed/dp_st_graph.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/util/thread_pool.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;                                                           // 错误码
using apollo::common::SpeedPoint;                                                          // 速度点(路程s, 速度v, 加速度a， 时间t, 加速度的微分da)
using apollo::common::Status;                                                              // 模块的状态
using apollo::common::VehicleParam;                                                        // 车辆的参数
using apollo::common::math::Vec2d;                                                         // 二维的向量
using apollo::common::util::ThreadPool;                                                    // 线程池

namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();                             // 常数无穷大

bool CheckOverlapOnDpStGraph(const std::vector<const StBoundary*>& boundaries,             // st的边界
                             const StGraphPoint& p1, const StGraphPoint& p2) {             // 检查两个st的点是否有过重叠
  const common::math::LineSegment2d seg(p1.point(), p2.point());                           // 通过两个点构成一个线段, poit返回的是st坐标系下的一个点
  for (const auto* boundary : boundaries) {                                                // 迭代所有的boundary
    if (boundary->boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {               // keep clear就会继续
      continue;
    }
    if (boundary->HasOverlap(seg)) {                                                       // 如果线段和boundary有重叠就是有重叠
      return true;
    }
  }
  return false;                                                                            // 否则就是没有重叠
}
}  // namespace

DpStGraph::DpStGraph(const StGraphData& st_graph_data,                                     // st图中的数据
                     const DpStSpeedConfig& dp_config,                                     // st图中的配置参数
                     const std::vector<const PathObstacle*>& obstacles,                    // path中障碍物组成的数组
                     const common::TrajectoryPoint& init_point,                            // 起点
                     const SLBoundary& adc_sl_boundary)                                    // 自动驾驶的sl边框
    : st_graph_data_(st_graph_data),
      dp_st_speed_config_(dp_config),
      obstacles_(obstacles),
      init_point_(init_point),
      dp_st_cost_(dp_config, obstacles, init_point_),
      adc_sl_boundary_(adc_sl_boundary) {                                                  // 进行初始化列表进行初始化
  dp_st_speed_config_.set_total_path_length(                                               // path的总长度
      std::fmin(dp_st_speed_config_.total_path_length(),
                st_graph_data_.path_data_length()));
  unit_s_ = dp_st_speed_config_.total_path_length() /
            (dp_st_speed_config_.matrix_dimension_s() - 1);                                // s的分辨率
  unit_t_ = dp_st_speed_config_.total_time() /
            (dp_st_speed_config_.matrix_dimension_t() - 1);                                // t的分辨率
}

Status DpStGraph::Search(SpeedData* const speed_data) {                                    // 搜索dp图中的一个速度点
  constexpr float kBounadryEpsilon = 1e-2;                                                 // 无穷的边框
  for (const auto& boundary : st_graph_data_.st_boundaries()) {                            // 迭代st图中的边框
    if (boundary->boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {               // keep clear就是完全不用管的意思(美国交规)
      continue;
    }
    if (boundary->IsPointInBoundary({0.0, 0.0}) ||                                         // 原点在boundary上
        (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
         std::fabs(boundary->min_s()) < kBounadryEpsilon)) {                               // 并且时间t和路程s都很小
      std::vector<SpeedPoint> speed_profile;                                               // 速度曲线
      float t = 0.0;                                                                       // 定义一个时间t
      for (int i = 0; i < dp_st_speed_config_.matrix_dimension_t();
           ++i, t += unit_t_) {                                                            // 迭代时间维度, 大概是8秒的信息
        SpeedPoint speed_point;
        speed_point.set_s(0.0);                                                            // 速度点的路程s，为什么一直处于0
        speed_point.set_t(t);
        speed_profile.emplace_back(speed_point);
      }
      speed_data->set_speed_vector(speed_profile);                                         // 将速度的曲线放到速度相关的data中
      return Status::OK();
    }
  }

  if (st_graph_data_.st_boundaries().empty()) {                                            // st图中的boundary不存在就就是说明没有障碍物 
    ADEBUG << "No path obstacles, dp_st_graph output default speed profile.";
    std::vector<SpeedPoint> speed_profile;                                                 // 速度的曲线就是由一个一个的speed point组成
    float s = 0.0;                                                                         // s的值
    float t = 0.0;                                                                         // t的值
    for (int i = 0; i < dp_st_speed_config_.matrix_dimension_t() &&
                    i < dp_st_speed_config_.matrix_dimension_s();                          // 迭代时间和路程的所有点(一个时间t和一个路程s是一一对应的)
         ++i, t += unit_t_, s += unit_s_) {
      SpeedPoint speed_point;                                                              // 速度的点
      speed_point.set_s(s);                                                                // 设置速度点的s(路程)
      speed_point.set_t(t);                                                                // 设置时间t
      const float v_default = unit_s_ / unit_t_;                                           // 路程和时间就得到了速度
      speed_point.set_v(v_default);
      speed_point.set_a(0.0);                                                              // 将加速度设置为0, 表示匀速运动
      speed_profile.emplace_back(std::move(speed_point));                                  // 将新建的点构成速度的曲线
    }
    speed_data->set_speed_vector(std::move(speed_profile));                                // 并将速度曲线放到速度向量中
    return Status::OK();                                                                   // 并返回
  }
                                                                                           // 如果有障碍物
  if (!InitCostTable().ok()) {                                                             // 就初始化cost的代价函数表
    const std::string msg = "Initialize cost table failed.";                               // 初始化失败就会报错
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!CalculateTotalCost().ok()) {                                                        // 然后计算总共的费用(代价)
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!RetrieveSpeedProfile(speed_data).ok()) {                                            // 检索最佳速度曲线
    const std::string msg = "Retrieve best speed profile failed.";                         // 失败就会返回错误
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();                                                                     // 成功的话就会将最好的速度曲线放到speed_data中
}

Status DpStGraph::InitCostTable() {                                                        // 对cost_table(一个二维数据)进行初始化
  uint32_t dim_s = dp_st_speed_config_.matrix_dimension_s();                               // matrix_dimension_s被配置为150米
  uint32_t dim_t = dp_st_speed_config_.matrix_dimension_t();                               // matrix_dimension_t被配置为8秒
  DCHECK_GT(dim_s, 2);                                                                     // 必须大于2， 两个元素??? 150x8的举证
  DCHECK_GT(dim_t, 2);
  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dim_t, std::vector<StGraphPoint>(dim_s, StGraphPoint()));                            // 果然是150x8的矩阵, 而且行是t, 列是s(路程)

  float curr_t = 0.0;                                                                      // 初始化第一个时间点为是0
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {                   // 迭代时间维度的分辨率
    auto& cost_table_i = cost_table_[i];
    float curr_s = 0.0;
    for (uint32_t j = 0; j < cost_table_i.size(); ++j, curr_s += unit_s_) {                // 迭代路程维度的分辨率
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));                                 // 并且初始化每个代价函数的点
    }
  }
  return Status::OK();
}

Status DpStGraph::CalculateTotalCost() {                                                   // 动态规划计算最小的代价值
  // col and row are for STGraph
  // t corresponding to col
  // s corresponding to row
  uint32_t next_highest_row = 0;                                                           // 下一个最高的行
  uint32_t next_lowest_row = 0;                                                            // 下一个最低的行

  for (size_t c = 0; c < cost_table_.size(); ++c) {                                        // 一行一行的迭代
    int highest_row = 0;                                                                   // 最高的行设置为0
    int lowest_row = cost_table_.back().size() - 1;                                        // 最低的行就是初始化为最后一行

    int count = next_highest_row - next_lowest_row + 1;                                    // 最低和最高的差距
    if (count > 0) {
      std::vector<std::future<void>> futures;                                              // 如果大于零, 即有点像双指针

      for (uint32_t r = next_lowest_row; r <= next_highest_row; ++r) {                     // 迭代两个指针之间的内容
        if (FLAGS_enable_multi_thread_in_dp_st_graph) {                                    // 如果使用了多线程进行动态规划的计算, 就会开起线程池进行计算
          futures.push_back(ThreadPool::pool()->push(                                      // FLAGS_enable_multi_thread_in_dp_st_graph设置为false
              std::bind(&DpStGraph::CalculateCostAt, this, c, r)));
        } else {
          CalculateCostAt(c, r);                                                           // 计算一个点的代价
        }
      }

      for (const auto& f : futures) {                                                      // 没有使用多线程的话, 就会是空的数组
        f.wait();
      }
    }

    for (uint32_t r = next_lowest_row; r <= next_highest_row; ++r) {                       // 找到双指针之间的行
      const auto& cost_cr = cost_table_[c][r];                                             // 代价函数值的二维数组
      if (cost_cr.total_cost() < std::numeric_limits<float>::infinity()) {
        int h_r = 0;                                                                       // 高行
        int l_r = 0;                                                                       // 低行
        GetRowRange(cost_cr, &h_r, &l_r);                                                  // 获得大致的范围
        highest_row = std::max(highest_row, h_r);                                          // 扩展高行的值
        lowest_row = std::min(lowest_row, l_r);                                            // 扩展低行的值
      }
    }
    next_highest_row = highest_row;                                                        // 迭代最高的行(row)
    next_lowest_row = lowest_row;                                                          // 和最低的行(row)
  }

  return Status::OK();                                                                     // 迭代完了就返回OK的状态
}

void DpStGraph::GetRowRange(const StGraphPoint& point, int* next_highest_row,              // 返回一个st点所在位置的最低点和最高点
                            int* next_lowest_row) {
  float v0 = 0.0;                                                                          // 初始速度v0
  if (!point.pre_point()) {                                                                // 前一个点不存在， 就复制为起点
    v0 = init_point_.v();
  } else {
    v0 = (point.index_s() - point.pre_point()->index_s()) * unit_s_ / unit_t_;             // 否者就是用位置增量除以时间增量
  }

  const int max_s_size = cost_table_.back().size() - 1;                                    // 最大的行

  const float speed_coeff = unit_t_ * unit_t_;                                             // 速度的系数是时间的平方

  const float delta_s_upper_bound =
      v0 * unit_t_ + vehicle_param_.max_acceleration() * speed_coeff;                      // 上界s的增量
  *next_highest_row =
      point.index_s() + static_cast<int>(delta_s_upper_bound / unit_s_);                   // 下一个最高的行
  if (*next_highest_row >= max_s_size) {
    *next_highest_row = max_s_size;
  }

  const float delta_s_lower_bound = std::fmax(
      0.0, v0 * unit_t_ + vehicle_param_.max_deceleration() * speed_coeff);                // 下界的增量
  *next_lowest_row =
      point.index_s() + static_cast<int>(delta_s_lower_bound / unit_s_);
  if (*next_lowest_row > max_s_size) {
    *next_lowest_row = max_s_size;                                                         // 获取最大的s个数
  } else if (*next_lowest_row < 0) {
    *next_lowest_row = 0;
  }
}

void DpStGraph::CalculateCostAt(const uint32_t c, const uint32_t r) {                      // 计算二维数组中一个点中的值
  auto& cost_cr = cost_table_[c][r];
  cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));                           // 设置障碍物的代价
  if (cost_cr.obstacle_cost() > std::numeric_limits<float>::max()) {                       // 如果障碍物的代价特别大就直接返回
    return;
  }

  const auto& cost_init = cost_table_[0][0];                                               // 代价函数的初始值
  if (c == 0) {
    DCHECK_EQ(r, 0) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0);                                                             // 设置总共cost
    return;
  }

  float speed_limit =
      st_graph_data_.speed_limit().GetSpeedLimitByS(unit_s_ * r);                          // 速度限制
  if (c == 1) {                                                                            // 第一列的值
    const float acc = (r * unit_s_ / unit_t_ - init_point_.v()) / unit_t_;
    if (acc < dp_st_speed_config_.max_deceleration() ||                                    // 最大减速度
        acc > dp_st_speed_config_.max_acceleration()) {                                    // 最大加速度
      return;
    }

    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                cost_init)) {                                              // 检查是否和起点有重叠
      return;                                                                              // 如果有重叠的点就直接返回
    }
    cost_cr.SetTotalCost(cost_cr.obstacle_cost() + cost_init.total_cost() +                // total cost是所有代价的叠加值
                         CalculateEdgeCostForSecondCol(r, speed_limit));                   // 第二列的代价值
    cost_cr.SetPrePoint(cost_init);                                                        // 设置起点
    return;                                                                                // 然后返回
  }

  constexpr float kSpeedRangeBuffer = 0.20;                                                // 速度range的一个buffer(缓冲区为0.2米)
  const uint32_t max_s_diff =
      static_cast<uint32_t>(FLAGS_planning_upper_speed_limit *                             // planning速度的上界是31.3米每秒
                            (1 + kSpeedRangeBuffer) * unit_t_ / unit_s_);                  // 每个分辨率就会增加一个buffer
  const uint32_t r_low = (max_s_diff < r ? r - max_s_diff : 0);                            // 获取最低的行， 每一行就代表一个距离s

  const auto& pre_col = cost_table_[c - 1];                                                // 获得前一列(cloumn)的值

  if (c == 2) {                                                                            // 如果是第二列(就是第二秒内的速度)
    for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {                                    // 迭代前向的s
      const float acc =
          (r * unit_s_ - 2 * r_pre * unit_s_) / (unit_t_ * unit_t_);                       // 计算加速度
      if (acc < dp_st_speed_config_.max_deceleration() ||
          acc > dp_st_speed_config_.max_acceleration()) {                                  // 加速度的范围必须在有效范围内
        continue;
      }

      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,                 // 检查是否有重叠的boundary
                                  pre_col[r_pre])) {                                       // 有的话就继续
        continue;
      }

      const float cost = cost_cr.obstacle_cost() + pre_col[r_pre].total_cost() +           // 障碍物的代价, 再加上第三行的代价
                         CalculateEdgeCostForThirdCol(r, r_pre, speed_limit);

      if (cost < cost_cr.total_cost()) {                                                   // 获取更小的代价
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);
      }
    }
    return;                                                                                // 计算完后直接返回
  }
  for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {                                      // 迭代前向点
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {                                           // 有效性检查
      continue;
    }

    const float curr_a = (cost_cr.index_s() * unit_s_ +
                          pre_col[r_pre].pre_point()->index_s() * unit_s_ -
                          2 * pre_col[r_pre].index_s() * unit_s_) /
                         (unit_t_ * unit_t_);                                              // 计算当前的加速度
    if (curr_a > vehicle_param_.max_acceleration() ||
        curr_a < vehicle_param_.max_deceleration()) {                                      // 保证当前的加速度在有效的范围内
      continue;
    }
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,                   // 检查是否有重叠
                                pre_col[r_pre])) {
      continue;
    }

    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];                 // 获得前两项的节点
    if (std::isinf(prepre_graph_point.total_cost())) {                                     // 不存在继续迭代
      continue;
    }

    if (!prepre_graph_point.pre_point()) {                                                 // 前向节点不存在也继续迭代
      continue;
    }
    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();             // 取出前三项节点
    const STPoint& prepre_point = prepre_graph_point.point();                              // 取出前二个节点
    const STPoint& pre_point = pre_col[r_pre].point();                                     // 取出前向节点
    const STPoint& curr_point = cost_cr.point();                                           // 计算当前节点
    float cost = cost_cr.obstacle_cost() + pre_col[r_pre].total_cost() +
                 CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                   curr_point, speed_limit);                               // 计算叠加的代价

    if (cost < cost_cr.total_cost()) {                                                     // 如果比较小就保存较小的代价
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
    }
  }
}

Status DpStGraph::RetrieveSpeedProfile(SpeedData* const speed_data) {                      // 修正速度曲线
  float min_cost = std::numeric_limits<float>::infinity();                                 // 声明一个最小的代价
  const StGraphPoint* best_end_point = nullptr;
  for (const StGraphPoint& cur_point : cost_table_.back()) {                               // 从后往前迭代动态规划
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {                                               // 保存最好的点
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  for (const auto& row : cost_table_) {                                                    // 迭代每一行
    const StGraphPoint& cur_point = row.back();
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();                                                   // 找到最小的代价
    }
  }

  if (best_end_point == nullptr) {                                                         // 如果是空指针就报错
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;                                          // 将速度保存在速度曲线上
  while (cur_point != nullptr) {                                                           // 当前的速度点不为空指针
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.emplace_back(speed_point);
    cur_point = cur_point->pre_point();                                                    // 取出前向点
  }
  std::reverse(speed_profile.begin(), speed_profile.end());                                // 然后将速度的曲线翻转

  constexpr float kEpsilon = std::numeric_limits<float>::epsilon();                        // 无穷小
  if (speed_profile.front().t() > kEpsilon ||
      speed_profile.front().s() > kEpsilon) {                                              // 第一个点应该是无穷小(即是0)
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  speed_data->set_speed_vector(speed_profile);                                             // 将速度点放到速度数据中(speed data)
  return Status::OK();
}

float DpStGraph::CalculateEdgeCost(const STPoint& first, const STPoint& second,
                                   const STPoint& third, const STPoint& forth,
                                   const float speed_limit) {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit) +                         // 速度的代价
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +                 // 加速度的代价
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);             // 加速度的导数的代价
}

float DpStGraph::CalculateEdgeCostForSecondCol(const uint32_t row,                     // 计算两行的边组成的成本
                                               const float speed_limit) {
  float init_speed = init_point_.v();                                                  // 初始化速度(起点的速度)
  float init_acc = init_point_.a();                                                    // 起点的加速度
  const STPoint& pre_point = cost_table_[0][0].point();                                // 这里为什么要反着放st的曲线
  const STPoint& curr_point = cost_table_[1][row].point();
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,                    // 两个点的加速度
                                             curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,           // 两个点的加速度的导数
                                            curr_point);
}

float DpStGraph::CalculateEdgeCostForThirdCol(const uint32_t curr_row,                 // 计算三行的的代价函数值
                                              const uint32_t pre_row,
                                              const float speed_limit) {
  float init_speed = init_point_.v();
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

}  // namespace planning
}  // namespace apollo
