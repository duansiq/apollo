/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/math/path_matcher.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "glog/logging.h"

#include "modules/common/math/linear_interpolation.h"

namespace apollo {
namespace common {
namespace math {

//找到匹配点
PathPoint PathMatcher::MatchToPath(const std::vector<PathPoint>& reference_line,
                                   const double x, const double y) {
  CHECK_GT(reference_line.size(), 0U);

  auto func_distance_square = [](const PathPoint& point, const double x,
                                 const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return dx * dx + dy * dy;
  };

  // 首先计算了一下参考线的起点到traj上当前车辆位置的距离
  double distance_min = func_distance_square(reference_line.front(), x, y);
  std::size_t index_min = 0;// 不仅要知道大小,还要知道索引
  // 从index为1的点开始(因为index=0的点前面刚计算过),依次计算和车辆位置的距离
  for (std::size_t i = 1; i < reference_line.size(); ++i) {
    double distance_temp = func_distance_square(reference_line[i], x, y);
    // 找到参考线上距离自车当前位置最近的点的index
    if (distance_temp < distance_min) {
      distance_min = distance_temp;
      index_min = i;
    }
  }

  std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;
  std::size_t index_end =
      (index_min + 1 == reference_line.size()) ? index_min : index_min + 1;
  // 到这里找下来相当于在referenceLine上找了三个点,即参考线上距离车辆位置最近的点,以及这个点
  // 的前后各一个点,为什么要在前后各找一个点呢?因为他要做车辆当前的点 在referenceLine上的投影,
  // 这里会把index_start和index_end连起来,当作一条线(方向start指向end),供我们做投影,公式为:
  // 向量A(index_start -> traj上的车辆当前位置) 点乘 向量B(index_start -> index_end) 
  if (index_start == index_end) {
    return reference_line[index_start];
  }

  return FindProjectionPoint(reference_line[index_start],
                             reference_line[index_end], x, y);
}

//获得障碍物在SL坐标系中的s，l值
std::pair<double, double> PathMatcher::GetPathFrenetCoordinate(
    const std::vector<PathPoint>& reference_line, const double x,
    const double y) {
  auto matched_path_point = MatchToPath(reference_line, x, y);
  double rtheta = matched_path_point.theta();
  double rx = matched_path_point.x();
  double ry = matched_path_point.y();
  double delta_x = x - rx;
  double delta_y = y - ry;
  double side = std::cos(rtheta) * delta_y - std::sin(rtheta) * delta_x;
  std::pair<double, double> relative_coordinate;
  relative_coordinate.first = matched_path_point.s();
  relative_coordinate.second =
      std::copysign(std::hypot(delta_x, delta_y), side);
  return relative_coordinate;
}

PathPoint PathMatcher::MatchToPath(const std::vector<PathPoint>& reference_line,
                                   const double s) {
  auto comp = [](const PathPoint& point, const double s) {
    return point.s() < s;
  };

  auto it_lower =
      std::lower_bound(reference_line.begin(), reference_line.end(), s, comp);
  if (it_lower == reference_line.begin()) {
    return reference_line.front();
  } else if (it_lower == reference_line.end()) {
    return reference_line.back();
  }

  // interpolate between it_lower - 1 and it_lower
  // return interpolate(*(it_lower - 1), *it_lower, s);
  return InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, s);
}

//投影匹配点
PathPoint PathMatcher::FindProjectionPoint(const PathPoint& p0,
                                           const PathPoint& p1, const double x,
                                           const double y) {
  double v0x = x - p0.x();
  double v0y = y - p0.y();

  double v1x = p1.x() - p0.x();
  double v1y = p1.y() - p0.y();

  double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
  double dot = v0x * v1x + v0y * v1y;

  // v0 · v1 = ||v0||*||v1||*cos(theta)    theta为v0和v1的夹角
  // (v0 · v1) / ||v1|| = ||v0||*cos(theta)
  // ||v0||*cos(theta) 即为v0在v1上的投影
  double delta_s = dot / v1_norm;
  return InterpolateUsingLinearApproximation(p0, p1, p0.s() + delta_s);//线性插值返回轨迹点
}

}  // namespace math
}  // namespace common
}  // namespace apollo
