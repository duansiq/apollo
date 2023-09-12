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

#include "modules/planning/lattice/behavior/prediction_querier.h"

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/path_matcher.h"

namespace apollo {
namespace planning {

PredictionQuerier::PredictionQuerier(
    const std::vector<const Obstacle*>& obstacles,
    const std::shared_ptr<std::vector<common::PathPoint>>& ptr_reference_line)
    : ptr_reference_line_(ptr_reference_line) {
  for (const auto ptr_obstacle : obstacles) {
    if (common::util::InsertIfNotPresent(&id_obstacle_map_, ptr_obstacle->Id(),
                                         ptr_obstacle)) {
       //将障碍物的 Id 作为键，ptr_obstacle 指针作为值插入到 id_obstacle_map_ 中。
       //如果插入成功（即之前不存在相同的键），则将 ptr_obstacle 添加到 obstacles_ 向量中。如果插入失败（即存在相同的键），则会发出警告信息并跳过添加操作
      obstacles_.push_back(ptr_obstacle);
    } else {
      AWARN << "Duplicated obstacle found [" << ptr_obstacle->Id() << "]";
    }
  }
}

std::vector<const Obstacle*> PredictionQuerier::GetObstacles() const {
  return obstacles_;
}

double PredictionQuerier::ProjectVelocityAlongReferenceLine(
    const std::string& obstacle_id, const double s, const double t) const {
  ACHECK(id_obstacle_map_.find(obstacle_id) != id_obstacle_map_.end());

  const auto& trajectory = id_obstacle_map_.at(obstacle_id)->Trajectory();//根据id，取出障碍物的轨迹
  int num_traj_point = trajectory.trajectory_point_size();//获取障碍物轨迹点的数量，数量过小，则视为不动
  if (num_traj_point < 2) {
    return 0.0;
  }

  
  // 如果传入的t不在障碍物轨迹点的t的范围之内，说明该障碍物不会造成影响，直接返回
  if (t < trajectory.trajectory_point(0).relative_time() ||
      t > trajectory.trajectory_point(num_traj_point - 1).relative_time()) {
    return 0.0;
  }

  // 查找最靠近时间点t的轨迹点
  auto matched_it =
      std::lower_bound(trajectory.trajectory_point().begin(),
                       trajectory.trajectory_point().end(), t,
                       [](const common::TrajectoryPoint& p, const double t) {
                         return p.relative_time() < t;
                       });

  double v = matched_it->v();
  double theta = matched_it->path_point().theta();
  double v_x = v * std::cos(theta);
  double v_y = v * std::sin(theta);

  common::PathPoint obstacle_point_on_ref_line =
      common::math::PathMatcher::MatchToPath(*ptr_reference_line_, s);
  auto ref_theta = obstacle_point_on_ref_line.theta();

  return std::cos(ref_theta) * v_x + std::sin(ref_theta) * v_y;
}

}  // namespace planning
}  // namespace apollo
