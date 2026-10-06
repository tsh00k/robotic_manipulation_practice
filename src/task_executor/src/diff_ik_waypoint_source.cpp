// Copyright 2026 anby
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "task_executor/diff_ik_waypoint_source.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace task_executor
{
DiffIkWaypointSource::DiffIkWaypointSource(
  arm_kinematics::ArmModel model,
  std::shared_ptr<const CartesianWaypointSource> cartesian_source)
: model_(std::move(model)), cartesian_source_(std::move(cartesian_source))
{
  if (!cartesian_source_) {
    throw std::invalid_argument("Cartesian waypoint source must not be null");
  }
}

void DiffIkWaypointSource::beginEpisode()
{
  cached_phase_.reset();
  grasp_object_pose_.reset();
  diagnostics_.reset();
}

void DiffIkWaypointSource::setSeed(const std::array<double, 7> & positions)
{
  for (std::size_t i = 0; i < positions.size(); ++i) {
    seed_(static_cast<Eigen::Index>(i)) = positions[i];
  }
  have_seed_ = true;
}

JointTarget DiffIkWaypointSource::jointTargetFor(
  Phase phase, const ObjectPose & object_pose, const PlaceTarget & place) const
{
  if (cached_phase_ == phase) {
    return cached_target_;
  }
  if (!have_seed_) {
    throw std::logic_error("IK seed has not been set");
  }
  if (!std::isfinite(object_pose.x) || !std::isfinite(object_pose.y) ||
    !std::isfinite(object_pose.z))
  {
    throw std::invalid_argument("Object position must be finite");
  }
  if (phase == Phase::kPregrasp && !grasp_object_pose_) {
    grasp_object_pose_ = object_pose;
  }

  // Every phase after PREGRASP uses the box pose seen at PREGRASP: the grasp targets must not
  // follow the box once it is touched, and the tool rotation (taken from the box yaw, Week 4.1
  // Stage 9) must not change while the box is held. Place-side positions come from the place
  // target and do not read the box pose.
  const ObjectPose & task_object = grasp_object_pose_.value_or(object_pose);
  const CartesianWaypoint waypoint = cartesian_source_->waypointFor(phase, task_object, place);
  const Eigen::Isometry3d & tcp_target = waypoint.world_to_hand_tcp;
  const auto result = arm_kinematics::solveIk(model_, seed_, tcp_target);
  if (result.status != arm_kinematics::IkStatus::kConverged) {
    throw std::runtime_error(
            "IK did not converge: iterations=" + std::to_string(result.iterations) +
            " position_error=" + std::to_string(result.position_error) +
            " orientation_error=" + std::to_string(result.orientation_error));
  }
  JointTarget target;
  target.gripper_width_m = waypoint.gripper_width_m;
  for (std::size_t i = 0; i < target.arm_positions.size(); ++i) {
    target.arm_positions[i] = result.q(static_cast<Eigen::Index>(i));
  }
  cached_target_ = target;
  cached_phase_ = phase;
  diagnostics_ = WaypointDiagnostics{tcp_target, result};
  return cached_target_;
}

}  // namespace task_executor
