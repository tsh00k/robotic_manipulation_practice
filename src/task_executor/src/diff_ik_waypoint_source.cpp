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

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "arm_kinematics/forward_kinematics.hpp"

namespace task_executor
{
DiffIkWaypointSource::DiffIkWaypointSource(
  arm_kinematics::ArmModel model,
  std::shared_ptr<const CartesianWaypointSource> cartesian_source,
  std::array<double, 7> home_joint_positions)
: model_(std::move(model)), cartesian_source_(std::move(cartesian_source)),
  home_joint_positions_(home_joint_positions)
{
  if (!cartesian_source_) {
    throw std::invalid_argument("Cartesian waypoint source must not be null");
  }
  arm_kinematics::JointVector home;
  for (std::size_t i = 0; i < home_joint_positions_.size(); ++i) {
    home(static_cast<Eigen::Index>(i)) = home_joint_positions_[i];
  }
  if (!home.allFinite() || !arm_kinematics::withinJointLimits(model_, home)) {
    throw std::invalid_argument("HOME joint positions must be finite and within joint limits");
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
  if (phase == Phase::kHome || phase == Phase::kVerify) {
    // No IK: the gripper width comes from the task, the arm is the HOME configuration, and the
    // diagnostics report that configuration's own TCP with zero residual.
    arm_kinematics::IkResult exact;
    exact.status = arm_kinematics::IkStatus::kConverged;
    for (std::size_t i = 0; i < home_joint_positions_.size(); ++i) {
      exact.q(static_cast<Eigen::Index>(i)) = home_joint_positions_[i];
    }
    exact.position_error = 0.0;
    exact.orientation_error = 0.0;
    cached_target_ = JointTarget{home_joint_positions_, waypoint.gripper_width_m};
    cached_phase_ = phase;
    diagnostics_ = WaypointDiagnostics{arm_kinematics::fk(model_, exact.q).hand_tcp, exact};
    return cached_target_;
  }
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

std::vector<std::array<double, 7>> DiffIkWaypointSource::cartesianLine(
  const std::array<double, 7> & start, const std::array<double, 7> & goal, double step_m,
  double max_joint_jump_rad) const
{
  arm_kinematics::JointVector q_start, q_goal;
  for (std::size_t i = 0; i < 7; ++i) {
    q_start(static_cast<Eigen::Index>(i)) = start[i];
    q_goal(static_cast<Eigen::Index>(i)) = goal[i];
  }
  const Eigen::Isometry3d a = arm_kinematics::fk(model_, q_start).hand_tcp;
  const Eigen::Isometry3d b = arm_kinematics::fk(model_, q_goal).hand_tcp;
  const Eigen::Quaterniond ra(a.linear()), rb(b.linear());
  // About step_m apart, and at most 0.02 rad of tool rotation between points.
  const double length = (b.translation() - a.translation()).norm();
  const int intervals = std::max(
    1, static_cast<int>(std::ceil(std::max(length / step_m, ra.angularDistance(rb) / 0.02))));

  std::vector<std::array<double, 7>> path{start};
  arm_kinematics::JointVector seed = q_start;
  for (int k = 1; k <= intervals; ++k) {
    const double s = static_cast<double>(k) / intervals;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = a.translation() + s * (b.translation() - a.translation());
    pose.linear() = ra.slerp(s, rb).toRotationMatrix();
    const auto result = arm_kinematics::solveIk(model_, seed, pose);
    if (result.status != arm_kinematics::IkStatus::kConverged) {
      throw std::runtime_error(
              "TCP line: IK did not converge at point " + std::to_string(k) + " of " +
              std::to_string(intervals));
    }
    const double jump = (result.q - seed).cwiseAbs().maxCoeff();
    if (jump > max_joint_jump_rad) {
      throw std::runtime_error(
              "TCP line: joints jump " + std::to_string(jump) + " rad at point " +
              std::to_string(k) + " of " + std::to_string(intervals));
    }
    std::array<double, 7> q{};
    for (std::size_t i = 0; i < 7; ++i) {
      q[i] = result.q(static_cast<Eigen::Index>(i));
    }
    path.push_back(q);
    seed = result.q;
  }
  return path;
}

}  // namespace task_executor
