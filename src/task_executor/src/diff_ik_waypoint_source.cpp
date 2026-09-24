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

#include "arm_kinematics/forward_kinematics.hpp"

namespace task_executor
{

namespace
{
constexpr double kReferenceBoxX = 0.5;
constexpr double kReferenceBoxY = 0.0;
constexpr double kReferenceBoxZ = 0.241;

Phase armReferencePhase(Phase phase)
{
  switch (phase) {
    case Phase::kClose: return Phase::kGrasp;
    case Phase::kOpen: return Phase::kPlace;
    case Phase::kVerify:
    case Phase::kDone:
    case Phase::kRecover:
    case Phase::kFailed: return Phase::kRetract;
    default: return phase;
  }
}

bool followsObject(Phase phase)
{
  return phase == Phase::kPregrasp || phase == Phase::kGrasp ||
         phase == Phase::kClose || phase == Phase::kLift;
}
}  // namespace

DiffIkWaypointSource::DiffIkWaypointSource(arm_kinematics::ArmModel model)
: model_(std::move(model))
{
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

Eigen::Isometry3d DiffIkWaypointSource::tcpTargetFor(
  Phase phase, const ObjectPose & object_pose) const
{
  const JointTarget reference = reference_.jointTargetFor(armReferencePhase(phase), object_pose);
  arm_kinematics::JointVector q;
  for (std::size_t i = 0; i < reference.arm_positions.size(); ++i) {
    q(static_cast<Eigen::Index>(i)) = reference.arm_positions[i];
  }
  Eigen::Isometry3d target = arm_kinematics::fk(model_, q).hand_tcp;
  if (followsObject(phase)) {
    const ObjectPose & observed = grasp_object_pose_.value_or(object_pose);
    target.translation() += Eigen::Vector3d(
      observed.x - kReferenceBoxX, observed.y - kReferenceBoxY,
      observed.z - kReferenceBoxZ);
  }
  return target;
}

JointTarget DiffIkWaypointSource::jointTargetFor(
  Phase phase, const ObjectPose & object_pose) const
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

  const Eigen::Isometry3d tcp_target = tcpTargetFor(phase, object_pose);
  const auto result = arm_kinematics::solveIk(model_, seed_, tcp_target);
  if (result.status != arm_kinematics::IkStatus::kConverged) {
    throw std::runtime_error(
            "IK did not converge: iterations=" + std::to_string(result.iterations) +
            " position_error=" + std::to_string(result.position_error) +
            " orientation_error=" + std::to_string(result.orientation_error));
  }
  JointTarget target = reference_.jointTargetFor(phase, object_pose);
  for (std::size_t i = 0; i < target.arm_positions.size(); ++i) {
    target.arm_positions[i] = result.q(static_cast<Eigen::Index>(i));
  }
  cached_target_ = target;
  cached_phase_ = phase;
  diagnostics_ = WaypointDiagnostics{tcp_target, result};
  return cached_target_;
}

}  // namespace task_executor
