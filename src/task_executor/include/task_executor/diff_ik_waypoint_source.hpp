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

#pragma once

#include <Eigen/Geometry>

#include <optional>
#include <memory>

#include "arm_kinematics/differential_ik.hpp"
#include "task_executor/cartesian_waypoint_source.hpp"

namespace task_executor
{

struct WaypointDiagnostics
{
  Eigen::Isometry3d tcp_target = Eigen::Isometry3d::Identity();
  arm_kinematics::IkResult ik;
};

// Adapts a Cartesian task waypoint to the legacy joint-command interface.
// Solves once on phase entry, then holds the commanded joint target while the
// physical arm settles. The grasp-side object reference is latched at PREGRASP;
// later object motion while carried cannot drag the target along with it.
class DiffIkWaypointSource : public WaypointSource
{
public:
  DiffIkWaypointSource(
    arm_kinematics::ArmModel model,
    std::shared_ptr<const CartesianWaypointSource> cartesian_source);

  void beginEpisode();
  void setSeed(const std::array<double, 7> & positions);
  JointTarget jointTargetFor(
    Phase phase, const ObjectPose & object_pose, const PlaceTarget & place) const override;
  const std::optional<WaypointDiagnostics> & diagnostics() const {return diagnostics_;}

private:
  arm_kinematics::ArmModel model_;
  std::shared_ptr<const CartesianWaypointSource> cartesian_source_;
  arm_kinematics::JointVector seed_ = arm_kinematics::JointVector::Zero();
  bool have_seed_ = false;
  mutable std::optional<Phase> cached_phase_;
  mutable std::optional<ObjectPose> grasp_object_pose_;
  mutable JointTarget cached_target_;
  mutable std::optional<WaypointDiagnostics> diagnostics_;
};

}  // namespace task_executor
