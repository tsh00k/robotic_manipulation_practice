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

#include <array>

#include "task_executor/phase.hpp"

namespace task_executor
{

// Plain struct, not geometry_msgs::msg::Pose -- same "Layer 1 stays message-free"
// discipline mujoco_bridge's grasp_criteria.hpp uses for GraspSignals (week2.md
// Stage F/H). Orientation is carried for the interface's sake (a future IK-based
// source needs it) but KeyframeWaypointSource below ignores it entirely.
struct ObjectPose
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
};

// Where the box is to be put down: the centre of the support surface (world x, y) and its
// height, which is the table top without a bin and the bin's inner floor with one
// (Week 4.1 Stage 8). The defaults are the legacy scene's fixed place marker on the table.
struct PlaceTarget
{
  double x = 0.5;
  double y = 0.3;
  double support_z = 0.22;
  // The bin's long side about world z (Stage 11's containment check); 0 without a bin.
  double yaw_rad = 0.0;
};

// What one phase commands: the 7 arm joints (joint1..joint7, mujoco_bridge's own
// order) plus the gripper's total finger-to-finger opening in meters -- the same
// two-part split as mujoco_bridge's ~/joint_command vs ~/gripper_command (Stage H).
struct JointTarget
{
  std::array<double, 7> arm_positions{};
  double gripper_width_m = 0.0;
};

// Swappable target abstraction (week2.md Stage I). KeyframeWaypointSource is a
// fixed lookup table; DiffIkWaypointSource uses object_pose and offline IK.
// The FSM consumes JointTarget without knowing which source produced it.
class WaypointSource
{
public:
  virtual ~WaypointSource() = default;

  virtual JointTarget jointTargetFor(
    Phase phase, const ObjectPose & object_pose, const PlaceTarget & place) const = 0;
};

}  // namespace task_executor
