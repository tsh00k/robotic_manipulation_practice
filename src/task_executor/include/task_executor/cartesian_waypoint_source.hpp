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

#include "task_executor/waypoint_source.hpp"

namespace task_executor
{

// The pose maps hand_tcp coordinates into world. There is no ROS message, joint
// configuration, or implicit camera-frame transform in this task contract.
struct CartesianWaypoint
{
  Phase phase = Phase::kHome;
  Eigen::Isometry3d world_to_hand_tcp = Eigen::Isometry3d::Identity();
  double gripper_width_m = 0.08;
};

class CartesianWaypointSource
{
public:
  virtual ~CartesianWaypointSource() = default;
  virtual CartesianWaypoint waypointFor(
    Phase phase, const ObjectPose & object_pose, const PlaceTarget & place) const = 0;
};

struct PickPlaceGeometry
{
  // Where to put the box is per episode (PlaceTarget); these are the fixed shape of the task.
  double box_half_height_m = 0.02;
  double hover_height_m = 0.15;
  double place_tcp_above_box_center_m = 0.05;
  // Rotation about world z relative to the downward reference, not Euler yaw.
  double tool_yaw_rad = 0.0;
};

// Stage geometry in world coordinates.
class PickPlaceCartesianWaypointSource : public CartesianWaypointSource
{
public:
  explicit PickPlaceCartesianWaypointSource(PickPlaceGeometry geometry = {});
  CartesianWaypoint waypointFor(
    Phase phase, const ObjectPose & object_pose, const PlaceTarget & place) const override;

private:
  PickPlaceGeometry geometry_;
};

}  // namespace task_executor
