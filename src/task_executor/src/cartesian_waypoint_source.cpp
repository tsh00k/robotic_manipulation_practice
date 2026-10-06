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

#include "task_executor/cartesian_waypoint_source.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace task_executor
{

namespace
{
constexpr double kOpenWidthM = 0.08;
constexpr double kClosedWidthM = 0.0;

// At zero offset, TCP x/y/z map to world +y/+x/-z. Apply the configured
// rotation about world z independently of the target position.
Eigen::Isometry3d downwardPose(double x, double y, double z, double tool_yaw_rad)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  const Eigen::Vector3d diagonal = Eigen::Vector3d(1.0, 1.0, 0.0).normalized();
  pose.linear() = (
    Eigen::AngleAxisd(tool_yaw_rad, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(M_PI, diagonal)).toRotationMatrix();
  return pose;
}
}  // namespace

PickPlaceCartesianWaypointSource::PickPlaceCartesianWaypointSource(PickPlaceGeometry geometry)
: geometry_(std::move(geometry))
{
  if (!std::isfinite(geometry_.box_half_height_m) ||
    !std::isfinite(geometry_.hover_height_m) ||
    !std::isfinite(geometry_.place_tcp_above_box_center_m) ||
    !std::isfinite(geometry_.tool_yaw_rad) ||
    geometry_.box_half_height_m <= 0.0 || geometry_.hover_height_m <= 0.0 ||
    geometry_.place_tcp_above_box_center_m <= 0.0)
  {
    throw std::invalid_argument("Invalid pick-and-place geometry");
  }
}

CartesianWaypoint PickPlaceCartesianWaypointSource::waypointFor(
  Phase phase, const ObjectPose & object_pose, const PlaceTarget & place) const
{
  if (!std::isfinite(place.x) || !std::isfinite(place.y) || !std::isfinite(place.support_z)) {
    throw std::invalid_argument("Place target must be finite");
  }
  if (!std::isfinite(object_pose.x) || !std::isfinite(object_pose.y) ||
    !std::isfinite(object_pose.z) || !std::isfinite(object_pose.qw) ||
    !std::isfinite(object_pose.qx) || !std::isfinite(object_pose.qy) ||
    !std::isfinite(object_pose.qz) ||
    std::abs(
      object_pose.qw * object_pose.qw + object_pose.qx * object_pose.qx +
      object_pose.qy * object_pose.qy + object_pose.qz * object_pose.qz - 1.0) > 1e-3)
  {
    throw std::invalid_argument("Object pose must be finite with unit quaternion");
  }

  CartesianWaypoint waypoint;
  waypoint.phase = phase;
  // Box centre resting on the support surface: the table, or the bin floor (Stage 8).
  const double place_box_z = place.support_z + geometry_.box_half_height_m;
  switch (phase) {
    case Phase::kHome:
      waypoint.world_to_hand_tcp = downwardPose(0.5545, 0.0, 0.5211, geometry_.tool_yaw_rad);
      break;
    case Phase::kPregrasp:
    case Phase::kLift:
      waypoint.world_to_hand_tcp = downwardPose(
        object_pose.x, object_pose.y, object_pose.z + geometry_.hover_height_m,
        geometry_.tool_yaw_rad);
      break;
    case Phase::kGrasp:
    case Phase::kClose:
      waypoint.world_to_hand_tcp = downwardPose(
        object_pose.x, object_pose.y, object_pose.z, geometry_.tool_yaw_rad);
      break;
    case Phase::kPreplace:
    case Phase::kRetract:
    case Phase::kVerify:
    case Phase::kDone:
    case Phase::kRecover:
    case Phase::kFailed:
      waypoint.world_to_hand_tcp = downwardPose(
        place.x, place.y,
        place_box_z + geometry_.hover_height_m, geometry_.tool_yaw_rad);
      break;
    case Phase::kPlace:
    case Phase::kOpen:
      waypoint.world_to_hand_tcp = downwardPose(
        place.x, place.y,
        place_box_z + geometry_.place_tcp_above_box_center_m, geometry_.tool_yaw_rad);
      break;
  }
  if (phase == Phase::kClose || phase == Phase::kLift || phase == Phase::kPreplace ||
    phase == Phase::kPlace)
  {
    waypoint.gripper_width_m = kClosedWidthM;
  } else {
    waypoint.gripper_width_m = kOpenWidthM;
  }
  return waypoint;
}

}  // namespace task_executor
