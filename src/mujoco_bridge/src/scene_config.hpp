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

// Scene layout (box / bin start poses) for the simulator. Pure geometry and validation:
// no ROS, no MuJoCo, so it is unit-testable on its own (test_scene_config.cpp).
//
// This header lives in src/, not include/mujoco_bridge/, on purpose. include/ is both
// installed and the PUBLIC include directory of the exported grasp_criteria target, so
// anything there is reachable from task_executor. The scene poses are the simulator's
// ground truth about where things are; perception and the executor must get box/bin
// poses from the camera (docs/adr/015), and keeping this out of the exported include
// tree makes "they do not even compile against it" a build fact instead of a habit.

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace mujoco_bridge
{

// Extents the placement checks need, read from the loaded model (scene_ops.hpp,
// readSceneGeometry) so that the MJCF stays the only place the numbers are written.
struct SceneGeometry
{
  // Box geom half extents, in the box body frame (origin = box centre).
  Eigen::Vector3d box_half_extents = Eigen::Vector3d::Zero();
  // Axis-aligned extent of every bin geom (floor and walls) in the bin body frame,
  // whose origin is the centre of the inner floor surface.
  Eigen::Vector3d bin_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d bin_max = Eigen::Vector3d::Zero();
  double table_top_z = 0.0;
  Eigen::Vector2d table_min_xy = Eigen::Vector2d::Zero();
  Eigen::Vector2d table_max_xy = Eigen::Vector2d::Zero();
};

// A start pose as given on the command line. Metres and radians, expressed in world,
// rotation R = Rz(yaw) * Ry(pitch) * Rx(roll). z is the origin height (not the support
// height): for the box the centre, for the bin the inner floor surface.
struct PoseRequest
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

struct SceneRequest
{
  PoseRequest box;
  PoseRequest bin;
};

struct ScenePoses
{
  Eigen::Isometry3d box = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d bin = Eigen::Isometry3d::Identity();
};

// Defaults for x/y when a pose parameter is not given. The box default matches
// pick_place_scene.xml's own box position, so scene.enabled=true with no other
// parameter reproduces the legacy layout.
constexpr double kDefaultBoxX = 0.5;
constexpr double kDefaultBoxY = 0.0;
constexpr double kDefaultBinX = 0.5;
constexpr double kDefaultBinY = 0.3;

// Gap left between the lowest corner and the table when z is chosen automatically,
// the same 1 mm the MJCF uses for the default box, so the body starts out of contact.
constexpr double kAutoSupportGapM = 0.001;
// An explicit z may sink the lowest corner this far below the table top before it
// counts as penetration (numerical noise, not intent).
constexpr double kPenetrationToleranceM = 0.0001;
// Smallest z component of the bin's up axis: cos(20 deg) = 0.9397. A more tilted bin
// changes where a box can come to rest and is outside what the task is validated for.
constexpr double kMinBinUpAxisZ = 0.94;
// Free space required between the box footprint and the bin's outer footprint: half
// of the spare gripper opening, (0.08 m open - 0.04 m box) / 2. A necessary condition
// for the fingers to reach the box, not a proof that they can; reachability and wall
// clearance are validated when the bin becomes a task target.
constexpr double kBoxBinClearanceM = 0.02;

Eigen::Matrix3d rotationFromRpy(double roll, double pitch, double yaw);

// Origin height that rests the lowest corner of the oriented box [lo, hi] (given in its
// own frame) kAutoSupportGapM above the table.
double autoSupportZ(
  const Eigen::Matrix3d & rotation, const Eigen::Vector3d & lo, const Eigen::Vector3d & hi,
  double table_top_z);

// Validates both poses against the geometry and returns them as transforms.
// Throws std::invalid_argument naming the offending parameter and why, never clamps:
//   - non-finite values
//   - a pose whose lowest corner is below the table top (penetration)
//   - a pose whose footprint leaves the table
//   - a bin tilted more than kMinBinUpAxisZ allows
//   - a box footprint closer than kBoxBinClearanceM to the bin footprint
ScenePoses resolveScene(const SceneRequest & request, const SceneGeometry & geometry);

// A scene.box.* / scene.bin.* parameter given while scene.enabled is false would be
// silently ignored; throws std::invalid_argument naming it instead.
void rejectPoseOverridesWhenDisabled(
  bool enabled, const std::vector<std::string> & parameter_names);

}  // namespace mujoco_bridge
