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

#include "scene_config.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace mujoco_bridge
{

namespace
{

// The eight corners of the axis-aligned box [lo, hi], in its own frame.
std::array<Eigen::Vector3d, 8> corners(const Eigen::Vector3d & lo, const Eigen::Vector3d & hi)
{
  std::array<Eigen::Vector3d, 8> result;
  for (int i = 0; i < 8; ++i) {
    result[i] = Eigen::Vector3d(
      (i & 1) ? hi.x() : lo.x(), (i & 2) ? hi.y() : lo.y(), (i & 4) ? hi.z() : lo.z());
  }
  return result;
}

[[noreturn]] void reject(const std::string & parameter, const std::string & reason)
{
  throw std::invalid_argument(parameter + ": " + reason);
}

std::string number(double value)
{
  std::ostringstream out;
  out.precision(4);
  out << value;
  return out.str();
}

struct ValidatedPose
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  // World-frame extent of the oriented box, for the table and overlap checks.
  Eigen::Vector3d world_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d world_max = Eigen::Vector3d::Zero();
};

ValidatedPose validatePose(
  const std::string & name, const PoseRequest & pose, const Eigen::Vector3d & lo,
  const Eigen::Vector3d & hi, const SceneGeometry & geometry)
{
  // z last: an automatic z is derived from the angles, so a non-finite angle would
  // otherwise be reported as a non-finite z.
  const std::pair<const char *, double> fields[] = {
    {"x", pose.x}, {"y", pose.y}, {"roll", pose.roll}, {"pitch", pose.pitch},
    {"yaw", pose.yaw}, {"z", pose.z}};
  for (const auto & field : fields) {
    if (!std::isfinite(field.second)) {
      reject(name + "." + field.first, "must be a finite number");
    }
  }

  ValidatedPose result;
  result.transform.linear() = rotationFromRpy(pose.roll, pose.pitch, pose.yaw);
  result.transform.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);

  result.world_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  result.world_max = -result.world_min;
  for (const auto & corner : corners(lo, hi)) {
    const Eigen::Vector3d world = result.transform * corner;
    result.world_min = result.world_min.cwiseMin(world);
    result.world_max = result.world_max.cwiseMax(world);
  }

  if (result.world_min.z() < geometry.table_top_z - kPenetrationToleranceM) {
    reject(
      name + ".z",
      "lowest corner at z=" + number(result.world_min.z()) + " m is below the table top (" +
      number(geometry.table_top_z) + " m); omit z for automatic support height");
  }
  if (result.world_min.x() < geometry.table_min_xy.x() ||
    result.world_max.x() > geometry.table_max_xy.x() ||
    result.world_min.y() < geometry.table_min_xy.y() ||
    result.world_max.y() > geometry.table_max_xy.y())
  {
    reject(
      name,
      "footprint x in [" + number(result.world_min.x()) + ", " + number(result.world_max.x()) +
      "], y in [" + number(result.world_min.y()) + ", " + number(result.world_max.y()) +
      "] leaves the table (x in [" + number(geometry.table_min_xy.x()) + ", " +
      number(geometry.table_max_xy.x()) + "], y in [" + number(geometry.table_min_xy.y()) +
      ", " + number(geometry.table_max_xy.y()) + "])");
  }
  return result;
}

}  // namespace

Eigen::Matrix3d rotationFromRpy(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

double autoSupportZ(
  const Eigen::Matrix3d & rotation, const Eigen::Vector3d & lo, const Eigen::Vector3d & hi,
  double table_top_z)
{
  double lowest = std::numeric_limits<double>::infinity();
  for (const auto & corner : corners(lo, hi)) {
    lowest = std::min(lowest, (rotation * corner).z());
  }
  return table_top_z - lowest + kAutoSupportGapM;
}

ScenePoses resolveScene(const SceneRequest & request, const SceneGeometry & geometry)
{
  const ValidatedPose box = validatePose(
    "scene.box", request.box, -geometry.box_half_extents, geometry.box_half_extents, geometry);
  const ValidatedPose bin = validatePose(
    "scene.bin", request.bin, geometry.bin_min, geometry.bin_max, geometry);

  if (bin.transform.linear()(2, 2) < kMinBinUpAxisZ) {
    reject(
      "scene.bin",
      "up axis z component " + number(bin.transform.linear()(2, 2)) + " is below " +
      number(kMinBinUpAxisZ) + " (tilt over about 20 degrees)");
  }

  // Box footprint in the bin frame: the axis-aligned extent of the box's corners, which
  // is conservative (it may reject a rotated box that would just clear the bin).
  Eigen::Vector2d local_min = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d local_max = -local_min;
  const Eigen::Isometry3d bin_from_world = bin.transform.inverse();
  for (const auto & corner :
    corners(-geometry.box_half_extents, geometry.box_half_extents))
  {
    const Eigen::Vector3d p = bin_from_world * (box.transform * corner);
    local_min = local_min.cwiseMin(p.head<2>());
    local_max = local_max.cwiseMax(p.head<2>());
  }
  const Eigen::Vector2d keep_out_min = geometry.bin_min.head<2>().array() - kBoxBinClearanceM;
  const Eigen::Vector2d keep_out_max = geometry.bin_max.head<2>().array() + kBoxBinClearanceM;
  if ((local_max.array() > keep_out_min.array()).all() &&
    (local_min.array() < keep_out_max.array()).all())
  {
    reject(
      "scene.box",
      "footprint comes within " + number(kBoxBinClearanceM) + " m of the bin footprint " +
      "(box x/y in the bin frame: [" + number(local_min.x()) + ", " + number(local_max.x()) +
      "] x [" + number(local_min.y()) + ", " + number(local_max.y()) + "])");
  }

  ScenePoses result;
  result.box = box.transform;
  result.bin = bin.transform;
  return result;
}

void rejectPoseOverridesWhenDisabled(
  bool enabled, const std::vector<std::string> & parameter_names)
{
  if (enabled) {
    return;
  }
  for (const auto & name : parameter_names) {
    if (name.rfind("scene.box.", 0) == 0 || name.rfind("scene.bin.", 0) == 0) {
      reject(name, "is set but scene.enabled is false; set scene.enabled:=true to use it");
    }
  }
}

}  // namespace mujoco_bridge
