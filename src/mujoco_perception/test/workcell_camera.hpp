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

// The workcell camera as the simulator defines it, and a synthetic scene whose depth image is
// computed exactly, for tests and offline tools.
//
// Camera: robot_description/mujoco/franka_emika_panda/pick_place_scene.xml, body camera_link
// pos="0.5 -0.45 1.0" xyaxes="1 0 0 0 0.857 0.514", camera fovy=50, offscreen 320 x 240; the
// intrinsics are the bridge's (focal = height / (2 tan(fovy / 2)), principal point at the
// centre of the pixel grid). If the MJCF camera changes, change the constants here and the
// copy in initial_pose_eval.py together.
//
// Scene: a horizontal table plane plus oriented boxes. depth(u, v) is the optical z of the
// first surface hit by the pixel's ray, so a test knows the true answer by construction instead
// of obtaining it from the detector it is testing.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"

namespace mujoco_perception_test
{

constexpr int kImageWidth = 320;
constexpr int kImageHeight = 240;
constexpr double kFovyDeg = 50.0;

inline sensor_msgs::msg::CameraInfo workcellCameraInfo()
{
  const double focal = kImageHeight / (2.0 * std::tan(kFovyDeg * M_PI / 180.0 / 2.0));
  const double cx = (kImageWidth - 1) / 2.0;
  const double cy = (kImageHeight - 1) / 2.0;
  sensor_msgs::msg::CameraInfo info;
  info.width = kImageWidth;
  info.height = kImageHeight;
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

// Optical frame (x right, y down, z forward) in the world. MuJoCo's camera looks along its own
// -z with +y up, so the optical axes are (x, -y, -z) of the MuJoCo camera.
inline Eigen::Isometry3d workcellWorldFromOptical()
{
  const Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d y_axis = Eigen::Vector3d(0.0, 0.857, 0.514).normalized();
  const Eigen::Vector3d z_axis = x_axis.cross(y_axis);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear().col(0) = x_axis;
  result.linear().col(1) = -y_axis;
  result.linear().col(2) = -z_axis;
  result.translation() = Eigen::Vector3d(0.5, -0.45, 1.0);
  return result;
}

// A box rotated about the world z axis only (all boxes in this project stand flat).
struct SceneBox
{
  Eigen::Vector3d center;
  double yaw_rad = 0.0;
  Eigen::Vector3d half_extents;
};

// Depth image of the table plane z = table_z and the boxes, in metres, row-major.
inline std::vector<float> renderDepth(const std::vector<SceneBox> & boxes, double table_z)
{
  const auto info = workcellCameraInfo();
  const Eigen::Isometry3d world_from_optical = workcellWorldFromOptical();
  const Eigen::Vector3d origin = world_from_optical.translation();
  std::vector<float> depth(
    static_cast<std::size_t>(kImageWidth) * kImageHeight, std::numeric_limits<float>::quiet_NaN());
  for (int v = 0; v < kImageHeight; ++v) {
    for (int u = 0; u < kImageWidth; ++u) {
      // Optical z of a point on the ray is the parameter t, because the ray's z component is 1.
      const Eigen::Vector3d ray_optical((u - info.k[2]) / info.k[0], (v - info.k[5]) / info.k[4],
        1.0);
      const Eigen::Vector3d direction = world_from_optical.linear() * ray_optical;
      double best = std::numeric_limits<double>::infinity();
      if (direction.z() < 0.0) {
        const double t = (table_z - origin.z()) / direction.z();
        if (t > 0.0) {
          best = t;
        }
      }
      for (const auto & box : boxes) {
        const Eigen::Matrix3d rotation =
          Eigen::AngleAxisd(box.yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Vector3d local_origin = rotation.transpose() * (origin - box.center);
        const Eigen::Vector3d local_direction = rotation.transpose() * direction;
        double t_enter = -std::numeric_limits<double>::infinity();
        double t_exit = std::numeric_limits<double>::infinity();
        bool parallel_outside = false;
        for (int axis = 0; axis < 3; ++axis) {
          if (std::abs(local_direction[axis]) < 1e-12) {
            parallel_outside = parallel_outside ||
              std::abs(local_origin[axis]) > box.half_extents[axis];
            continue;
          }
          const double t1 = (-box.half_extents[axis] - local_origin[axis]) / local_direction[axis];
          const double t2 = (box.half_extents[axis] - local_origin[axis]) / local_direction[axis];
          t_enter = std::max(t_enter, std::min(t1, t2));
          t_exit = std::min(t_exit, std::max(t1, t2));
        }
        if (!parallel_outside && t_enter <= t_exit && t_exit > 0.0 && t_enter > 0.0) {
          best = std::min(best, t_enter);
        }
      }
      if (std::isfinite(best)) {
        depth[static_cast<std::size_t>(v) * kImageWidth + u] = static_cast<float>(best);
      }
    }
  }
  return depth;
}

// The bin of pick_place_bin_scene.xml as five boxes (floor and four walls), at yaw `yaw_rad`
// about its origin (the centre of the inner floor surface, at world height floor_z).
inline std::vector<SceneBox> binBoxes(double x, double y, double floor_z, double yaw_rad)
{
  const Eigen::Matrix2d rotation = Eigen::Rotation2Dd(yaw_rad).toRotationMatrix();
  const auto place = [&](double lx, double ly, double lz, double hx, double hy, double hz) {
      const Eigen::Vector2d xy = Eigen::Vector2d(x, y) + rotation * Eigen::Vector2d(lx, ly);
      return SceneBox{Eigen::Vector3d(xy.x(), xy.y(), floor_z + lz), yaw_rad,
      Eigen::Vector3d(hx, hy, hz)};
    };
  return {
    place(0.0, 0.0, -0.003, 0.076, 0.071, 0.003),    // floor, 6 mm thick below z = 0
    place(-0.073, 0.0, 0.006, 0.003, 0.071, 0.006),  // walls, 6 mm thick, 12 mm tall
    place(0.073, 0.0, 0.006, 0.003, 0.071, 0.006),
    place(0.0, -0.068, 0.006, 0.07, 0.003, 0.006),
    place(0.0, 0.068, 0.006, 0.07, 0.003, 0.006)};
}

}  // namespace mujoco_perception_test
