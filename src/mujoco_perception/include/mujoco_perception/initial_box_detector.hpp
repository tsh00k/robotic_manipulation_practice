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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <vector>

#include <sensor_msgs/msg/camera_info.hpp>

namespace mujoco_perception
{

// Detects the box on the table from ONE depth image of a static scene and returns its x, y and
// yaw. It sits beside the older segmentDepth()/estimateBoxPose() pair (geometry_pipeline.hpp),
// which stays for the check after the box has been released; see docs/adr/018 and
// Week 4.1 Stage 5 for why the two differ.
//
// Steps (each one is tested on its own in test_initial_box_detector.cpp):
//   1. back-project every valid depth pixel to a world point (image_geometry ray x depth);
//   2. keep the pixels inside the table region whose world height lies in a band around the
//      box top, so that the table, the bin and the robot at HOME are outside it;
//   3. group the kept pixels into 8-connected blocks (cv::connectedComponentsWithStats);
//   4. per block, drop the height and fit the minimum-area rectangle to the points' x-y
//      (cv::minAreaRect): its centre is the box position and the direction of its sides is the
//      yaw. A vertical face projects to a line and adds no area, so a box seen from the side
//      still gives the rectangle of its footprint;
//   5. a block is the box if both rectangle sides are within `side_tolerance_m` of the box
//      side; exactly one such block is a detection.
// No colour, no robot mask, no ground truth and no history are used.
struct InitialBoxConfig
{
  // Known scene priors.
  double plane_z_m = 0.22;                              // world height of the table top
  Eigen::Vector3d half_extents{0.02, 0.02, 0.02};       // the box; x and y must be equal

  // Height band around the box top (plane_z + 2 * half z): [top - below, top + above].
  // The default lower edge (0.245 m) is above the table (0.22 m) and above the bin's wall tops
  // (0.239 m), so the bin never enters the band; the upper edge is far below the arm at HOME.
  double band_below_top_m = 0.015;
  double band_above_top_m = 0.040;

  // Table region that is searched (world x and y).
  double x_min_m = 0.20;
  double x_max_m = 0.80;
  double y_min_m = -0.40;
  double y_max_m = 0.40;

  double depth_min_m = 0.10;
  double depth_max_m = 2.0;

  // Blocks with fewer pixels are noise, not candidates.
  std::size_t min_component_pixels = 20;
  // Both sides of the rectangle must be within this of the box side (2 * half extent).
  double side_tolerance_m = 0.005;
  // Pixels no lower than (top - this) are the top face, used to measure the box height.
  double top_face_depth_m = 0.005;
};

enum class InitialBoxRejection
{
  kNone,                       // a box was measured
  kInvalidInput,               // the depth image does not match the camera info size
  kNoValidDepth,               // the image has no usable depth at all
  kNoBandPixels,               // nothing in the height band forms a block
  kNoMatchingRectangle,        // blocks exist but none has the box's two sides
  kSeveralMatchingRectangles,  // more than one block looks like the box
};

const char * initialBoxRejectionName(InitialBoxRejection reason);

// One block of the height band, whether or not it was taken for the box. Reported so that a
// miss can be explained from the message alone (which blocks, how big, how far off).
struct InitialBoxCandidate
{
  std::size_t pixels = 0;
  // Rectangle sides, in metres: along the yaw direction and across it.
  double side_along_m = 0.0;
  double side_across_m = 0.0;
  Eigen::Vector2d center_xy = Eigen::Vector2d::Zero();
  double yaw_rad = 0.0;       // folded to [-45, 45) degrees
  bool matches_box = false;
};

struct InitialBoxResult
{
  InitialBoxRejection rejection = InitialBoxRejection::kNone;
  // Set only when a box was measured. position is the box centre: x, y from the rectangle,
  // z from the height of the top face minus the half height.
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  // Rotation about +z, folded to [-45, 45) degrees: a square repeats every 90 degrees, so only
  // the orientation modulo 90 degrees exists to be measured.
  double yaw_rad = 0.0;
  std::size_t valid_depth_pixels = 0;
  std::vector<InitialBoxCandidate> candidates;

  bool measured() const {return rejection == InitialBoxRejection::kNone;}
};

// `depth` is row-major, camera_info.height rows of camera_info.width metres, NaN or <= 0 for no
// measurement; world_from_optical is the camera optical frame in the world.
InitialBoxResult detectInitialBox(
  const std::vector<float> & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Isometry3d & world_from_optical,
  const InitialBoxConfig & config);

}  // namespace mujoco_perception
