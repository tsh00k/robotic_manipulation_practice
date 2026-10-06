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

#include "mujoco_perception/block_detection.hpp"

namespace mujoco_perception
{

// Detects the bin on the table from ONE depth image of a static scene and returns its x, y, yaw
// and inner floor height (Week 4.1 Stage 6). Same steps as detectInitialBox() (height band,
// 8-connected blocks, minimum-area rectangle of each block's x-y), with the bin's numbers:
//   - the band reaches from just below the floor surface to just above the wall tops, so the
//     whole bin footprint is one block, and a box standing on the table is a different block;
//   - the block is the bin when its two rectangle sides are the bin's outer sides within
//     `side_tolerance_m`; exactly one such block is a detection;
//   - yaw is the direction of the LONG side (the bin's own x axis), folded to [-90, 90) degrees:
//     a rectangle repeats every 180 degrees. The sides differ by only 10 mm, so noise can
//     swap them and turn the yaw by 90 degrees; the tests count that;
//   - the floor height is the median height of the pixels in the middle of the bin, which is
//     smaller than the inner opening so that the walls are not in it.
// The bin must be flat on the table, empty, and in the open: a box inside it, or a tilted bin,
// is not supported. No colour, no robot mask, no ground truth and no history are used.
struct InitialBinConfig
{
  // Known scene priors.
  double plane_z_m = 0.22;                // world height of the table top
  double floor_above_plane_m = 0.007;     // inner floor surface above the table (bin origin)
  double wall_height_m = 0.012;           // walls above the inner floor
  double outer_long_m = 0.152;            // outer footprint
  double outer_short_m = 0.142;

  // Height band: [floor - below, wall top + above] = [0.2235, 0.243] m with the defaults.
  double band_below_floor_m = 0.0035;
  double band_above_wall_top_m = 0.004;

  // Table region that is searched (world x and y).
  double x_min_m = 0.20;
  double x_max_m = 0.80;
  double y_min_m = -0.40;
  double y_max_m = 0.40;

  double depth_min_m = 0.10;
  double depth_max_m = 2.0;

  std::size_t min_component_pixels = 20;
  // Both rectangle sides must be within this of the bin's sides.
  double side_tolerance_m = 0.008;
  // The middle of the bin, half extents along the long and the short side.
  double floor_half_long_m = 0.056;
  double floor_half_short_m = 0.051;
  std::size_t min_floor_pixels = 20;
};

struct InitialBinResult
{
  DetectionRejection rejection = DetectionRejection::kNone;
  // Set only when a bin was measured. position is the centre of the inner floor: x, y from the
  // rectangle, z from the middle-of-the-bin pixels (the floor surface height).
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  // Rotation about +z of the long side, in [-pi/2, pi/2).
  double yaw_rad = 0.0;
  std::size_t valid_depth_pixels = 0;
  std::vector<BlockCandidate> candidates;

  bool measured() const {return rejection == DetectionRejection::kNone;}
};

// The names the InitialBinPose message carries ("NO_BIN_BAND_PIXELS", ...).
const char * initialBinRejectionName(DetectionRejection reason);

// `depth` is row-major, camera_info.height rows of camera_info.width metres, NaN or <= 0 for no
// measurement; world_from_optical is the camera optical frame in the world.
InitialBinResult detectInitialBin(
  const std::vector<float> & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Isometry3d & world_from_optical,
  const InitialBinConfig & config);

}  // namespace mujoco_perception
