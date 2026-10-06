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

// Private to the library: what the box and the bin detectors share. Back-project a depth image,
// keep the pixels in a height band inside a table region, group them into 8-connected blocks and
// fit the minimum-area rectangle to each block's x-y. Each detector then decides which block is
// its object.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <vector>

#include <sensor_msgs/msg/camera_info.hpp>

namespace mujoco_perception
{
namespace detail
{

struct ScanSpec
{
  double x_min_m, x_max_m, y_min_m, y_max_m;  // table region (world)
  double z_low_m, z_high_m;                   // height band (world)
  double depth_min_m, depth_max_m;
  std::size_t min_pixels;                     // blocks with fewer pixels are dropped
};

// A block and its minimum-area rectangle. The `width` side points along angle_deg (from +x
// toward +y) and the `height` side along angle_deg + 90; angle_deg is in [0, 90).
struct RectBlock
{
  std::vector<std::size_t> pixels;  // indices into the depth image
  Eigen::Vector2d center_xy;
  double width_m = 0.0;
  double height_m = 0.0;
  double angle_deg = 0.0;
};

struct Scan
{
  std::size_t valid_depth_pixels = 0;
  // World coordinates per pixel (only meaningful where the depth was valid).
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  std::vector<RectBlock> blocks;
};

// depth.size() must equal camera_info.width * camera_info.height.
Scan scanBlocks(
  const std::vector<float> & depth, const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Isometry3d & world_from_optical, const ScanSpec & spec);

// The middle value, or the mean of the two middle values for an even count.
double median(std::vector<double> values);

}  // namespace detail
}  // namespace mujoco_perception
