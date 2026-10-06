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

#include "mujoco_perception/initial_bin_detector.hpp"

#include <cmath>
#include <vector>

#include "rectangle_blocks.hpp"

namespace mujoco_perception
{

const char * initialBinRejectionName(DetectionRejection reason)
{
  switch (reason) {
    case DetectionRejection::kNone: return "NONE";
    case DetectionRejection::kInvalidInput: return "INVALID_INPUT";
    case DetectionRejection::kNoValidDepth: return "NO_VALID_DEPTH";
    case DetectionRejection::kNoBandPixels: return "NO_BIN_BAND_PIXELS";
    case DetectionRejection::kNoMatchingRectangle: return "NO_RECTANGLE_MATCHES_BIN";
    case DetectionRejection::kSeveralMatchingRectangles: return "SEVERAL_BIN_CANDIDATES";
    case DetectionRejection::kNoFloorPixels: return "NO_BIN_FLOOR_PIXELS";
  }
  return "UNKNOWN";
}

InitialBinResult detectInitialBin(
  const std::vector<float> & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Isometry3d & world_from_optical,
  const InitialBinConfig & config)
{
  InitialBinResult result;
  if (camera_info.width == 0 || camera_info.height == 0 ||
    depth.size() != static_cast<std::size_t>(camera_info.width) * camera_info.height)
  {
    result.rejection = DetectionRejection::kInvalidInput;
    return result;
  }

  const double floor_z = config.plane_z_m + config.floor_above_plane_m;
  const detail::ScanSpec spec{
    config.x_min_m, config.x_max_m, config.y_min_m, config.y_max_m,
    floor_z - config.band_below_floor_m,
    floor_z + config.wall_height_m + config.band_above_wall_top_m,
    config.depth_min_m, config.depth_max_m, config.min_component_pixels};
  const detail::Scan scan = detail::scanBlocks(depth, camera_info, world_from_optical, spec);
  result.valid_depth_pixels = scan.valid_depth_pixels;
  if (scan.valid_depth_pixels == 0) {
    result.rejection = DetectionRejection::kNoValidDepth;
    return result;
  }

  // Per block: the long side's direction (the bin's x axis), folded to [-90, 90) degrees, and
  // the sides along it and across it.
  std::vector<std::size_t> matching_candidates;
  for (const auto & block : scan.blocks) {
    const bool width_is_long = block.width_m >= block.height_m;
    double direction_deg = width_is_long ? block.angle_deg : block.angle_deg + 90.0;
    if (direction_deg >= 90.0) {
      direction_deg -= 180.0;
    }
    BlockCandidate candidate;
    candidate.pixels = block.pixels.size();
    candidate.side_along_m = width_is_long ? block.width_m : block.height_m;
    candidate.side_across_m = width_is_long ? block.height_m : block.width_m;
    candidate.center_xy = block.center_xy;
    candidate.yaw_rad = direction_deg * M_PI / 180.0;
    candidate.matches =
      std::abs(candidate.side_along_m - config.outer_long_m) <= config.side_tolerance_m &&
      std::abs(candidate.side_across_m - config.outer_short_m) <= config.side_tolerance_m;
    result.candidates.push_back(candidate);
    if (candidate.matches) {
      matching_candidates.push_back(result.candidates.size() - 1);
    }
  }

  if (result.candidates.empty()) {
    result.rejection = DetectionRejection::kNoBandPixels;
    return result;
  }
  if (matching_candidates.empty()) {
    result.rejection = DetectionRejection::kNoMatchingRectangle;
    return result;
  }
  if (matching_candidates.size() > 1) {
    result.rejection = DetectionRejection::kSeveralMatchingRectangles;
    return result;
  }

  // Exactly one block. The floor height from the pixels in the middle of the bin, found in the
  // rectangle's own frame (one candidate per block, in the same order).
  const std::size_t chosen = matching_candidates.front();
  const BlockCandidate & bin = result.candidates[chosen];
  const double c = std::cos(bin.yaw_rad);
  const double s = std::sin(bin.yaw_rad);
  std::vector<double> floor_heights;
  for (const std::size_t index : scan.blocks[chosen].pixels) {
    const double dx = scan.x[index] - bin.center_xy.x();
    const double dy = scan.y[index] - bin.center_xy.y();
    const double along = dx * c + dy * s;
    const double across = -dx * s + dy * c;
    if (std::abs(along) < config.floor_half_long_m &&
      std::abs(across) < config.floor_half_short_m)
    {
      floor_heights.push_back(scan.z[index]);
    }
  }
  if (floor_heights.size() < config.min_floor_pixels) {
    result.rejection = DetectionRejection::kNoFloorPixels;
    return result;
  }
  result.position = Eigen::Vector3d(
    bin.center_xy.x(), bin.center_xy.y(), detail::median(floor_heights));
  result.yaw_rad = bin.yaw_rad;
  return result;
}

}  // namespace mujoco_perception
