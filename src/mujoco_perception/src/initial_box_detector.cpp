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

#include "mujoco_perception/initial_box_detector.hpp"

#include <cmath>
#include <vector>

#include "rectangle_blocks.hpp"

namespace mujoco_perception
{

const char * initialBoxRejectionName(DetectionRejection reason)
{
  switch (reason) {
    case DetectionRejection::kNone: return "NONE";
    case DetectionRejection::kInvalidInput: return "INVALID_INPUT";
    case DetectionRejection::kNoValidDepth: return "NO_VALID_DEPTH";
    case DetectionRejection::kNoBandPixels: return "NO_BOX_BAND_PIXELS";
    case DetectionRejection::kNoMatchingRectangle: return "NO_RECTANGLE_MATCHES_BOX";
    case DetectionRejection::kSeveralMatchingRectangles: return "SEVERAL_BOX_CANDIDATES";
    case DetectionRejection::kNoFloorPixels: return "UNUSED_FOR_BOX";
  }
  return "UNKNOWN";
}

InitialBoxResult detectInitialBox(
  const std::vector<float> & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Isometry3d & world_from_optical,
  const InitialBoxConfig & config)
{
  InitialBoxResult result;
  // The configuration (square box, positive sizes, region) is fixed by the scene and is not
  // checked; only the image size is, because a mismatch would read outside the vector.
  if (camera_info.width == 0 || camera_info.height == 0 ||
    depth.size() != static_cast<std::size_t>(camera_info.width) * camera_info.height)
  {
    result.rejection = DetectionRejection::kInvalidInput;
    return result;
  }

  const double top_z = config.plane_z_m + 2.0 * config.half_extents.z();
  const double side_m = 2.0 * config.half_extents.x();
  const detail::ScanSpec spec{
    config.x_min_m, config.x_max_m, config.y_min_m, config.y_max_m,
    top_z - config.band_below_top_m, top_z + config.band_above_top_m,
    config.depth_min_m, config.depth_max_m, config.min_component_pixels};
  const detail::Scan scan = detail::scanBlocks(depth, camera_info, world_from_optical, spec);
  result.valid_depth_pixels = scan.valid_depth_pixels;
  if (scan.valid_depth_pixels == 0) {
    result.rejection = DetectionRejection::kNoValidDepth;
    return result;
  }

  // Per block: the yaw is the direction of the nearer of the two side directions, folded to
  // [-45, 45) degrees because a square repeats every 90 degrees; then pick the block that has
  // the box's sides.
  std::vector<std::size_t> matching_candidates;
  for (const auto & block : scan.blocks) {
    double yaw_deg = block.angle_deg;
    double along_m = block.width_m;
    double across_m = block.height_m;
    if (block.angle_deg >= 45.0) {
      // The nearest of the two side directions is the \`height\` side, 90 degrees back.
      yaw_deg = block.angle_deg - 90.0;
      along_m = block.height_m;
      across_m = block.width_m;
    }
    BlockCandidate candidate;
    candidate.pixels = block.pixels.size();
    candidate.side_along_m = along_m;
    candidate.side_across_m = across_m;
    candidate.center_xy = block.center_xy;
    candidate.yaw_rad = yaw_deg * M_PI / 180.0;
    candidate.matches =
      std::abs(along_m - side_m) <= config.side_tolerance_m &&
      std::abs(across_m - side_m) <= config.side_tolerance_m;
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

  // Exactly one block: its centre and yaw, and the box height from its top face. If no pixel
  // of the block reaches the top face the height is not measured and the table prior is used.
  const BlockCandidate & box = result.candidates[matching_candidates.front()];
  std::vector<double> top_heights;
  // One candidate per block, in the same order.
  for (const std::size_t index : scan.blocks[matching_candidates.front()].pixels) {
    if (scan.z[index] >= top_z - config.top_face_depth_m) {
      top_heights.push_back(scan.z[index]);
    }
  }
  const double center_z = top_heights.empty() ?
    config.plane_z_m + config.half_extents.z() :
    detail::median(top_heights) - config.half_extents.z();
  result.position = Eigen::Vector3d(box.center_xy.x(), box.center_xy.y(), center_z);
  result.yaw_rad = box.yaw_rad;
  return result;
}

}  // namespace mujoco_perception
