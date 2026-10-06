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

#include <image_geometry/pinhole_camera_model.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace mujoco_perception
{

namespace
{

bool validConfig(const InitialBoxConfig & c)
{
  return std::isfinite(c.plane_z_m) && c.half_extents.allFinite() &&
         (c.half_extents.array() > 0.0).all() &&
         std::abs(c.half_extents.x() - c.half_extents.y()) < 1e-6 &&
         c.band_below_top_m >= 0.0 && c.band_above_top_m > 0.0 &&
         c.x_min_m < c.x_max_m && c.y_min_m < c.y_max_m && c.depth_min_m > 0.0 &&
         c.depth_max_m > c.depth_min_m && c.min_component_pixels > 0 &&
         c.side_tolerance_m > 0.0 && c.top_face_depth_m > 0.0;
}

// OpenCV (4.5.1 and later) reports angle in (0, 90]: the `width` side points along `angle`
// measured from +x toward +y and the `height` side along angle + 90. The points are passed as
// world (x, y), so this is the yaw about +z. An angle of 90 is the same rectangle as 0 with the
// two sides exchanged. Returns the direction in [-45, 45) and the sides along and across it.
void foldRectangle(
  const cv::RotatedRect & rect, double & yaw_deg, double & along, double & across)
{
  double angle = rect.angle;
  double width = rect.size.width;
  double height = rect.size.height;
  if (angle >= 90.0 - 1e-6) {
    angle = 0.0;
    std::swap(width, height);
  }
  if (angle >= 45.0) {
    // The nearest of the two side directions is the `height` side, 90 degrees back.
    yaw_deg = angle - 90.0;
    along = height;
    across = width;
  } else {
    yaw_deg = angle;
    along = width;
    across = height;
  }
}

}  // namespace

namespace detail
{

// Median in the usual sense: the middle value, or the mean of the two middle values when the
// count is even (taking the upper one would bias an even-sized set high by half the gap).
double median(std::vector<double> values)
{
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2 == 1) {
    return *middle;
  }
  const double lower = *std::max_element(values.begin(), middle);
  return 0.5 * (lower + *middle);
}

}  // namespace detail

const char * initialBoxRejectionName(InitialBoxRejection reason)
{
  switch (reason) {
    case InitialBoxRejection::kNone: return "NONE";
    case InitialBoxRejection::kInvalidInput: return "INVALID_INPUT";
    case InitialBoxRejection::kNoValidDepth: return "NO_VALID_DEPTH";
    case InitialBoxRejection::kNoBandPixels: return "NO_BOX_BAND_PIXELS";
    case InitialBoxRejection::kNoMatchingRectangle: return "NO_RECTANGLE_MATCHES_BOX";
    case InitialBoxRejection::kSeveralMatchingRectangles: return "SEVERAL_BOX_CANDIDATES";
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
  const std::size_t width = camera_info.width;
  const std::size_t height = camera_info.height;
  if (!validConfig(config) || width == 0 || height == 0 || camera_info.k[0] <= 0.0 ||
    camera_info.k[4] <= 0.0 || depth.size() != width * height ||
    !world_from_optical.matrix().allFinite())
  {
    result.rejection = InitialBoxRejection::kInvalidInput;
    return result;
  }

  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info);

  const double top_z = config.plane_z_m + 2.0 * config.half_extents.z();
  const double band_low = top_z - config.band_below_top_m;
  const double band_high = top_z + config.band_above_top_m;
  const double side_m = 2.0 * config.half_extents.x();

  // Step 1 and 2: world point per valid pixel, band mask.
  // Double, not float: the minimum-area rectangle of a nearly square block reacts to the last
  // digits of its points, and the band edges are compared against these values.
  std::vector<double> world_x(depth.size(), 0.0);
  std::vector<double> world_y(depth.size(), 0.0);
  std::vector<double> world_z(depth.size(), 0.0);
  cv::Mat band = cv::Mat::zeros(static_cast<int>(height), static_cast<int>(width), CV_8UC1);
  for (std::size_t v = 0; v < height; ++v) {
    for (std::size_t u = 0; u < width; ++u) {
      const std::size_t index = v * width + u;
      const float d = depth[index];
      if (!std::isfinite(d) || d < config.depth_min_m || d > config.depth_max_m) {
        continue;
      }
      ++result.valid_depth_pixels;
      const cv::Point3d ray = camera_model.projectPixelTo3dRay(
        cv::Point2d(static_cast<double>(u), static_cast<double>(v)));
      const Eigen::Vector3d point = world_from_optical *
        Eigen::Vector3d(ray.x * d, ray.y * d, d);
      world_x[index] = point.x();
      world_y[index] = point.y();
      world_z[index] = point.z();
      if (point.x() >= config.x_min_m && point.x() <= config.x_max_m &&
        point.y() >= config.y_min_m && point.y() <= config.y_max_m &&
        point.z() >= band_low && point.z() <= band_high)
      {
        band.at<uint8_t>(static_cast<int>(v), static_cast<int>(u)) = 255;
      }
    }
  }
  if (result.valid_depth_pixels == 0) {
    result.rejection = InitialBoxRejection::kNoValidDepth;
    return result;
  }

  // Step 3: blocks of connected pixels.
  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int label_count = cv::connectedComponentsWithStats(band, labels, stats, centroids, 8);
  std::vector<std::vector<std::size_t>> members(static_cast<std::size_t>(label_count));
  for (std::size_t v = 0; v < height; ++v) {
    for (std::size_t u = 0; u < width; ++u) {
      const int label = labels.at<int>(static_cast<int>(v), static_cast<int>(u));
      if (label > 0 && static_cast<std::size_t>(stats.at<int>(label, cv::CC_STAT_AREA)) >=
        config.min_component_pixels)
      {
        members[static_cast<std::size_t>(label)].push_back(v * width + u);
      }
    }
  }

  // Step 4 and 5: rectangle per block, then pick the one that has the box's sides.
  std::vector<std::size_t> matching_candidates;
  // The pixels of each candidate, parallel to result.candidates.
  std::vector<const std::vector<std::size_t> *> candidate_pixels;
  for (const auto & block : members) {
    if (block.empty()) {
      continue;
    }
    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const std::size_t index : block) {
      mean_x += world_x[index];
      mean_y += world_y[index];
    }
    mean_x /= static_cast<double>(block.size());
    mean_y /= static_cast<double>(block.size());
    // Coordinates relative to the block's mean, in metres: cv::minAreaRect works in float, and
    // the offset keeps the numbers small. (Its result was checked to be the same to better than
    // 1e-5 relative for metres and for millimetres, so the unit does not matter here.)
    std::vector<cv::Point2f> points;
    points.reserve(block.size());
    for (const std::size_t index : block) {
      points.emplace_back(
        static_cast<float>(world_x[index] - mean_x), static_cast<float>(world_y[index] - mean_y));
    }
    const cv::RotatedRect rect = cv::minAreaRect(points);
    double yaw_deg = 0.0;
    double along_m = 0.0;
    double across_m = 0.0;
    foldRectangle(rect, yaw_deg, along_m, across_m);

    InitialBoxCandidate candidate;
    candidate.pixels = block.size();
    candidate.side_along_m = along_m;
    candidate.side_across_m = across_m;
    candidate.center_xy = Eigen::Vector2d(mean_x + rect.center.x, mean_y + rect.center.y);
    candidate.yaw_rad = yaw_deg * M_PI / 180.0;
    candidate.matches_box =
      std::abs(candidate.side_along_m - side_m) <= config.side_tolerance_m &&
      std::abs(candidate.side_across_m - side_m) <= config.side_tolerance_m;
    result.candidates.push_back(candidate);
    candidate_pixels.push_back(&block);
    if (candidate.matches_box) {
      matching_candidates.push_back(result.candidates.size() - 1);
    }
  }

  if (result.candidates.empty()) {
    result.rejection = InitialBoxRejection::kNoBandPixels;
    return result;
  }
  if (matching_candidates.empty()) {
    result.rejection = InitialBoxRejection::kNoMatchingRectangle;
    return result;
  }
  if (matching_candidates.size() > 1) {
    result.rejection = InitialBoxRejection::kSeveralMatchingRectangles;
    return result;
  }

  // Exactly one block: its centre and yaw, and the box height from its top face. If no pixel
  // of the block reaches the top face the height is not measured and the table prior is used.
  const InitialBoxCandidate & box = result.candidates[matching_candidates.front()];
  std::vector<double> top_heights;
  for (const std::size_t index : *candidate_pixels[matching_candidates.front()]) {
    if (world_z[index] >= top_z - config.top_face_depth_m) {
      top_heights.push_back(world_z[index]);
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
