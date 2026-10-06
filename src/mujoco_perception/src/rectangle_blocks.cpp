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

#include "rectangle_blocks.hpp"

#include <image_geometry/pinhole_camera_model.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace mujoco_perception
{
namespace detail
{

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

Scan scanBlocks(
  const std::vector<float> & depth, const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Isometry3d & world_from_optical, const ScanSpec & spec)
{
  Scan scan;
  const std::size_t width = camera_info.width;
  const std::size_t height = camera_info.height;
  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info);

  // World point per valid pixel, and the band mask. Double, not float: the minimum-area
  // rectangle of a nearly square block reacts to the last digits of its points, and the band
  // edges are compared against these values.
  scan.x.assign(depth.size(), 0.0);
  scan.y.assign(depth.size(), 0.0);
  scan.z.assign(depth.size(), 0.0);
  cv::Mat band = cv::Mat::zeros(static_cast<int>(height), static_cast<int>(width), CV_8UC1);
  for (std::size_t v = 0; v < height; ++v) {
    for (std::size_t u = 0; u < width; ++u) {
      const std::size_t index = v * width + u;
      const float d = depth[index];
      if (!std::isfinite(d) || d < spec.depth_min_m || d > spec.depth_max_m) {
        continue;
      }
      ++scan.valid_depth_pixels;
      const cv::Point3d ray = camera_model.projectPixelTo3dRay(
        cv::Point2d(static_cast<double>(u), static_cast<double>(v)));
      const Eigen::Vector3d point = world_from_optical *
        Eigen::Vector3d(ray.x * d, ray.y * d, d);
      scan.x[index] = point.x();
      scan.y[index] = point.y();
      scan.z[index] = point.z();
      if (point.x() >= spec.x_min_m && point.x() <= spec.x_max_m &&
        point.y() >= spec.y_min_m && point.y() <= spec.y_max_m &&
        point.z() >= spec.z_low_m && point.z() <= spec.z_high_m)
      {
        band.at<uint8_t>(static_cast<int>(v), static_cast<int>(u)) = 255;
      }
    }
  }
  if (scan.valid_depth_pixels == 0) {
    return scan;
  }

  // Blocks of connected pixels.
  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int label_count = cv::connectedComponentsWithStats(band, labels, stats, centroids, 8);
  std::vector<std::vector<std::size_t>> members(static_cast<std::size_t>(label_count));
  for (std::size_t v = 0; v < height; ++v) {
    for (std::size_t u = 0; u < width; ++u) {
      const int label = labels.at<int>(static_cast<int>(v), static_cast<int>(u));
      if (label > 0 && static_cast<std::size_t>(stats.at<int>(label, cv::CC_STAT_AREA)) >=
        spec.min_pixels)
      {
        members[static_cast<std::size_t>(label)].push_back(v * width + u);
      }
    }
  }

  // The minimum-area rectangle of each block.
  for (auto & block_pixels : members) {
    if (block_pixels.empty()) {
      continue;
    }
    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const std::size_t index : block_pixels) {
      mean_x += scan.x[index];
      mean_y += scan.y[index];
    }
    mean_x /= static_cast<double>(block_pixels.size());
    mean_y /= static_cast<double>(block_pixels.size());
    // Coordinates relative to the block's mean, in metres: cv::minAreaRect works in float, and
    // the offset keeps the numbers small.
    std::vector<cv::Point2f> points;
    points.reserve(block_pixels.size());
    for (const std::size_t index : block_pixels) {
      points.emplace_back(
        static_cast<float>(scan.x[index] - mean_x), static_cast<float>(scan.y[index] - mean_y));
    }
    const cv::RotatedRect rect = cv::minAreaRect(points);

    // OpenCV (4.5.1 and later) reports angle in (0, 90]: the `width` side points along `angle`
    // measured from +x toward +y and the `height` side along angle + 90. The points are world
    // (x, y), so this is the direction about +z. An angle of 90 is the same rectangle as 0 with
    // the two sides exchanged.
    RectBlock block;
    block.angle_deg = rect.angle;
    block.width_m = rect.size.width;
    block.height_m = rect.size.height;
    if (block.angle_deg >= 90.0 - 1e-6) {
      block.angle_deg = 0.0;
      std::swap(block.width_m, block.height_m);
    }
    block.center_xy = Eigen::Vector2d(mean_x + rect.center.x, mean_y + rect.center.y);
    block.pixels = std::move(block_pixels);
    scan.blocks.push_back(std::move(block));
  }
  return scan;
}

}  // namespace detail
}  // namespace mujoco_perception
