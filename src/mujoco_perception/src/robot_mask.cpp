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

#include "mujoco_perception/robot_mask.hpp"

#include <geometric_shapes/mesh_operations.h>
#include <moveit/mesh_filter/mesh_filter.h>
#include <moveit/mesh_filter/stereo_camera_model.h>

#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

namespace mujoco_perception
{

std::vector<RobotMesh> loadRobotVisualMeshes(const std::string & assets_directory)
{
  const std::filesystem::path assets(assets_directory);
  std::vector<RobotMesh> meshes;
  const std::map<std::string, std::size_t> expected_counts{
    {"link0", 11}, {"link1", 1}, {"link2", 1}, {"link3", 4},
    {"link4", 4}, {"link5", 3}, {"link6", 17}, {"link7", 8},
    {"hand", 5}, {"left_finger", 2}, {"right_finger", 2}};
  for (const auto & entry : expected_counts) {
    const std::string prefix = entry.first == "left_finger" || entry.first == "right_finger" ?
      "finger_" : entry.first == "link1" || entry.first == "link2" ?
      entry.first + ".obj" : entry.first + "_";
    std::size_t count = 0;
    for (const auto & file : std::filesystem::directory_iterator(assets)) {
      const std::string name = file.path().filename().string();
      const bool matches = prefix.find(".obj") != std::string::npos ?
        name == prefix : name.compare(0, prefix.size(), prefix) == 0 &&
        file.path().extension() == ".obj" && name.find("collision") == std::string::npos;
      if (matches) {
        std::shared_ptr<shapes::Mesh> geometry(
          shapes::createMeshFromResource("file://" + file.path().string()));
        if (!geometry || geometry->triangle_count == 0) {
          throw std::runtime_error("cannot load robot mesh: " + file.path().string());
        }
        meshes.push_back({entry.first, std::move(geometry)});
        ++count;
      }
    }
    if (count != entry.second) {
      throw std::runtime_error("robot visual mesh count changed for " + entry.first);
    }
  }
  return meshes;
}

class RobotMaskFilter::Impl
{
public:
  using Filter = mesh_filter::MeshFilter<mesh_filter::StereoCameraModel>;

  explicit Impl(std::vector<RobotMesh> meshes)
  : meshes_(std::move(meshes)) {}

  void prepare(
    const sensor_msgs::msg::CameraInfo & info,
    const Eigen::Isometry3d & optical_from_world,
    const std::map<std::string, Eigen::Isometry3d> & world_from_links)
  {
    poses_.clear();
    if (!filter_) {
      mesh_filter::StereoCameraModel::Parameters camera(
        info.width, info.height, 0.05F, 10.0F,
        static_cast<float>(info.k[0]), static_cast<float>(info.k[4]),
        static_cast<float>(info.k[2]), static_cast<float>(info.k[5]), 0.1F, 0.125F);
      filter_ = std::make_unique<Filter>(
        [this](mesh_filter::MeshHandle handle, Eigen::Isometry3d & pose) {
          const auto found = poses_.find(handle);
          if (found == poses_.end()) {
            return false;
          }
          pose = found->second;
          return true;
        }, camera);
      filter_->setPaddingScale(0.0F);
      filter_->setPaddingOffset(0.0F);
      handles_.clear();
      for (const auto & mesh : meshes_) {
        handles_.push_back(filter_->addMesh(*mesh.geometry));
      }
      width_ = info.width;
      height_ = info.height;
      fx_ = info.k[0];
      fy_ = info.k[4];
      cx_ = info.k[2];
      cy_ = info.k[5];
    }
    for (std::size_t index = 0; index < meshes_.size(); ++index) {
      poses_.emplace(
        handles_[index], optical_from_world * world_from_links.at(meshes_[index].frame));
    }
  }

  bool matchesCalibration(const sensor_msgs::msg::CameraInfo & info) const
  {
    return !filter_ || (
      width_ == info.width && height_ == info.height &&
      fx_ == info.k[0] && fy_ == info.k[4] &&
      cx_ == info.k[2] && cy_ == info.k[5]);
  }

  std::vector<float> render(const std::vector<float> & observed_depth)
  {
    filter_->filter(observed_depth.data(), GL_FLOAT, true);
    std::vector<float> predicted(observed_depth.size());
    filter_->getModelDepth(predicted.data());
    for (float & depth : predicted) {
      if (!std::isfinite(depth) || depth <= 0.0F) {
        depth = std::numeric_limits<float>::quiet_NaN();
      }
    }
    return predicted;
  }

  const std::vector<RobotMesh> meshes_;
  std::vector<mesh_filter::MeshHandle> handles_;
  std::map<mesh_filter::MeshHandle, Eigen::Isometry3d> poses_;
  std::unique_ptr<Filter> filter_;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_ = 0.0;
  double cy_ = 0.0;
};

RobotMaskFilter::RobotMaskFilter(std::vector<RobotMesh> meshes)
: impl_(std::make_unique<Impl>(std::move(meshes))) {}

RobotMaskFilter::~RobotMaskFilter() = default;

RobotMaskResult RobotMaskFilter::filter(
  const std::vector<float> & observed_depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Isometry3d & optical_from_world,
  const std::map<std::string, Eigen::Isometry3d> & world_from_links,
  const RobotMaskConfig & config)
{
  RobotMaskResult result;
  const std::size_t pixel_count = static_cast<std::size_t>(camera_info.width) * camera_info.height;
  if (camera_info.width == 0 || camera_info.height == 0 ||
    observed_depth.size() != pixel_count || camera_info.k[0] <= 0.0 ||
    camera_info.k[4] <= 0.0 || !optical_from_world.matrix().allFinite() ||
    !std::isfinite(config.depth_tolerance_m) || config.depth_tolerance_m <= 0.0 ||
    impl_->meshes_.empty())
  {
    return result;
  }
  for (const auto & mesh : impl_->meshes_) {
    const auto pose = world_from_links.find(mesh.frame);
    if (!mesh.geometry || pose == world_from_links.end() ||
      !pose->second.matrix().allFinite())
    {
      return result;
    }
  }
  if (!impl_->matchesCalibration(camera_info)) {
    throw std::runtime_error("camera calibration changed during robot mask session");
  }
  impl_->prepare(camera_info, optical_from_world, world_from_links);
  result.predicted_depth = impl_->render(observed_depth);
  result.filtered_depth = observed_depth;
  result.mask.assign(pixel_count, 0);
  for (std::size_t index = 0; index < pixel_count; ++index) {
    const float predicted = result.predicted_depth[index];
    if (!std::isfinite(predicted)) {
      continue;
    }
    ++result.projected_pixels;
    const float observed = observed_depth[index];
    if (!std::isfinite(observed) || observed <= 0.0F) {
      continue;
    }
    ++result.comparison_pixels;
    const double difference = static_cast<double>(observed) - predicted;
    if (std::abs(difference) <= config.depth_tolerance_m) {
      result.mask[index] = 255;
      result.filtered_depth[index] = std::numeric_limits<float>::quiet_NaN();
      ++result.masked_pixels;
    } else if (difference > config.depth_tolerance_m) {
      ++result.mismatch_pixels;
    }
  }
  result.valid = true;
  return result;
}

}  // namespace mujoco_perception
