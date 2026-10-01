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

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/camera_info.hpp>

namespace shapes
{
class Mesh;
}

namespace mujoco_perception
{

struct RobotMaskConfig
{
  double depth_tolerance_m = 0.012;
};

struct RobotMaskResult
{
  std::vector<float> predicted_depth;
  std::vector<float> filtered_depth;
  std::vector<uint8_t> mask;
  std::size_t projected_pixels = 0;
  std::size_t masked_pixels = 0;
  std::size_t comparison_pixels = 0;
  std::size_t mismatch_pixels = 0;
  bool valid = false;
};

struct RobotMesh
{
  std::string frame;
  std::shared_ptr<shapes::Mesh> geometry;
};

std::vector<RobotMesh> loadRobotVisualMeshes(const std::string & assets_directory);

class RobotMaskFilter
{
public:
  explicit RobotMaskFilter(std::vector<RobotMesh> meshes);
  ~RobotMaskFilter();

  RobotMaskFilter(const RobotMaskFilter &) = delete;
  RobotMaskFilter & operator=(const RobotMaskFilter &) = delete;

  RobotMaskResult filter(
    const std::vector<float> & observed_depth,
    const sensor_msgs::msg::CameraInfo & camera_info,
    const Eigen::Isometry3d & optical_from_world,
    const std::map<std::string, Eigen::Isometry3d> & world_from_links,
    const RobotMaskConfig & config);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_perception
