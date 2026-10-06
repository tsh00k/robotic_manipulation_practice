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

#include "mujoco_perception/initial_pose_estimator.hpp"

namespace mujoco_perception
{

const char * initialPoseStateName(InitialPoseState state)
{
  switch (state) {
    case InitialPoseState::kWarmingUp:
      return "WARMING_UP";
    case InitialPoseState::kNotMeasured:
      return "NOT_MEASURED";
    case InitialPoseState::kMeasured:
      return "MEASURED";
  }
  return "UNKNOWN";
}

InitialPoseEstimator::InitialPoseEstimator(
  std::size_t frames, const InitialBoxConfig & box_config, const InitialBinConfig & bin_config)
: window_(frames), box_config_(box_config), bin_config_(bin_config)
{
}

InitialPoseEstimate InitialPoseEstimator::update(
  const std::vector<float> & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Isometry3d & world_from_optical)
{
  window_.push(depth);
  InitialPoseEstimate estimate;
  estimate.frames_averaged = window_.size();
  estimate.frames_required = window_.capacity();
  if (!window_.full()) {
    return estimate;
  }
  const std::vector<float> mean = window_.mean();
  estimate.box = detectInitialBox(mean, camera_info, world_from_optical, box_config_);
  estimate.bin = detectInitialBin(mean, camera_info, world_from_optical, bin_config_);
  return estimate;
}

void InitialPoseEstimator::reset()
{
  window_.clear();
}

}  // namespace mujoco_perception
