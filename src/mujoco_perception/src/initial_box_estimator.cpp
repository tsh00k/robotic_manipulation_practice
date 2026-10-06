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

#include "mujoco_perception/initial_box_estimator.hpp"

namespace mujoco_perception
{

const char * initialBoxStateName(InitialBoxState state)
{
  switch (state) {
    case InitialBoxState::kWarmingUp:
      return "WARMING_UP";
    case InitialBoxState::kNotMeasured:
      return "NOT_MEASURED";
    case InitialBoxState::kMeasured:
      return "MEASURED";
  }
  return "UNKNOWN";
}

InitialBoxEstimator::InitialBoxEstimator(std::size_t frames, const InitialBoxConfig & config)
: window_(frames), config_(config)
{
}

InitialBoxEstimate InitialBoxEstimator::update(
  const std::vector<float> & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Isometry3d & world_from_optical)
{
  window_.push(depth);
  InitialBoxEstimate estimate;
  estimate.frames_averaged = window_.size();
  estimate.frames_required = window_.capacity();
  if (!window_.full()) {
    return estimate;
  }
  estimate.detection = detectInitialBox(window_.mean(), camera_info, world_from_optical, config_);
  estimate.state = estimate.detection.measured() ? InitialBoxState::kMeasured :
    InitialBoxState::kNotMeasured;
  return estimate;
}

void InitialBoxEstimator::reset()
{
  window_.clear();
}

}  // namespace mujoco_perception
