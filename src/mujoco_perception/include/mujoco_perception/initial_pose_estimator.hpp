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

#include "mujoco_perception/depth_window.hpp"
#include "mujoco_perception/initial_bin_detector.hpp"
#include "mujoco_perception/initial_box_detector.hpp"

namespace mujoco_perception
{

// Where one object's estimate stands. WARMING_UP is not a failure: it is the time it takes N
// frames to arrive, and a consumer has to be able to tell it from "the detector looked and
// found nothing".
enum class InitialPoseState
{
  kWarmingUp,    // fewer than `frames` usable frames since the window was last emptied
  kNotMeasured,  // the window is full and the detector refused; see the detection's rejection
  kMeasured,     // the detection holds the pose
};

const char * initialPoseStateName(InitialPoseState state);

struct InitialPoseEstimate
{
  std::size_t frames_averaged = 0;   // frames in the window when this estimate was made
  std::size_t frames_required = 0;
  // Valid only when the window was full.
  InitialBoxResult box;
  InitialBinResult bin;

  bool windowFull() const {return frames_averaged == frames_required;}
  InitialPoseState boxState() const
  {
    return !windowFull() ? InitialPoseState::kWarmingUp :
           box.measured() ? InitialPoseState::kMeasured : InitialPoseState::kNotMeasured;
  }
  InitialPoseState binState() const
  {
    return !windowFull() ? InitialPoseState::kWarmingUp :
           bin.measured() ? InitialPoseState::kMeasured : InitialPoseState::kNotMeasured;
  }
};

// Averages the last `frames` depth images of a STATIC scene and detects the box and the bin in
// the mean (Week 4.1 Stages 5 and 6, docs/adr/018). Both use the same window, since they look at
// the same depth. It owns no ROS state: the node decides which frames are usable and when the
// scene stopped being static, and calls reset() then.
class InitialPoseEstimator
{
public:
  InitialPoseEstimator(
    std::size_t frames, const InitialBoxConfig & box_config, const InitialBinConfig & bin_config);

  // Add one usable depth frame (metres, row-major, NaN or <= 0 where there is no measurement,
  // for instance under the masked robot) and estimate from the window as it now stands.
  InitialPoseEstimate update(
    const std::vector<float> & depth,
    const sensor_msgs::msg::CameraInfo & camera_info,
    const Eigen::Isometry3d & world_from_optical);

  // Forget every frame: the scene has changed (new episode, box moved by the robot).
  void reset();

  std::size_t frames() const {return window_.capacity();}
  std::size_t framesInWindow() const {return window_.size();}

private:
  DepthWindow window_;
  InitialBoxConfig box_config_;
  InitialBinConfig bin_config_;
};

}  // namespace mujoco_perception
