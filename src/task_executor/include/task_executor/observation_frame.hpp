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

#include <cstdint>
#include <string>

#include "task_executor/fsm.hpp"
#include "task_executor/waypoint_source.hpp"

namespace task_executor
{

// A complete observation in the episode domain. It deliberately contains no
// ROS message or clock type; adapters can construct it from any observation
// source (ROS, a replay, or a unit test).
struct ObservationFrame
{
  ArmState arm;
  double gripper_width_m = 0.0;
  ObjectPose object_pose;
  std::string object_frame_id;
  mujoco_bridge::GraspSignals grasp_signals;
  Eigen::Isometry3d world_to_hand_tcp = Eigen::Isometry3d::Identity();
};

struct ObservationEnvelope
{
  ObservationFrame frame;
  uint64_t bridge_session = 0;
  uint64_t generation = 0;
  uint64_t sample_sequence = 0;
  double sim_time_s = 0.0;
};

}  // namespace task_executor
