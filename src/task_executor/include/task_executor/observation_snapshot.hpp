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

#include <array>
#include <optional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "task_executor/observation_frame.hpp"

namespace task_executor
{

inline constexpr std::array<const char *, 7> kArmJointNames = {
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

using ObservationSnapshot = ObservationFrame;

std::optional<ObservationSnapshot> makeObservationSnapshot(
  const sensor_msgs::msg::JointState & joints,
  const geometry_msgs::msg::PoseStamped & object_pose,
  bool left_contact, bool right_contact,
  const geometry_msgs::msg::TransformStamped & hand_tcp_tf);

}  // namespace task_executor
