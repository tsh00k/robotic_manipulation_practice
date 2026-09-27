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

#include "task_executor/observation_snapshot.hpp"

#include <algorithm>
#include <cmath>

namespace task_executor
{

std::optional<ObservationSnapshot> makeObservationSnapshot(
  const sensor_msgs::msg::JointState & joints,
  const geometry_msgs::msg::PoseStamped & object_pose,
  bool left_contact, bool right_contact,
  const geometry_msgs::msg::TransformStamped & hand_tcp_tf)
{
  ObservationSnapshot snapshot;
  for (size_t i = 0; i < kArmJointNames.size(); ++i) {
    const auto it = std::find(
      joints.name.begin(), joints.name.end(), std::string(kArmJointNames[i]));
    if (it == joints.name.end()) {
      return std::nullopt;
    }
    const size_t idx = std::distance(joints.name.begin(), it);
    if (idx >= joints.position.size() || idx >= joints.velocity.size()) {
      return std::nullopt;
    }
    snapshot.arm.positions[i] = joints.position[idx];
    snapshot.arm.velocities[i] = joints.velocity[idx];
  }

  int fingers_found = 0;
  for (size_t i = 0; i < joints.name.size(); ++i) {
    if (joints.name[i] == "finger_joint1" || joints.name[i] == "finger_joint2") {
      if (i >= joints.position.size()) {
        return std::nullopt;
      }
      snapshot.gripper_width_m += joints.position[i];
      ++fingers_found;
    }
  }
  if (fingers_found != 2) {
    return std::nullopt;
  }

  snapshot.object_pose = {
    object_pose.pose.position.x, object_pose.pose.position.y,
    object_pose.pose.position.z, object_pose.pose.orientation.w,
    object_pose.pose.orientation.x, object_pose.pose.orientation.y,
    object_pose.pose.orientation.z};
  snapshot.object_frame_id = object_pose.header.frame_id;
  snapshot.grasp_signals.gripper_width_m = snapshot.gripper_width_m;
  snapshot.grasp_signals.box_height_m = snapshot.object_pose.z;
  snapshot.grasp_signals.box_to_tcp_horizontal_m = std::hypot(
    snapshot.object_pose.x - hand_tcp_tf.transform.translation.x,
    snapshot.object_pose.y - hand_tcp_tf.transform.translation.y);
  snapshot.grasp_signals.left_finger_contact = left_contact;
  snapshot.grasp_signals.right_finger_contact = right_contact;
  snapshot.world_to_hand_tcp.translation() = Eigen::Vector3d(
    hand_tcp_tf.transform.translation.x, hand_tcp_tf.transform.translation.y,
    hand_tcp_tf.transform.translation.z);
  snapshot.world_to_hand_tcp.linear() = Eigen::Quaterniond(
    hand_tcp_tf.transform.rotation.w, hand_tcp_tf.transform.rotation.x,
    hand_tcp_tf.transform.rotation.y, hand_tcp_tf.transform.rotation.z).toRotationMatrix();
  return snapshot;
}

}  // namespace task_executor
