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

#include <gtest/gtest.h>

#include <algorithm>

#include "task_executor/observation_snapshot.hpp"

namespace task_executor
{
namespace
{

sensor_msgs::msg::JointState jointState()
{
  sensor_msgs::msg::JointState joints;
  for (size_t i = 0; i < kArmJointNames.size(); ++i) {
    joints.name.push_back(kArmJointNames[i]);
    joints.position.push_back(static_cast<double>(i));
    joints.velocity.push_back(static_cast<double>(i) / 10.0);
  }
  joints.name.insert(joints.name.end(), {"finger_joint1", "finger_joint2"});
  joints.position.insert(joints.position.end(), {0.02, 0.03});
  joints.velocity.insert(joints.velocity.end(), {0.0, 0.0});
  return joints;
}

TEST(ObservationSnapshot, JointNamesDetermineOrder)
{
  auto joints = jointState();
  std::reverse(joints.name.begin(), joints.name.end());
  std::reverse(joints.position.begin(), joints.position.end());
  std::reverse(joints.velocity.begin(), joints.velocity.end());
  geometry_msgs::msg::PoseStamped object;
  object.header.frame_id = "world";
  object.pose.position.x = 0.5;
  object.pose.position.y = 0.1;
  object.pose.position.z = 0.25;
  geometry_msgs::msg::TransformStamped hand;
  hand.transform.translation.x = 0.5;
  hand.transform.rotation.w = 1.0;

  const auto snapshot = makeObservationSnapshot(joints, object, true, false, hand);
  ASSERT_TRUE(snapshot);
  for (size_t i = 0; i < kArmJointNames.size(); ++i) {
    EXPECT_DOUBLE_EQ(snapshot->arm.positions[i], static_cast<double>(i));
    EXPECT_DOUBLE_EQ(snapshot->arm.velocities[i], static_cast<double>(i) / 10.0);
  }
  EXPECT_DOUBLE_EQ(snapshot->gripper_width_m, 0.05);
  EXPECT_DOUBLE_EQ(snapshot->grasp_signals.box_to_tcp_horizontal_m, 0.1);
  EXPECT_TRUE(snapshot->grasp_signals.left_finger_contact);
  EXPECT_FALSE(snapshot->grasp_signals.right_finger_contact);
  EXPECT_EQ(snapshot->object_frame_id, "world");
}

TEST(ObservationSnapshot, IncompleteJointMessageIsNotReady)
{
  const geometry_msgs::msg::PoseStamped object;
  geometry_msgs::msg::TransformStamped hand;
  hand.transform.rotation.w = 1.0;
  auto joints = jointState();
  joints.name.pop_back();
  EXPECT_FALSE(makeObservationSnapshot(joints, object, false, false, hand));

  joints = jointState();
  joints.velocity.resize(2);
  EXPECT_FALSE(makeObservationSnapshot(joints, object, false, false, hand));
}

}  // namespace
}  // namespace task_executor
