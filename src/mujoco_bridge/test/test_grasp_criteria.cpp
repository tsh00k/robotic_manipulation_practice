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

#include "mujoco_bridge/grasp_criteria.hpp"

namespace
{

// Placeholder numbers, not the measured ones -- classifyGrasp's *logic* does not
// depend on the real box_width_m/width_epsilon_m from week2.md Stage H's manual
// measurement, only the node's runtime parameters do. See mujoco_bridge_node.cpp for
// the actual defaults.
constexpr mujoco_bridge::GraspCriteria kCriteria{
  /*box_width_m=*/ 0.04,
  /*width_epsilon_m=*/ 0.01,
  /*lift_height_threshold_m=*/ 0.26,
  /*region_radius_m=*/ 0.05,
};

mujoco_bridge::GraspSignals AllGood()
{
  return {0.04, 0.30, 0.0, true, true};
}

}  // namespace

TEST(ClassifyGrasp, AllFourConditionsMetIsSuccess)
{
  EXPECT_EQ(
    mujoco_bridge::classifyGrasp(
      AllGood(), kCriteria), mujoco_bridge::GraspOutcome::kSuccess);
}

TEST(ClassifyGrasp, ClosedOnNothingNearTheGripperIsGraspEmpty)
{
  mujoco_bridge::GraspSignals s{0.0, 0.24, 0.02, false, false};
  EXPECT_EQ(mujoco_bridge::classifyGrasp(s, kCriteria), mujoco_bridge::GraspOutcome::kGraspEmpty);
}

TEST(ClassifyGrasp, ClosedOnNothingFarFromAnyObjectIsNoObject)
{
  mujoco_bridge::GraspSignals s{0.0, 0.24, 0.5, false, false};
  EXPECT_EQ(mujoco_bridge::classifyGrasp(s, kCriteria), mujoco_bridge::GraspOutcome::kNoObject);
}

TEST(ClassifyGrasp, OnlyOneFingerTouchingIsUnexpectedContact)
{
  mujoco_bridge::GraspSignals s = AllGood();
  s.right_finger_contact = false;
  EXPECT_EQ(
    mujoco_bridge::classifyGrasp(
      s,
      kCriteria),
    mujoco_bridge::GraspOutcome::kUnexpectedContact);
}

TEST(ClassifyGrasp, WidthMatchesButNoContactIsUnexpectedContact)
{
  mujoco_bridge::GraspSignals s = AllGood();
  s.left_finger_contact = false;
  s.right_finger_contact = false;
  EXPECT_EQ(
    mujoco_bridge::classifyGrasp(
      s,
      kCriteria),
    mujoco_bridge::GraspOutcome::kUnexpectedContact);
}

TEST(ClassifyGrasp, ContactButWidthReadsFullyClosedIsUnexpectedContact)
{
  mujoco_bridge::GraspSignals s = AllGood();
  s.gripper_width_m = 0.0;
  EXPECT_EQ(
    mujoco_bridge::classifyGrasp(
      s,
      kCriteria),
    mujoco_bridge::GraspOutcome::kUnexpectedContact);
}

TEST(ClassifyGrasp, GrippedButNeverLiftedIsSlip)
{
  mujoco_bridge::GraspSignals s = AllGood();
  s.box_height_m = 0.24;  // still resting on the table
  EXPECT_EQ(mujoco_bridge::classifyGrasp(s, kCriteria), mujoco_bridge::GraspOutcome::kSlip);
}

TEST(ClassifyGrasp, GrippedAndLiftedButFarFromTcpIsSlip)
{
  mujoco_bridge::GraspSignals s = AllGood();
  s.box_to_tcp_horizontal_m = 0.2;
  EXPECT_EQ(mujoco_bridge::classifyGrasp(s, kCriteria), mujoco_bridge::GraspOutcome::kSlip);
}

TEST(ClassifyGrasp, WidthJustInsideEpsilonIsBracketed)
{
  mujoco_bridge::GraspSignals s = AllGood();
  s.gripper_width_m = kCriteria.box_width_m + kCriteria.width_epsilon_m * 0.99;
  EXPECT_EQ(mujoco_bridge::classifyGrasp(s, kCriteria), mujoco_bridge::GraspOutcome::kSuccess);
}

TEST(ClassifyGrasp, WidthJustOutsideEpsilonIsNotBracketed)
{
  mujoco_bridge::GraspSignals s = AllGood();
  s.gripper_width_m = kCriteria.box_width_m + kCriteria.width_epsilon_m * 1.01;
  EXPECT_EQ(
    mujoco_bridge::classifyGrasp(
      s,
      kCriteria),
    mujoco_bridge::GraspOutcome::kUnexpectedContact);
}
