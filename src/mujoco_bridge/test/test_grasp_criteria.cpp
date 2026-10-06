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

#include <cmath>
#include <limits>

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

TEST(AttachmentConfirmation, GripOnTableCanConfirmBeforeLift)
{
  auto s = AllGood();
  s.box_height_m = 0.24;
  EXPECT_EQ(mujoco_bridge::classifyGrasp(s, kCriteria), mujoco_bridge::GraspOutcome::kSlip);
  EXPECT_TRUE(mujoco_bridge::confirmsAttachment(s, kCriteria));
}

TEST(AttachmentConfirmation, RequiresWidthBothContactsAndProximity)
{
  auto s = AllGood();
  s.left_finger_contact = false;
  EXPECT_FALSE(mujoco_bridge::confirmsAttachment(s, kCriteria));
  s = AllGood();
  s.right_finger_contact = false;
  EXPECT_FALSE(mujoco_bridge::confirmsAttachment(s, kCriteria));
  s = AllGood();
  s.gripper_width_m = 0.0;
  EXPECT_FALSE(mujoco_bridge::confirmsAttachment(s, kCriteria));
  s = AllGood();
  s.box_to_tcp_horizontal_m = 0.2;
  EXPECT_FALSE(mujoco_bridge::confirmsAttachment(s, kCriteria));
  s = AllGood();
  s.gripper_width_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mujoco_bridge::confirmsAttachment(s, kCriteria));
  s = AllGood();
  s.box_to_tcp_horizontal_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mujoco_bridge::confirmsAttachment(s, kCriteria));
}

namespace
{

mujoco_bridge::AttachmentSignals Holding()
{
  return {0.0385, 0.005, true};
}

// Feed the same signals at the bridge's 100 Hz from t0 for `duration`; return the last answer.
bool feed(
  mujoco_bridge::AttachmentConfirmer & confirmer, const mujoco_bridge::AttachmentSignals & s,
  double t0, double duration)
{
  bool confirmed = false;
  const int steps = static_cast<int>(std::lround(duration / 0.01));
  for (int i = 0; i <= steps; ++i) {
    confirmed = confirmer.update(s, t0 + 0.01 * i);
  }
  return confirmed;
}

TEST(AttachmentConfirmer, ConfirmsAfterTheHoldTimeWithoutAnyBoxQuantity)
{
  mujoco_bridge::AttachmentConfirmer confirmer;
  EXPECT_FALSE(feed(confirmer, Holding(), 0.0, 0.09));
  EXPECT_TRUE(confirmer.update(Holding(), 0.10));
}

TEST(AttachmentConfirmer, EachConditionIsNeeded)
{
  const mujoco_bridge::AttachmentParams params;
  auto empty = Holding();
  empty.gripper_width_m = 0.0;        // closed on nothing
  auto still_closing = Holding();  // passing through the window on the way to 0
  still_closing.gripper_width_m = 0.045;
  still_closing.finger_speed_m_s = 0.2;
  auto no_command = Holding();
  no_command.closing_commanded = false;
  auto too_wide = Holding();
  too_wide.gripper_width_m = 0.052;
  for (const auto & s : {empty, still_closing, no_command, too_wide}) {
    mujoco_bridge::AttachmentConfirmer confirmer;
    EXPECT_FALSE(feed(confirmer, s, 0.0, 1.0)) << mujoco_bridge::AttachmentConfirmer::missing(
      s,
      params);
  }
  EXPECT_STREQ(mujoco_bridge::AttachmentConfirmer::missing(empty, params), "WIDTH_OUTSIDE_BOX");
  EXPECT_STREQ(
    mujoco_bridge::AttachmentConfirmer::missing(still_closing, params), "FINGERS_STILL_MOVING");
  EXPECT_STREQ(mujoco_bridge::AttachmentConfirmer::missing(no_command, params), "NO_CLOSE_COMMAND");
  EXPECT_STREQ(mujoco_bridge::AttachmentConfirmer::missing(Holding(), params), "");
}

TEST(AttachmentConfirmer, ABrokenSampleRestartsTheHoldAndResetForgets)
{
  mujoco_bridge::AttachmentConfirmer confirmer;
  feed(confirmer, Holding(), 0.0, 0.08);
  auto flicker = Holding();
  flicker.finger_speed_m_s = 0.03;
  EXPECT_FALSE(confirmer.update(flicker, 0.09));
  EXPECT_FALSE(feed(confirmer, Holding(), 0.10, 0.09));
  EXPECT_TRUE(confirmer.update(Holding(), 0.20));
  confirmer.reset();
  EXPECT_FALSE(confirmer.update(Holding(), 0.30));
}

}  // namespace
