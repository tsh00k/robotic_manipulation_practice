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

#include "task_executor/fsm.hpp"

namespace task_executor
{
namespace
{

FsmParams defaultParams()
{
  FsmParams p;
  p.position_epsilon_rad = 0.05;
  p.grasp_position_epsilon_rad = 0.3;
  p.velocity_epsilon_rad_s = 0.05;
  p.min_settle_s = 0.5;
  p.lift_settle_grace_s = 2.0;
  p.close_settle_s = 2.0;
  p.phase_timeout_s = 6.0;
  p.max_retries = 3;
  p.grasp_criteria.box_width_m = 0.04;
  p.grasp_criteria.width_epsilon_m = 0.01;
  p.grasp_criteria.lift_height_threshold_m = 0.26;
  p.grasp_criteria.region_radius_m = 0.05;
  p.place_x_m = 0.43;
  p.place_y_m = 0.31;
  p.place_region_radius_m = 0.08;
  return p;
}

JointTarget targetAt(std::array<double, 7> arm, double gripper_width_m = 0.08)
{
  JointTarget t;
  t.arm_positions = arm;
  t.gripper_width_m = gripper_width_m;
  return t;
}

ArmState settledAt(const std::array<double, 7> & positions)
{
  ArmState a;
  a.positions = positions;
  a.velocities = {0, 0, 0, 0, 0, 0, 0};
  return a;
}

constexpr std::array<double, 7> kHomeArm = kFrankaReadyPose;

TEST(Fsm, MotionPhaseAdvancesOnceSettledPastMinSettle)
{
  FsmInputs in;
  in.phase = Phase::kHome;
  in.arm = settledAt(kHomeArm);
  in.elapsed_in_phase_s = 1.0;  // > min_settle_s
  const FsmDecision d = step(in, targetAt(kHomeArm), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kPregrasp);
  EXPECT_EQ(d.exit_reason, ExitReason::kReached);
  EXPECT_FALSE(d.is_retry);
}

TEST(Fsm, MotionPhaseDoesNotAdvanceBeforeMinSettle)
{
  // Regression guard for the exact bug flagged in fsm.hpp's FsmParams::min_settle_s
  // docstring: a freshly-entered phase reads back the *previous* phase's settled
  // (zero) velocity for at least one tick, and must not be mistaken for "reached"
  // just because velocity happens to already be zero.
  FsmInputs in;
  in.phase = Phase::kHome;
  in.arm = settledAt(kHomeArm);
  in.elapsed_in_phase_s = 0.0;
  const FsmDecision d = step(in, targetAt(kHomeArm), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kHome);
  EXPECT_EQ(d.exit_reason, ExitReason::kNone);
}

TEST(Fsm, MotionPhaseRecoversOnTimeout)
{
  FsmInputs in;
  in.phase = Phase::kPregrasp;
  in.arm = settledAt(kHomeArm);  // never actually reaches PREGRASP's target
  in.elapsed_in_phase_s = 100.0;
  const FsmDecision d = step(in, targetAt({0, 0.2, 0, -1.6, 0, 1.5708, -0.7853}), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kRecover);
  EXPECT_EQ(d.exit_reason, ExitReason::kTimeout);
}

TEST(Fsm, CloseAdvancesOnBridgeAttachmentBeforeLift)
{
  FsmInputs in;
  in.phase = Phase::kClose;
  in.attachment_state = 1;
  in.grasp_signals.gripper_width_m = 0.04;
  in.grasp_signals.box_height_m = 0.24;  // still on the table, below lift threshold
  in.grasp_signals.box_to_tcp_horizontal_m = 0.003;
  in.grasp_signals.left_finger_contact = true;
  in.grasp_signals.right_finger_contact = true;
  in.elapsed_in_phase_s = 0.5;
  in.attached_for_s = 0.25;  // attached for longer than close_after_attach_s
  const FsmDecision d = step(in, targetAt({0, 0.4, 0, -2.0, 0, 1.5708, -0.7853}), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kLift);
  EXPECT_EQ(d.exit_reason, ExitReason::kReached);
}

TEST(Fsm, CloseDoesNotAdvanceUntilAttachmentHasHeldForCloseAfterAttachS)
{
  // Regression guard for the exact bug caught live in week2.md Stage I: the very first tick
  // after entering kClose could already read kSlip and advance to kLift before the gripper had
  // finished squeezing. Since Week 5 Stage 5 the gate is "attached for close_after_attach_s"
  // (the bridge's own confirmation already requires the fingers to have stopped for 0.1 s).
  FsmInputs in;
  in.phase = Phase::kClose;
  in.attachment_state = 1;
  in.grasp_signals.gripper_width_m = 0.04;
  in.grasp_signals.box_height_m = 0.24;
  in.grasp_signals.box_to_tcp_horizontal_m = 0.003;
  in.grasp_signals.left_finger_contact = true;
  in.grasp_signals.right_finger_contact = true;
  in.elapsed_in_phase_s = 0.5;
  in.attached_for_s = 0.05;  // attached, but not yet for close_after_attach_s
  const FsmDecision d = step(in, targetAt({0, 0.4, 0, -2.0, 0, 1.5708, -0.7853}), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kClose);
  EXPECT_EQ(d.exit_reason, ExitReason::kNone);
}

// Week 5 Stage 5: OPEN ends as soon as the fingers are actually open, without waiting out
// min_settle_s (no stale reading is possible: CLOSE ends at about 40 mm).
TEST(Fsm, OpenAdvancesOnWidthWithoutWaitingForMinSettle)
{
  FsmInputs in;
  in.phase = Phase::kOpen;
  in.gripper_width_m = 0.07;
  in.elapsed_in_phase_s = 0.1;  // well under min_settle_s
  EXPECT_EQ(step(in, targetAt(kHomeArm), defaultParams()).next_phase, Phase::kRetract);
  in.gripper_width_m = 0.05;  // not open yet
  EXPECT_EQ(step(in, targetAt(kHomeArm), defaultParams()).next_phase, Phase::kOpen);
}

TEST(Fsm, CloseRecoversOnGraspEmptyPastTimeout)
{
  FsmInputs in;
  in.phase = Phase::kClose;
  in.grasp_signals.gripper_width_m = 0.0;
  in.grasp_signals.box_height_m = 0.24;
  in.grasp_signals.box_to_tcp_horizontal_m = 0.003;  // near tcp, but never touched
  in.grasp_signals.left_finger_contact = false;
  in.grasp_signals.right_finger_contact = false;
  in.elapsed_in_phase_s = 100.0;
  const FsmDecision d = step(in, targetAt({0, 0.4, 0, -2.0, 0, 1.5708, -0.7853}), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kRecover);
  EXPECT_EQ(d.exit_reason, ExitReason::kGraspEmpty);
}

TEST(Fsm, CloseDoesNotGiveUpBeforeTimeoutWhileStillClosing)
{
  FsmInputs in;
  in.phase = Phase::kClose;
  in.grasp_signals.gripper_width_m = 0.06;  // partway closed, not yet bracketing box
  in.grasp_signals.box_height_m = 0.24;
  in.grasp_signals.box_to_tcp_horizontal_m = 0.003;
  in.elapsed_in_phase_s = 0.2;
  const FsmDecision d = step(in, targetAt({0, 0.4, 0, -2.0, 0, 1.5708, -0.7853}), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kClose);
  EXPECT_EQ(d.exit_reason, ExitReason::kNone);
}

TEST(Fsm, LiftAdvancesOnSuccess)
{
  FsmInputs in;
  in.phase = Phase::kLift;
  in.attachment_state = 1;
  in.grasp_signals.gripper_width_m = 0.04;
  in.grasp_signals.box_height_m = 0.37;  // above lift_height_threshold_m
  in.grasp_signals.box_to_tcp_horizontal_m = 0.003;
  in.grasp_signals.left_finger_contact = true;
  in.grasp_signals.right_finger_contact = true;
  in.elapsed_in_phase_s = 1.0;
  in.arm = settledAt({0, 0.2, 0, -1.6, 0, 1.5708, -0.7853});
  const FsmDecision d = step(in, targetAt({0, 0.2, 0, -1.6, 0, 1.5708, -0.7853}), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kPreplace);
  EXPECT_EQ(d.exit_reason, ExitReason::kReached);
}

TEST(Fsm, AttachedLiftWaitsForArmToReachTarget)
{
  FsmInputs in;
  in.phase = Phase::kLift;
  in.attachment_state = 1;
  in.arm = settledAt(kHomeArm);
  in.elapsed_in_phase_s = 2.5;
  const auto d = step(in, targetAt({0, 0.2, 0, -1.6, 0, 1.5708, -0.7853}), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kLift);
  EXPECT_EQ(d.exit_reason, ExitReason::kNone);
  in.elapsed_in_phase_s = 7.0;
  const auto timeout = step(in, targetAt({0, 0.2, 0, -1.6, 0, 1.5708, -0.7853}), defaultParams());
  EXPECT_EQ(timeout.exit_reason, ExitReason::kTimeout);
}

TEST(Fsm, LiftRecoversAsSlippedWhenArmReachedButGripDidNotSucceed)
{
  const std::array<double, 7> lift_target = {0, 0.2, 0, -1.6, 0, 1.5708, -0.7853};
  FsmInputs in;
  in.phase = Phase::kLift;
  in.arm = settledAt(lift_target);  // arm got there
  in.grasp_signals.gripper_width_m = 0.0;  // but the box is gone
  in.grasp_signals.box_height_m = 0.24;
  in.grasp_signals.box_to_tcp_horizontal_m = 0.2;
  in.grasp_signals.left_finger_contact = false;
  in.grasp_signals.right_finger_contact = false;
  in.elapsed_in_phase_s = 2.5;  // past lift_settle_grace_s
  const FsmDecision d = step(in, targetAt(lift_target), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kRecover);
  EXPECT_EQ(d.exit_reason, ExitReason::kSlipped);
}

TEST(Fsm, LiftGivesTheBoxTimeToCatchUpBeforeCallingItSlipped)
{
  // Regression guard for the exact bug caught live in week2.md Stage I. The
  // mechanism originally recorded here ("box lags the arm via friction, needs
  // real time to physically catch up") was disproven by direct instrumentation,
  // then the real root cause was confirmed (week2.md 10.10.2): finger-contact
  // detection flickers (mjContact regenerates every physics step, see 10.10.4)
  // for a duration that depends on grip firmness, and can land on the exact
  // tick armReached() first turns true -- reproduced directly (1/50 trials at
  // the historical looser grip width used when this bug was first found). This
  // test's synthetic input (arm fully settled, box_height_m still at table
  // height) is a worst-case version of that same disagreement: concluding
  // kSlipped the instant armReached() turns true, without any grace period, is
  // wrong regardless of how transient the underlying signal glitch is.
  const std::array<double, 7> lift_target = {0, 0.2, 0, -1.6, 0, 1.5708, -0.7853};
  FsmInputs in;
  in.phase = Phase::kLift;
  in.arm = settledAt(lift_target);  // arm just got there
  in.grasp_signals.gripper_width_m = 0.04;
  in.grasp_signals.box_height_m = 0.24;  // box has not risen yet
  in.grasp_signals.box_to_tcp_horizontal_m = 0.003;
  in.grasp_signals.left_finger_contact = true;
  in.grasp_signals.right_finger_contact = true;
  in.elapsed_in_phase_s = 0.55;  // just past min_settle_s, well under the grace period
  const FsmDecision d = step(in, targetAt(lift_target), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kLift);
  EXPECT_EQ(d.exit_reason, ExitReason::kNone);
}

TEST(Fsm, LiftKeepsWaitingWhileArmStillMovingAndNotYetTimedOut)
{
  FsmInputs in;
  in.phase = Phase::kLift;
  in.arm = settledAt(kHomeArm);  // nowhere near lift_target yet
  in.grasp_signals.gripper_width_m = 0.04;
  in.grasp_signals.box_height_m = 0.24;
  in.grasp_signals.box_to_tcp_horizontal_m = 0.003;
  in.grasp_signals.left_finger_contact = true;
  in.grasp_signals.right_finger_contact = true;
  in.elapsed_in_phase_s = 0.2;
  const FsmDecision d = step(in, targetAt({0, 0.2, 0, -1.6, 0, 1.5708, -0.7853}), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kLift);
  EXPECT_EQ(d.exit_reason, ExitReason::kNone);
}

TEST(Fsm, PreplaceRecoversWhenBridgeReportsNoAttachment)
{
  FsmInputs in;
  in.phase = Phase::kPreplace;
  in.grasp_signals.box_height_m = 0.24;  // stale geometry cannot override bridge state
  in.elapsed_in_phase_s = 0.1;  // well before any timeout
  const FsmDecision d =
    step(in, targetAt({0.62, 0.2, 0, -1.6, 0, 1.5708, -0.7853}, 0.03), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kRecover);
  EXPECT_EQ(d.exit_reason, ExitReason::kSlipped);
}

TEST(Fsm, PreplaceToleratesMomentaryFingerContactFlickerWhileStillHeldAloft)
{
  // Regression guard for the exact bug caught live in week2.md Stage I:
  // mujoco_bridge's own grasp-outcome log shows left/right contact flickering
  // false<->true every physics tick even during a completely stable hold. Gating
  // this phase's drop-detection on the raw booleans (rather than box height,
  // which changes continuously) made every real success look like an instant
  // drop the moment kPreplace began.
  FsmInputs in;
  in.phase = Phase::kPreplace;
  in.attachment_state = 1;
  in.grasp_signals.box_height_m = 0.37;  // well above lift threshold, still aloft
  in.grasp_signals.left_finger_contact = false;  // this tick's contact flicker
  in.grasp_signals.right_finger_contact = true;
  in.arm = settledAt(kHomeArm);  // not yet at PREPLACE's target
  in.elapsed_in_phase_s = 0.1;
  const FsmDecision d =
    step(in, targetAt({0.62, 0.2, 0, -1.6, 0, 1.5708, -0.7853}, 0.03), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kPreplace);
  EXPECT_EQ(d.exit_reason, ExitReason::kNone);
}

TEST(Fsm, OpenAdvancesWhenGripperNearlyFullyOpen)
{
  FsmInputs in;
  in.phase = Phase::kOpen;
  in.gripper_width_m = 0.075;
  in.elapsed_in_phase_s = 1.0;
  const FsmDecision d =
    step(in, targetAt({0.62, 0.27, 0, -1.75, 0, 1.5708, -0.7853}, 0.08), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kRetract);
  EXPECT_EQ(d.exit_reason, ExitReason::kReached);
}

TEST(Fsm, VerifyDoneWhenBoxNearPlaceAndReleased)
{
  FsmInputs in;
  in.phase = Phase::kVerify;
  in.attachment_state = 2;
  in.box_x_m = 0.43;
  in.box_y_m = 0.30;  // within place_region_radius_m of (0.43, 0.31)
  in.grasp_signals.left_finger_contact = false;
  in.grasp_signals.right_finger_contact = false;
  in.arm = settledAt(kHomeArm);
  in.elapsed_in_phase_s = 1.0;
  const FsmDecision d = step(in, targetAt(kHomeArm), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kDone);
  EXPECT_EQ(d.exit_reason, ExitReason::kReached);
}

// VERIFY's target is HOME (Week 5 Stage 2): a box already in place is not DONE while the arm
// is still on its way back, so DONE always leaves the arm where the next reset puts it.
TEST(Fsm, VerifyWaitsForTheArmToReturnHome)
{
  FsmInputs in;
  in.phase = Phase::kVerify;
  in.attachment_state = 2;
  in.box_x_m = 0.43;
  in.box_y_m = 0.30;
  in.elapsed_in_phase_s = 1.0;
  std::array<double, 7> on_the_way = kHomeArm;
  on_the_way[1] += 0.2;
  in.arm = settledAt(on_the_way);
  EXPECT_EQ(step(in, targetAt(kHomeArm), defaultParams()).next_phase, Phase::kVerify);
  in.arm = settledAt(kHomeArm);
  in.arm.velocities[3] = 0.3;  // there, but still moving
  EXPECT_EQ(step(in, targetAt(kHomeArm), defaultParams()).next_phase, Phase::kVerify);
  in.arm = settledAt(kHomeArm);
  EXPECT_EQ(step(in, targetAt(kHomeArm), defaultParams()).next_phase, Phase::kDone);
}

TEST(Fsm, WithABinVerifyIsDoneOnlyWhenTheBoxIsInsideTheBin)
{
  FsmParams params = defaultParams();
  params.place_into_bin = true;
  params.bin = {0.56, -0.12, 0.227, 0.3};
  FsmInputs in;
  in.phase = Phase::kVerify;
  in.attachment_state = 2;
  in.elapsed_in_phase_s = 1.0;
  in.box_x_m = 0.56;
  in.box_y_m = -0.12;
  in.box_z_m = 0.247;
  in.box_yaw_rad = 0.7;
  in.arm = settledAt(kHomeArm);
  EXPECT_EQ(step(in, targetAt(kHomeArm), params).next_phase, Phase::kDone);

  // 45 mm off centre: inside the old 0.08 m radius, but in the bin frame a corner reaches
  // 69 mm against the 65 mm inner face.
  in.box_y_m = -0.12 + 0.045;
  EXPECT_EQ(step(in, targetAt(kHomeArm), params).next_phase, Phase::kVerify);
  // Centred but resting on a wall rim, 12 mm too high.
  in.box_y_m = -0.12;
  in.box_z_m = 0.259;
  EXPECT_EQ(step(in, targetAt(kHomeArm), params).next_phase, Phase::kVerify);
  in.elapsed_in_phase_s = 100.0;
  EXPECT_EQ(step(in, targetAt(kHomeArm), params).exit_reason, ExitReason::kPlaceMissed);
}

TEST(Fsm, VerifyRecoversAsPlaceMissedOnTimeout)
{
  FsmInputs in;
  in.phase = Phase::kVerify;
  in.box_x_m = 0.9;  // nowhere near the place target
  in.box_y_m = 0.9;
  in.elapsed_in_phase_s = 100.0;
  const FsmDecision d = step(in, targetAt(kHomeArm), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kRecover);
  EXPECT_EQ(d.exit_reason, ExitReason::kPlaceMissed);
}

TEST(Fsm, RecoverRetriesToHomeUnderRetryLimit)
{
  FsmInputs in;
  in.phase = Phase::kRecover;
  in.retry_count = 1;
  const FsmDecision d = step(in, targetAt(kHomeArm), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kHome);
  EXPECT_TRUE(d.is_retry);
}

TEST(Fsm, RecoverGivesUpPastRetryLimit)
{
  FsmInputs in;
  in.phase = Phase::kRecover;
  in.retry_count = 3;  // == max_retries
  const FsmDecision d = step(in, targetAt(kHomeArm), defaultParams());
  EXPECT_EQ(d.next_phase, Phase::kFailed);
  EXPECT_EQ(d.exit_reason, ExitReason::kRetryLimitExceeded);
  EXPECT_FALSE(d.is_retry);
}

TEST(Fsm, DoneAndFailedAreTerminal)
{
  FsmInputs in;
  in.phase = Phase::kDone;
  EXPECT_EQ(step(in, targetAt(kHomeArm), defaultParams()).next_phase, Phase::kDone);
  in.phase = Phase::kFailed;
  EXPECT_EQ(step(in, targetAt(kHomeArm), defaultParams()).next_phase, Phase::kFailed);
}

}  // namespace
}  // namespace task_executor
