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

#include <mujoco_bridge/grasp_criteria.hpp>

#include "task_executor/phase.hpp"
#include "task_executor/waypoint_source.hpp"

namespace task_executor
{

enum class ExitReason
{
  kNone,               // still in progress, no transition this tick
  kReached,            // forward progress: this phase's own criterion was met
  kTimeout,            // phase_timeout_s elapsed without meeting the criterion
  kGraspEmpty,         // kClose/kLift: classifyGrasp said kNoObject/kGraspEmpty
  kSlipped,            // kLift/kPreplace/kPlace: had it, lost it
  kUnexpectedContact,  // kClose: classifyGrasp said kUnexpectedContact
  kPlaceMissed,        // kVerify: box never settled within place_region_radius_m
  kRetryLimitExceeded,  // kRecover: max_retries exhausted, giving up
};

const char * exitReasonName(ExitReason reason);

// The 7 arm joints only (joint1..joint7) -- finger state is carried separately as
// gripper_width_m/grasp_signals below, same split as JointTarget (waypoint_source.hpp)
// and as mujoco_bridge's own ~/joint_command vs ~/gripper_command topics.
struct ArmState
{
  std::array<double, 7> positions{};
  std::array<double, 7> velocities{};
};

// Everything one FSM tick needs. Assembled by task_executor_node.cpp from
// /joint_states + mujoco_bridge's ~/ground_truth/* topics; deliberately not
// rclcpp::Time or any message type, so step() below stays Layer 1 (week2.md Stage
// F's four-layer scheme) -- testable with hand-built structs, no node, no DDS, no
// mjModel.
struct FsmInputs
{
  Phase phase = Phase::kHome;
  ArmState arm;
  double gripper_width_m = 0.0;
  // classifyGrasp()'s own vocabulary (mujoco_bridge/grasp_criteria.hpp) -- reused
  // rather than reimplemented, see that header's docstring on why GraspOutcome was
  // written expecting this node as its second consumer.
  mujoco_bridge::GraspSignals grasp_signals;
  // Absolute box xy, for kVerify's "did it land near the place target" check.
  // classifyGrasp()'s GraspSignals deliberately does not carry this (it only knows
  // box-relative-to-tcp distance, not box-relative-to-a-place-target distance --
  // two different questions), so it travels separately here.
  double box_x_m = 0.0;
  double box_y_m = 0.0;
  double elapsed_in_phase_s = 0.0;
  int retry_count = 0;
};

struct FsmParams
{
  // Tight tolerance for every phase except kGrasp/kClose, where the position
  // servo's steady-state error under gravity is measured (week2.md Stage I,
  // KeyframeWaypointSource's docstring) to be an order of magnitude larger than
  // everywhere else in the sequence -- this is not one global constant because the
  // underlying physical quantity (how much gravity torque a target needs to hold
  // against) genuinely is not.
  double position_epsilon_rad = 0.05;
  double grasp_position_epsilon_rad = 0.3;
  double velocity_epsilon_rad_s = 0.05;
  // Minimum dwell before ANY phase's own "reached" criterion is honored -- not
  // just motionStep()'s velocity check. The general failure mode is a freshly-
  // entered phase reading leftover state from the phase before it: a newly-issued
  // arm command reads back near-zero velocity for the first control cycle simply
  // because the *previous* target's motion had already settled; kClose read back
  // classifyGrasp()==kSlip on its very first tick because the fingers were still
  // resting on the box from kGrasp's approach, not because closing had done
  // anything yet (caught live in week2.md Stage I's validation run). This floor
  // rules both out; it is not a claim about how long real settling takes (that is
  // phase_timeout_s).
  double min_settle_s = 0.5;
  // kLift-only: how long to keep waiting on classifyGrasp() after the ARM has
  // already reached lift height before concluding the grip did not survive.
  // Caught live (week2.md Stage I) -- the mechanism first written down here
  // ("the box lags the arm's settling by ~1s via friction") was disproven by
  // direct instrumentation, then the real root cause was confirmed (week2.md
  // 10.10.2): finger-contact detection flickers (mjContact regenerates every
  // physics step) for a duration that depends on grip firmness -- with a looser
  // grip, that flicker window can last long enough to land on the exact tick
  // armReached() first turns true (reproduced directly: 1/50 trials at the
  // historical looser grip width). This constant's job: don't trust a
  // single-tick disagreement between "arm says done" and "grasp classifier says
  // not yet" -- give it more ticks. Deliberately larger than min_settle_s,
  // which answers a different question (has *this tick's* reading gone stale
  // from the previous phase).
  double lift_settle_grace_s = 2.0;
  // kClose-only: how long to hold the closed gripper command before treating
  // classifyGrasp()==kSlip as "gripped, ready to lift". Caught live (week2.md
  // Stage I): advancing at min_settle_s (0.5s) meant the fingers had only just
  // started squeezing -- the position servo has not finished converging on its
  // steady-state closed width yet, so the grip has not actually stabilized even
  // though width+contact already momentarily bracket the box. This is separate
  // from lift_settle_grace_s (which waits on a stable classifyGrasp() reading
  // after the arm has already reached lift height) -- this one waits on the
  // *gripper* stabilizing before the arm moves at all.
  double close_settle_s = 2.0;
  double phase_timeout_s = 6.0;
  int max_retries = 3;
  mujoco_bridge::GraspCriteria grasp_criteria;
  double place_x_m = 0.0;
  double place_y_m = 0.0;
  double place_region_radius_m = 0.06;
};

struct FsmDecision
{
  Phase next_phase = Phase::kHome;
  ExitReason exit_reason = ExitReason::kNone;
  // True exactly when next_phase re-enters kHome from kRecover: the glue
  // (task_executor_node.cpp) uses this, not a phase comparison, to decide whether
  // to increment retry_count -- kHome is also the very first phase of an episode,
  // and that entry must NOT count against max_retries.
  bool is_retry = false;
};

// One tick's worth of pure phase-transition logic: no publishing, no logging, no
// clock. `target` is the *commanded* configuration for `in.phase` (from a
// WaypointSource) -- passed in rather than looked up here so this function has no
// dependency on WaypointSource's concrete implementation either.
FsmDecision step(const FsmInputs & in, const JointTarget & target, const FsmParams & params);

}  // namespace task_executor
