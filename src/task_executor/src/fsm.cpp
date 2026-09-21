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

#include "task_executor/fsm.hpp"

#include <algorithm>
#include <cmath>

namespace task_executor
{

using mujoco_bridge::classifyGrasp;
using mujoco_bridge::GraspOutcome;

const char * exitReasonName(ExitReason reason)
{
  switch (reason) {
    case ExitReason::kNone: return "NONE";
    case ExitReason::kReached: return "REACHED";
    case ExitReason::kTimeout: return "TIMEOUT";
    case ExitReason::kGraspEmpty: return "GRASP_EMPTY";
    case ExitReason::kSlipped: return "SLIPPED";
    case ExitReason::kUnexpectedContact: return "UNEXPECTED_CONTACT";
    case ExitReason::kPlaceMissed: return "PLACE_MISSED";
    case ExitReason::kRetryLimitExceeded: return "RETRY_LIMIT_EXCEEDED";
  }
  return "UNKNOWN";
}

namespace
{

bool armReached(
  const ArmState & arm, const std::array<double, 7> & target, double position_epsilon_rad,
  double velocity_epsilon_rad_s)
{
  for (size_t i = 0; i < 7; ++i) {
    if (std::abs(arm.positions[i] - target[i]) >= position_epsilon_rad) {
      return false;
    }
    if (std::abs(arm.velocities[i]) >= velocity_epsilon_rad_s) {
      return false;
    }
  }
  return true;
}

bool bothFingersHolding(const mujoco_bridge::GraspSignals & s)
{
  return s.left_finger_contact && s.right_finger_contact;
}

// Guards every "reached" branch below, not just motionStep()'s. Caught live
// (week2.md Stage I validation run) on kClose: the very first tick after entering
// kClose could already read classifyGrasp() as kSlip and advance to kLift in
// 0.05s -- not because the gripper had closed, but because the fingers were
// already resting on the box from kGrasp's *open-handed* approach, so the
// leftover contact from the previous phase satisfied kClose's own criterion
// before the close command had done anything. Same root cause as
// FsmParams::min_settle_s (a freshly-entered phase's first reading can be stale
// leftover state from the phase before it), just discovered on a non-motion
// branch, so the fix generalizes here instead of staying motionStep-only.
bool pastMinSettle(const FsmInputs & in, const FsmParams & params)
{
  return in.elapsed_in_phase_s >= params.min_settle_s;
}

// Motion-only phases: advance on armReached(), RECOVER on phase_timeout_s, else
// hold. Shared by every phase where the FSM's only job is "wait for the arm to get
// there" -- kClose/kLift/kVerify each need extra logic layered on top (see step()
// below) and do not call this helper directly.
FsmDecision motionStep(
  const FsmInputs & in, const JointTarget & target, const FsmParams & params,
  double position_epsilon_rad)
{
  if (armReached(
      in.arm, target.arm_positions, position_epsilon_rad, params.velocity_epsilon_rad_s) &&
    pastMinSettle(in, params))
  {
    return {nextPhase(in.phase), ExitReason::kReached, false};
  }
  if (in.elapsed_in_phase_s > params.phase_timeout_s) {
    return {Phase::kRecover, ExitReason::kTimeout, false};
  }
  return {in.phase, ExitReason::kNone, false};
}

}  // namespace

FsmDecision step(const FsmInputs & in, const JointTarget & target, const FsmParams & params)
{
  switch (in.phase) {
    case Phase::kHome:
    case Phase::kPregrasp:
    case Phase::kRetract:
      return motionStep(in, target, params, params.position_epsilon_rad);

    case Phase::kGrasp:
      // Deliberately not checking grasp_signals here: kGrasp is a pure approach
      // waypoint (fingers are still open), so there is nothing for classifyGrasp()
      // to classify yet other than incidental contact from the approach itself.
      return motionStep(in, target, params, params.grasp_position_epsilon_rad);

    case Phase::kPreplace:
    case Phase::kPlace: {
        // Holding the object across the whole transport leg (kPreplace, kPlace), so
        // a drop mid-transit must be caught even though each phase's own criterion
        // is otherwise a plain motion check.
        //
        // Deliberately box_height_m, NOT bothFingersHolding() -- caught live
        // (week2.md Stage I): mujoco_bridge's own grasp outcome log shows
        // left/right finger contact flickering SUCCESS<->UNEXPECTED_CONTACT every
        // single tick even during a hold that is, by every other measure (box
        // height staying near its lifted value), completely stable. That flicker
        // is mjContact regenerating from scratch every mj_forward (grasp_state.hpp)
        // catching one finger geom out of contact for a single physics step's
        // worth of jitter, not a real drop. Height changes continuously and only
        // actually falls when the object is really let go, which is exactly the
        // physical event this check needs to catch.
        if (in.grasp_signals.box_height_m < params.grasp_criteria.lift_height_threshold_m) {
          return {Phase::kRecover, ExitReason::kSlipped, false};
        }
        return motionStep(in, target, params, params.grasp_position_epsilon_rad);
      }

    case Phase::kClose: {
        const GraspOutcome outcome = classifyGrasp(in.grasp_signals, params.grasp_criteria);
        // kSuccess or kSlip both mean "width+contact already look like a grip" --
        // classifyGrasp() cannot tell "grasped" from "grasped, not yet lifted" apart
        // on its own (grasp_criteria.hpp's own docstring), and kClose is exactly the
        // phase supplying that missing context: we have not attempted to lift yet,
        // so kSlip here reads as success, not failure.
        if ((outcome == GraspOutcome::kSuccess || outcome == GraspOutcome::kSlip) &&
          in.elapsed_in_phase_s >= params.close_settle_s)
        {
          return {nextPhase(in.phase), ExitReason::kReached, false};
        }
        if (in.elapsed_in_phase_s > params.phase_timeout_s) {
          const ExitReason reason = outcome == GraspOutcome::kUnexpectedContact ?
            ExitReason::kUnexpectedContact :
            ExitReason::kGraspEmpty;
          return {Phase::kRecover, reason, false};
        }
        return {in.phase, ExitReason::kNone, false};
      }

    case Phase::kLift: {
        const GraspOutcome outcome = classifyGrasp(in.grasp_signals, params.grasp_criteria);
        if (outcome == GraspOutcome::kSuccess && pastMinSettle(in, params)) {
          return {nextPhase(in.phase), ExitReason::kReached, false};
        }
        const bool arm_at_lift_height = armReached(
          in.arm, target.arm_positions, params.position_epsilon_rad, params.velocity_epsilon_rad_s);
        if (arm_at_lift_height && in.elapsed_in_phase_s > params.lift_settle_grace_s) {
          // The arm got where kLift's target says it should be, but classifyGrasp()
          // still is not reporting kSuccess -- the grip did not survive the lift.
          // This is the one place in the whole sequence that can genuinely tell
          // "grasped then slipped" apart from "never got lifted" (grasp_criteria.hpp
          // flags this as its own blind spot): the arm-reached fact is exactly the
          // extra information classifyGrasp() does not have.
          return {Phase::kRecover, ExitReason::kSlipped, false};
        }
        if (in.elapsed_in_phase_s > params.phase_timeout_s) {
          return {Phase::kRecover, ExitReason::kTimeout, false};
        }
        return {in.phase, ExitReason::kNone, false};
      }

    case Phase::kOpen: {
        // The one phase where checking the *gripper's* own position (rather than the
        // arm's) is the right criterion -- releasing has no gravity-sag problem the
        // way holding a reach-forward arm pose does, so a plain width threshold is
        // enough, no phase-specific epsilon needed.
        constexpr double kOpenWidthM = 0.08;
        constexpr double kOpenEpsilonM = 0.02;
        if (in.gripper_width_m > kOpenWidthM - kOpenEpsilonM && pastMinSettle(in, params)) {
          return {nextPhase(in.phase), ExitReason::kReached, false};
        }
        if (in.elapsed_in_phase_s > params.phase_timeout_s) {
          return {Phase::kRecover, ExitReason::kTimeout, false};
        }
        return {in.phase, ExitReason::kNone, false};
      }

    case Phase::kVerify: {
        const double dx = in.box_x_m - params.place_x_m;
        const double dy = in.box_y_m - params.place_y_m;
        const bool released = !bothFingersHolding(in.grasp_signals);
        if (std::hypot(dx, dy) < params.place_region_radius_m && released &&
          pastMinSettle(in, params))
        {
          return {Phase::kDone, ExitReason::kReached, false};
        }
        if (in.elapsed_in_phase_s > params.phase_timeout_s) {
          return {Phase::kRecover, ExitReason::kPlaceMissed, false};
        }
        return {in.phase, ExitReason::kNone, false};
      }

    case Phase::kRecover:
      if (in.retry_count < params.max_retries) {
        return {Phase::kHome, ExitReason::kNone, true};
      }
      return {Phase::kFailed, ExitReason::kRetryLimitExceeded, false};

    case Phase::kDone:
    case Phase::kFailed:
      // Terminal: hold here forever. task_executor_node.cpp stops issuing new
      // commands once it sees either, so returning in.phase is inert, not a bug.
      return {in.phase, ExitReason::kNone, false};
  }
  return {in.phase, ExitReason::kNone, false};
}

}  // namespace task_executor
