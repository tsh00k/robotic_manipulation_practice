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

#include "task_executor/waypoint_source.hpp"

namespace task_executor
{

// Fixed joint-space lookup table, hand-measured against the running sim (see
// week2.md Stage I for the full search log) rather than computed from object_pose --
// this is deliberately the "no IK yet" implementation the plan doc calls for.
// object_pose is accepted (WaypointSource's interface) and ignored.
//
// Every number below was found by publishing a candidate to
// /mujoco_bridge/joint_command against a *reset* sim, waiting long enough for the
// position servo to actually settle (>= 5 sim seconds -- see the docstring on why
// that number matters), and reading back /tf world->hand_tcp and
// ~/ground_truth/object_pose. Two things fell out of that search that are worth
// knowing before touching these numbers:
//
// 1. Steady-state PD error scales with how much gravity torque the target needs to
//    hold against, not a fixed few millimeters. At HOME (page docs/architecture.md
//    section 3, Stage D) it is ~7mm / a few degrees. At kGrasp/kClose's target
//    below (joint2=0.4, joint4=-2.0), joint2 alone settles ~0.12-0.17 rad short of
//    commanded -- commanding it *further* past that point does nothing at all
//    (verified: commanding 0.9 through 1.4 all settled to the same actual angle),
//    because the actuator's forcerange is saturated holding the arm's own weight at
//    that reach. This is why fsm.cpp's position_epsilon_rad is per-phase, not one
//    global constant: kGrasp/kClose need a much looser tolerance than every other
//    phase, and that looseness is a measured fact about this configuration, not a
//    knob turned until tests passed.
// 2. A direct one-shot jump from HOME straight to a "reach down and forward"
//    config sweeps the forearm sideways through the box's column and knocks it
//    away before the fingers ever close -- confirmed by watching
//    ~/ground_truth/object_pose shift several cm *before* any gripper command was
//    sent. kPregrasp exists as a real safety waypoint, not a cosmetic one: hovering
//    there first (same x/y neighborhood as kGrasp, well above the box) keeps the
//    kPregrasp->kGrasp transition close to a straight vertical descent.
// 3. A "narrower than the box" close target (0.03, i.e. leave the servo pushing
//    against the box with 1cm of commanded overlap) grips firmly enough to
//    survive a pure vertical kLift, but NOT the joint1 rotation kPreplace needs --
//    the box visibly rotates out through the single remaining point of contact
//    mid-swing (confirmed: classifyGrasp() flips to a persistent one-sided
//    kUnexpectedContact with no further log line as the box quietly settles back
//    at table height, since that category itself never changes even as the box
//    falls through it -- this is why fsm.cpp's kPreplace/kPlace drop check reads
//    box_height_m directly rather than trusting the finger-contact booleans).
//    Fully closing (0.0, i.e. as much commanded overlap as the actuator's
//    ctrlrange allows) squeezes harder and empirically survives the same swing --
//    verified end-to-end through kPlace and a full release. kClosedWidthM below
//    is 0.0, not 0.03, for exactly this reason.
//
// In the legacy joint-space mode, PLACE (joint1=0.62) lands ~5-7cm short of the
// visual place_marker (0.5, 0.3). These are empirically chosen servo commands,
// not world-frame task geometry; Stage N showed that applying FK to them already
// puts the nominal TCP target near (0.434, 0.310), before execution error enters.
// Stage O's separate Cartesian source now targets the marker directly.
class KeyframeWaypointSource : public WaypointSource
{
public:
  JointTarget jointTargetFor(Phase phase, const ObjectPose & /*object_pose*/) const override
  {
    switch (phase) {
      case Phase::kHome:
        return {{0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, -0.7853}, kOpenWidthM};
      case Phase::kPregrasp:
        return {{0.0, 0.2, 0.0, -1.6, 0.0, 1.5708, -0.7853}, kOpenWidthM};
      case Phase::kGrasp:
        return {{0.0, 0.4, 0.0, -2.0, 0.0, 1.5708, -0.7853}, kOpenWidthM};
      case Phase::kClose:
        // Same arm target as kGrasp -- only the gripper moves in this phase.
        return {{0.0, 0.4, 0.0, -2.0, 0.0, 1.5708, -0.7853}, kClosedWidthM};
      case Phase::kLift:
        // Same arm target as kPregrasp: retracing the same joint-space path back up
        // keeps the ascent close to vertical, same reasoning as kPregrasp itself.
        return {{0.0, 0.2, 0.0, -1.6, 0.0, 1.5708, -0.7853}, kClosedWidthM};
      case Phase::kPreplace:
        return {{0.62, 0.2, 0.0, -1.6, 0.0, 1.5708, -0.7853}, kClosedWidthM};
      case Phase::kPlace:
        return {{0.62, 0.27, 0.0, -1.75, 0.0, 1.5708, -0.7853}, kClosedWidthM};
      case Phase::kOpen:
        // Same arm target as kPlace -- only the gripper moves in this phase.
        return {{0.62, 0.27, 0.0, -1.75, 0.0, 1.5708, -0.7853}, kOpenWidthM};
      case Phase::kRetract:
        // Same arm target as kPreplace: withdraw back up the same path just placed
        // through.
        return {{0.62, 0.2, 0.0, -1.6, 0.0, 1.5708, -0.7853}, kOpenWidthM};
      case Phase::kVerify:
      case Phase::kDone:
      case Phase::kRecover:
      case Phase::kFailed:
        // No arm motion commanded in these phases -- fall back to kRetract's target
        // so a caller that (incorrectly) commands one of these still holds a sane,
        // already-reached configuration rather than snapping the ctrl target to all
        // zeros.
        return {{0.62, 0.2, 0.0, -1.6, 0.0, 1.5708, -0.7853}, kOpenWidthM};
    }
    return {{0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, -0.7853}, kOpenWidthM};
  }

private:
  // Total finger-to-finger opening, same convention as GripperCommand.position
  // (Stage H) -- 0.08 is fully open (2 x each finger's own 0.04m travel). 0.0
  // commands full closure; the servo cannot actually reach it with the box in the
  // way (see grasp_criteria.hpp's box_width_m/width_epsilon_m for how this still
  // reads back as ~0.04m once the box is actually in the way), so this is "squeeze
  // as hard as the position servo will push", not a literal target width -- see
  // this class's docstring point 3 for why that firmer squeeze (over a milder
  // narrower-than-box target) is what it takes to survive kPreplace's swing.
  static constexpr double kOpenWidthM = 0.08;
  static constexpr double kClosedWidthM = 0.0;
};

}  // namespace task_executor
