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

namespace mujoco_bridge
{

// Shared vocabulary between mujoco_bridge (which only ever sees *this instant*'s
// physical state) and task_executor (Stage I), which alone has the phase/timing
// context needed for kTimeout and kPlaceMissed. classifyGrasp() below never returns
// those two -- they exist here so the enum is already stable by the time Stage I's
// FSM starts raising them, not because this function can detect them.
enum class GraspOutcome
{
  kSuccess,
  kNoObject,
  kGraspEmpty,
  kSlip,
  kTimeout,
  kPlaceMissed,
  kUnexpectedContact,
};

// Everything classifyGrasp needs, read from *this* physics step. Deliberately not
// mjModel/mjData: readGripperState()/bodiesInContact() (grasp_state.hpp) do the
// MuJoCo-specific reading, this struct is their output, so the classification logic
// below can be unit tested without loading a model (week2.md Stage H).
struct GraspSignals
{
  double gripper_width_m;       // finger_joint1 + finger_joint2, meters of opening
  double box_height_m;          // box world z (compare against an absolute threshold,
                                // not a table-relative one -- see GraspCriteria)
  double box_to_tcp_horizontal_m;  // |box.xy - hand_tcp.xy|
  bool left_finger_contact;
  bool right_finger_contact;
};

// All four numbers are measured, not guessed -- see week2.md Stage H's three-scenario
// table (empty grasp / normal grasp / induced slip) for how the defaults were derived.
struct GraspCriteria
{
  double box_width_m;
  double width_epsilon_m;
  double lift_height_threshold_m;
  double region_radius_m;
};

// Pre-lift attachment confirmation of ADR 012: width, bilateral contact and TCP proximity.
// No longer drives the bridge (Week 4.1 Stage 13, ADR 019: AttachmentConfirmer below); kept
// for the existing tests and diagnostics. It reads the box's true position.
bool confirmsAttachment(const GraspSignals & signals, const GraspCriteria & criteria);

// What a real Franka Hand reports when it closes (Week 4.1 Stage 13): its width, whether it is
// still moving, and what it was commanded. Deliberately nothing about the box and no contact:
// the Franka Hand has no fingertip sensors; libfranka's is_grasped means "closing stopped
// with the width inside the expected range", which is what AttachmentConfirmer checks.
struct AttachmentSignals
{
  double gripper_width_m;        // finger_joint1 + finger_joint2
  double finger_speed_m_s;       // |finger_joint1 velocity| + |finger_joint2 velocity|
  bool closing_commanded;        // the last gripper command asks for less than the box width
};

struct AttachmentParams
{
  double box_width_m = 0.04;
  double width_epsilon_m = 0.01;   // the width must be within this of the box width
  // The fingers have stopped: closing runs at 450..520 mm/s and is still at 170..210 mm/s
  // inside the width window; blocked by the box it drops below 20 mm/s within 0.03 s.
  double max_finger_speed_m_s = 0.02;
  double hold_s = 0.1;             // for this long in simulated time
};

// Confirms an attachment when the gripper is commanded closed, its width brackets the box and
// the fingers have stopped, continuously for hold_s: closing that stops short of the commanded
// 0 mm means something is in the way. Pure rule: the
// caller feeds one sample per physics observation and resets it at every scene reset.
class AttachmentConfirmer
{
public:
  explicit AttachmentConfirmer(AttachmentParams params = {})
  : params_(params) {}

  // Returns true once the conditions have held for hold_s.
  bool update(const AttachmentSignals & signals, double sim_time_s);
  void reset() {holding_since_valid_ = false;}

  // Which condition is false in this sample, for the log ("" when all hold).
  static const char * missing(const AttachmentSignals & signals, const AttachmentParams & params);

private:
  AttachmentParams params_;
  bool holding_since_valid_ = false;
  double holding_since_s_ = 0.0;
};


// Pure function, single-instant classification. Deliberately does not look at
// whether a ~/gripper_command was ever sent -- outcome is judged from physical state,
// never from "did we issue the right command" (that asymmetry is what "success ==
// trajectory execution finished" gets wrong, per the plan doc 计划书 7.2 第5条).
//
// The width-bracket signal alone cannot tell "closed on nothing" (kGraspEmpty) apart
// from "there was never anything to grasp near here" (kNoObject); the region check is
// what disambiguates them. Contact without the right width, or width without matching
// contact on *both* fingers, is treated as kUnexpectedContact rather than silently
// folded into empty/success -- it usually means misalignment, not a clean miss.
// kSlip is this function's weakest case: a single instant cannot distinguish "grasped
// then slipped" from "never got lifted in the first place" -- Stage I's FSM has to
// bring phase/history to tell those apart; this function only reports "the grip
// looked right a moment ago (width+contact) but the position criteria are not met".

GraspOutcome classifyGrasp(const GraspSignals & signals, const GraspCriteria & criteria);

}  // namespace mujoco_bridge
