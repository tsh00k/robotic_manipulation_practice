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
