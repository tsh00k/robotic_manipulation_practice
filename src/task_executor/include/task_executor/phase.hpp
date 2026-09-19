#pragma once

namespace task_executor
{

// week2.md Stage I's fixed sequence: HOME -> PREGRASP -> GRASP -> CLOSE -> LIFT ->
// PREPLACE -> PLACE -> OPEN -> RETRACT -> VERIFY -> DONE, with RECOVER as the only
// off-ramp (from any phase, on timeout/failure) and FAILED as the terminal giving-up
// state after too many RECOVER round-trips. There is deliberately no "back to HOME
// to idle" phase after DONE/FAILED -- an episode boundary is the process boundary
// this week (Stage J's episode runner, not this node, owns "start a new attempt").
enum class Phase
{
  kHome,
  kPregrasp,
  kGrasp,
  kClose,
  kLift,
  kPreplace,
  kPlace,
  kOpen,
  kRetract,
  kVerify,
  kDone,
  kRecover,
  kFailed,
};

const char * phaseName(Phase phase);

// The phase that follows `phase` on a plain "reached, no problems" advance. Only
// defined for the 10 forward phases (kDone/kRecover/kFailed have no "next" in this
// sense -- fsm.cpp handles their transitions separately, not via this table).
Phase nextPhase(Phase phase);

}  // namespace task_executor
