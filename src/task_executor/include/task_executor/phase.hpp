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

namespace task_executor
{

// week2.md Stage I's fixed sequence: HOME -> PREGRASP -> GRASP -> CLOSE -> LIFT ->
// PREPLACE -> PLACE -> OPEN -> RETRACT -> VERIFY -> DONE, with RECOVER as the only
// off-ramp (from any phase, on timeout/failure) and FAILED as the terminal giving-up
// state after too many RECOVER round-trips. There is deliberately no explicit
// "idle" phase in this enum: task_executor_node.cpp's onTimer() treats kDone/kFailed
// as terminal (stops publishing/deciding) until it receives ~/start_episode, which
// starts a new controller episode at kHome without needing a phase of its own here.
// (Stage I originally said the *process* boundary was the episode boundary -- that
// assumption was replaced in Stage J once the episode runner needed to drive many
// consecutive episodes against the same long-lived node instead of restarting it
// each time, which would have reintroduced the DDS-discovery-race class of bug
// week2.md 10.10.2's first (discarded) reproduction attempt ran into.)
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
