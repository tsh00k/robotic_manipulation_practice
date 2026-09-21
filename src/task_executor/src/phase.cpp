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

#include "task_executor/phase.hpp"

#include <stdexcept>

namespace task_executor
{

const char * phaseName(Phase phase)
{
  switch (phase) {
    case Phase::kHome: return "HOME";
    case Phase::kPregrasp: return "PREGRASP";
    case Phase::kGrasp: return "GRASP";
    case Phase::kClose: return "CLOSE";
    case Phase::kLift: return "LIFT";
    case Phase::kPreplace: return "PREPLACE";
    case Phase::kPlace: return "PLACE";
    case Phase::kOpen: return "OPEN";
    case Phase::kRetract: return "RETRACT";
    case Phase::kVerify: return "VERIFY";
    case Phase::kDone: return "DONE";
    case Phase::kRecover: return "RECOVER";
    case Phase::kFailed: return "FAILED";
  }
  return "UNKNOWN";
}

Phase nextPhase(Phase phase)
{
  switch (phase) {
    case Phase::kHome: return Phase::kPregrasp;
    case Phase::kPregrasp: return Phase::kGrasp;
    case Phase::kGrasp: return Phase::kClose;
    case Phase::kClose: return Phase::kLift;
    case Phase::kLift: return Phase::kPreplace;
    case Phase::kPreplace: return Phase::kPlace;
    case Phase::kPlace: return Phase::kOpen;
    case Phase::kOpen: return Phase::kRetract;
    case Phase::kRetract: return Phase::kVerify;
    case Phase::kVerify: return Phase::kDone;
    default:
      throw std::logic_error(
              "nextPhase() called on a phase with no forward successor (DONE/RECOVER/FAILED "
              "are handled by fsm.cpp's step(), not this table)");
  }
}

}  // namespace task_executor
