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

#include <chrono>
#include <cstdint>

namespace task_executor
{

class ResetGate
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;
  enum class State {kIdle, kRequestPending, kAwaitingResponse, kAwaitingObservation, kReady,
    kFailed};

  uint64_t begin(TimePoint now);
  bool markRequestSent();
  bool onResetResponse(
    uint64_t request_id, bool success, uint64_t bridge_session, uint64_t generation);
  bool accept(
    uint64_t bridge_session, uint64_t generation, uint64_t sample_sequence, TimePoint now);
  bool checkTimeout(TimePoint now, std::chrono::steady_clock::duration limit);
  State state() const {return state_;}
  uint64_t generation() const {return generation_;}
  uint64_t bridgeSession() const {return bridge_session_;}
  const char * failureCode() const {return failure_code_;}

private:
  State state_ = State::kIdle;
  TimePoint started_{};
  uint64_t request_id_ = 0;
  uint64_t generation_ = 0;
  uint64_t bridge_session_ = 0;
  uint64_t last_sample_sequence_ = 0;
  TimePoint last_sample_at_{};
  const char * failure_code_ = "NONE";
};

}  // namespace task_executor
