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

#include "task_executor/reset_gate.hpp"

namespace task_executor
{

uint64_t ResetGate::begin(TimePoint now)
{
  ++request_id_;
  started_ = now;
  generation_ = 0;
  bridge_session_ = 0;
  last_sample_sequence_ = 0;
  failure_code_ = "NONE";
  state_ = State::kRequestPending;
  return request_id_;
}

bool ResetGate::markRequestSent()
{
  if (state_ != State::kRequestPending) {
    return false;
  }
  state_ = State::kAwaitingResponse;
  return true;
}

bool ResetGate::onResetResponse(
  uint64_t request_id, bool success, uint64_t bridge_session, uint64_t generation)
{
  if (request_id != request_id_ || state_ != State::kAwaitingResponse) {
    return false;
  }
  if (!success) {
    failure_code_ = "RESET_FAILED";
    state_ = State::kFailed;
    return true;
  }
  generation_ = generation;
  bridge_session_ = bridge_session;
  state_ = State::kAwaitingObservation;
  return true;
}

bool ResetGate::accept(
  uint64_t bridge_session, uint64_t generation, uint64_t sample_sequence, TimePoint now)
{
  if (bridge_session != bridge_session_) {
    return false;
  }
  if ((state_ == State::kAwaitingObservation || state_ == State::kReady) &&
    generation > generation_)
  {
    failure_code_ = "RESET_SUPERSEDED";
    state_ = State::kFailed;
    return false;
  }
  if ((state_ == State::kAwaitingObservation || state_ == State::kReady) &&
    generation == generation_ && sample_sequence > last_sample_sequence_)
  {
    last_sample_sequence_ = sample_sequence;
    last_sample_at_ = now;
    state_ = State::kReady;
    return true;
  }
  return false;
}

bool ResetGate::checkTimeout(TimePoint now, std::chrono::steady_clock::duration limit)
{
  if (state_ == State::kReady) {
    if (now - last_sample_at_ < limit) {
      return false;
    }
    failure_code_ = "OBSERVATION_STALE";
    state_ = State::kFailed;
    return true;
  }
  if (state_ == State::kFailed || state_ == State::kIdle || now - started_ < limit) {
    return false;
  }
  failure_code_ = state_ == State::kAwaitingObservation ?
    "OBSERVATION_STALE" : "RESET_UNAVAILABLE";
  state_ = State::kFailed;
  return true;
}

}  // namespace task_executor
