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
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "task_executor/episode_telemetry_data.hpp"
#include "task_executor/observation_frame.hpp"
#include "task_executor/diff_ik_waypoint_source.hpp"

namespace task_executor
{

struct ResetReceipt
{
  bool success = false;
  uint64_t bridge_session = 0;
  uint64_t generation = 0;
};

// Episode lifecycle; FSM-driven Phase transitions occur only while kReady.
enum class EpisodeState
{
  kIdle,
  kResetPending,
  kAwaitingResetResponse,
  kAwaitingObservation,
  kReady,
  kFinished,
  kFailed,
};

struct ResetRequest
{
  uint64_t request_id = 0;
};

struct TargetCommand
{
  Phase phase = Phase::kHome;
  JointTarget target;
};

struct EpisodeFinished
{
  bool success = false;
  std::string failure_code;
  int retry_count = 0;
  EpisodeTelemetry telemetry;
};

struct DiagnosticEvent
{
  enum class Kind
  {
    kObservationAccepted,
    kObservationRejected,
    kEpisodeFinished,
    kIkFailed,
  };

  Kind kind = Kind::kObservationRejected;
  std::string code;
  uint64_t sample_sequence = 0;
};

struct PhaseTransition
{
  Phase from = Phase::kHome;
  Phase to = Phase::kHome;
  ExitReason reason = ExitReason::kNone;
  JointTarget target;
  PhaseTelemetry telemetry;
  double tcp_rotation_error_rad = std::numeric_limits<double>::quiet_NaN();
};

// Values returned by the controller. A reset request can be repeated until
// the adapter acknowledges that it sent the service request.
struct EpisodeActions
{
  std::optional<ResetRequest> reset_request;
  std::optional<TargetCommand> target;
  std::optional<EpisodeFinished> finished;
  std::optional<PhaseTransition> transition;
  std::optional<WaypointDiagnostics> target_diagnostics;
  std::vector<DiagnosticEvent> diagnostics;

  bool empty() const
  {
    return !reset_request && !target && !finished && !transition &&
           !target_diagnostics && diagnostics.empty();
  }
};

class EpisodeController
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  // watchdog_timeout bounds every wait. awaiting_observation_timeout, when given, replaces it
  // for the wait between the reset response and the first admitted observation only: a vision
  // executor first waits for the initial pose to be latched (Week 4.1 Stage 7), which takes
  // longer than the 5 s that is right for a stalled stream.
  explicit EpisodeController(
    const WaypointSource & waypoint_source, FsmParams fsm_params = {},
    DiffIkWaypointSource * diff_ik_source = nullptr,
    std::chrono::steady_clock::duration watchdog_timeout = std::chrono::seconds(5),
    std::optional<std::chrono::steady_clock::duration> awaiting_observation_timeout = {});

  // Starts a new episode, including when an earlier episode is still active.
  // The same request id is offered on each pending tick until acknowledged.
  EpisodeActions startEpisode(TimePoint now, double sim_time_s = 0.0);

  // Retry keeps episode diagnostics but starts a new reset generation.
  EpisodeActions beginRetry(TimePoint now);

  // Call only after the adapter has sent the matching service request.
  bool onResetRequestSent(uint64_t request_id);

  // Delivers the service result belonging to request_id. Old or duplicate
  // receipts are ignored so they cannot affect a later episode.
  EpisodeActions onResetResponse(
    uint64_t request_id, const ResetReceipt & receipt, double sim_time_s = 0.0);

  // Stores only a fresh observation. It does not advance the FSM or publish a
  // target; those side effects are reserved for tick().
  EpisodeActions onObservation(
    const ObservationEnvelope & observation, TimePoint now, double sim_time_s = 0.0);

  // Consumes at most one newly admitted observation. The target in an action
  // batch always belongs to the phase before its transition.
  EpisodeActions tick(double sim_time_s, TimePoint wall_now);

  EpisodeActions finishEpisode(
    bool success, std::string failure_code = "NONE", bool append_failed_phase = false);

  // Accessors: any current state; no transition; return the stored state/data.
  EpisodeState state() const {return state_;}
  Phase phase() const {return phase_;}
  uint64_t requestId() const {return request_id_;}
  uint64_t bridgeSession() const {return bridge_session_;}
  uint64_t generation() const {return generation_;}
  int retryCount() const {return retry_count_;}
  const std::optional<ObservationEnvelope> & latestObservation() const {return latest_observation_;}
  const EpisodeTelemetry & telemetry() const {return telemetry_;}

private:
  EpisodeActions finishFromFailure(const char * failure_code);
  EpisodeActions rejectedObservation(uint64_t sample_sequence, const char * code) const;
  EpisodeActions beginReset(TimePoint now);
  EpisodeActions tickLifecycle(TimePoint wall_now);

  const WaypointSource & waypoint_source_;
  FsmParams fsm_params_;
  DiffIkWaypointSource * diff_ik_source_;
  const std::chrono::steady_clock::duration watchdog_timeout_;
  const std::chrono::steady_clock::duration awaiting_observation_timeout_;
  EpisodeState state_ = EpisodeState::kIdle;
  Phase phase_ = Phase::kHome;
  uint64_t request_id_ = 0;
  uint64_t bridge_session_ = 0;
  uint64_t generation_ = 0;
  uint64_t last_sample_sequence_ = 0;
  uint64_t consumed_sample_sequence_ = 0;
  double phase_start_sim_time_s_ = 0.0;
  double current_sim_time_s_ = 0.0;
  std::optional<Phase> logged_target_phase_;
  ExitReason last_failure_reason_ = ExitReason::kNone;
  TimePoint reset_started_at_{};
  TimePoint last_sample_at_{};
  int retry_count_ = 0;
  std::optional<ObservationEnvelope> latest_observation_;
  EpisodeTelemetry telemetry_;
};

}  // namespace task_executor
