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

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "task_executor/episode_telemetry_data.hpp"
#include "task_executor/observation_frame.hpp"
#include "task_executor/diff_ik_waypoint_source.hpp"
#include "task_executor/joint_trajectory_planner.hpp"

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
    kTrajectoryFailed,
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
  // The arm's timed trajectory for the phase, once on phase entry (Week 5 Stage 3); while
  // trajectories are used, target only carries the gripper width and the FSM's target.
  std::optional<JointTrajectoryPlan> trajectory;
  std::optional<EpisodeFinished> finished;
  std::optional<PhaseTransition> transition;
  std::optional<WaypointDiagnostics> target_diagnostics;
  std::vector<DiagnosticEvent> diagnostics;

  bool empty() const
  {
    return !reset_request && !target && !trajectory && !finished && !transition &&
           !target_diagnostics && diagnostics.empty();
  }
};

// Timed arm trajectories (Week 5 Stage 3, ADR 020).
struct TrajectoryOptions
{
  JointLimits limits = kPandaReferenceLimits;
  // Where the bridge's servo targets are right after a reset: the reset keyframe, which is the
  // HOME configuration (Week 5 Stage 2). The first trajectory after a reset starts here, not at
  // the measured joints, which sag a few milliradians under gravity: starting there would step
  // the servo targets by that much.
  std::array<double, 7> reset_arm_positions = kFrankaReadyPose;
};

class EpisodeController
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  // watchdog_timeout bounds every wait. awaiting_observation_timeout, when given, replaces it
  // for the wait between the reset response and the first admitted observation only: a vision
  // executor first waits for the initial pose to be latched (Week 4.1 Stage 7), which takes
  // longer than the 5 s that is right for a stalled stream.
  // trajectory, when given, makes every phase's arm motion a timed trajectory (Week 5 Stage 3,
  // ADR 020): on phase entry the controller plans from where the previous phase's reference
  // ended (after a reset, TrajectoryOptions::reset_arm_positions) to the phase's target and
  // returns it once; the phase cannot end before it has run out. The path is the joint-space
  // line, except GRASP, LIFT, PLACE and RETRACT, which follow the TCP straight line when the
  // waypoints come from IK (diff_ik_source). Without it, the target is a step, as before.
  explicit EpisodeController(
    const WaypointSource & waypoint_source, FsmParams fsm_params = {},
    DiffIkWaypointSource * diff_ik_source = nullptr,
    std::chrono::steady_clock::duration watchdog_timeout = std::chrono::seconds(5),
    std::optional<std::chrono::steady_clock::duration> awaiting_observation_timeout = {},
    std::optional<TrajectoryOptions> trajectory = {});

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

  // The place target of the current episode (IK targets) and the centre of VERIFY's place
  // region. With a bin both are the bin (Week 4.1 Stage 8). Without one they are the
  // configured target and verification centre, which only differ in the deliberate
  // verify.allow_target_mismatch experiment. Set before the first admitted observation;
  // kept across retries unless set again.
  // into_bin: VERIFY asks boxInBin() against `place` (its x, y, support_z, yaw) instead of the
  // radius around (verify_x_m, verify_y_m).
  void setPlacement(
    const PlaceTarget & place, double verify_x_m, double verify_y_m, bool into_bin = false);
  const PlaceTarget & placeTarget() const {return place_;}

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
  PlaceTarget place_;
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
  std::optional<TrajectoryOptions> trajectory_;
  // Where the last planned reference ends, the phase it was planned for, and when it started;
  // all cleared by a reset.
  std::optional<std::array<double, 7>> reference_end_;
  std::optional<Phase> planned_phase_;
  double trajectory_start_s_ = 0.0;
  double trajectory_duration_s_ = 0.0;
  ExitReason last_failure_reason_ = ExitReason::kNone;
  TimePoint reset_started_at_{};
  TimePoint last_sample_at_{};
  int retry_count_ = 0;
  std::optional<ObservationEnvelope> latest_observation_;
  EpisodeTelemetry telemetry_;
};

}  // namespace task_executor
