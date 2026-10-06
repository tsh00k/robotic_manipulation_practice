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

#include "task_executor/episode_controller.hpp"

#include <exception>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace task_executor
{

// Current: no episode exists.
// Transition: initialize to kIdle.
// Return: a controller; no actions.
EpisodeController::EpisodeController(
  const WaypointSource & waypoint_source, FsmParams fsm_params,
  DiffIkWaypointSource * diff_ik_source, std::chrono::steady_clock::duration watchdog_timeout,
  std::optional<std::chrono::steady_clock::duration> awaiting_observation_timeout)
: waypoint_source_(waypoint_source), fsm_params_(std::move(fsm_params)),
  diff_ik_source_(diff_ik_source), watchdog_timeout_(watchdog_timeout),
  awaiting_observation_timeout_(awaiting_observation_timeout.value_or(watchdog_timeout))
{
  place_.x = fsm_params_.place_x_m;
  place_.y = fsm_params_.place_y_m;
}

void EpisodeController::setPlacement(
  const PlaceTarget & place, double verify_x_m, double verify_y_m)
{
  place_ = place;
  fsm_params_.place_x_m = verify_x_m;
  fsm_params_.place_y_m = verify_y_m;
}

// Current: any state, including an active or terminal episode.
// Transition: any -> kResetPending; clear the previous episode.
// Return: ResetRequest with a new request id.
EpisodeActions EpisodeController::startEpisode(TimePoint now, double sim_time_s)
{
  current_sim_time_s_ = sim_time_s;
  phase_start_sim_time_s_ = sim_time_s;
  retry_count_ = 0;
  phase_ = Phase::kHome;
  telemetry_.clear();
  last_failure_reason_ = ExitReason::kNone;
  logged_target_phase_.reset();
  if (diff_ik_source_) {diff_ik_source_->beginEpisode();}
  return beginReset(now);
}

// Current: kReady (other states are unchanged).
// Transition: kReady -> kResetPending; keep telemetry and increment retry count.
// Return: new ResetRequest, or empty actions outside kReady.
EpisodeActions EpisodeController::beginRetry(TimePoint now)
{
  if (state_ != EpisodeState::kReady) {
    return {};
  }
  ++retry_count_;
  phase_ = Phase::kHome;
  logged_target_phase_.reset();
  if (diff_ik_source_) {diff_ik_source_->beginEpisode();}
  return beginReset(now);
}

// Current: start or retry has chosen to reset.
// Transition: -> kResetPending; invalidate the previous reset and observation.
// Return: ResetRequest with the new request id.
EpisodeActions EpisodeController::beginReset(TimePoint now)
{
  ++request_id_;
  state_ = EpisodeState::kResetPending;
  bridge_session_ = 0;
  generation_ = 0;
  last_sample_sequence_ = 0;
  consumed_sample_sequence_ = 0;
  reset_started_at_ = now;
  latest_observation_.reset();

  EpisodeActions actions;
  actions.reset_request = ResetRequest{request_id_};
  return actions;
}

// Current: kResetPending with the matching request id.
// Transition: kResetPending -> kAwaitingResetResponse; otherwise unchanged.
// Return: true only when the adapter's send confirmation is accepted.
bool EpisodeController::onResetRequestSent(uint64_t request_id)
{
  if (state_ != EpisodeState::kResetPending || request_id != request_id_) {
    return false;
  }
  state_ = EpisodeState::kAwaitingResetResponse;
  return true;
}

// Current: kAwaitingResetResponse with the matching request id.
// Transition: success -> kAwaitingObservation; failure -> kFailed.
// Return: empty on success/old reply, or one EpisodeFinished on failure.
EpisodeActions EpisodeController::onResetResponse(
  uint64_t request_id, const ResetReceipt & receipt, double sim_time_s)
{
  if (state_ != EpisodeState::kAwaitingResetResponse || request_id != request_id_) {
    return {};
  }

  current_sim_time_s_ = sim_time_s;

  if (!receipt.success) {
    return finishFromFailure("RESET_FAILED");
  }

  bridge_session_ = receipt.bridge_session;
  generation_ = receipt.generation;
  phase_start_sim_time_s_ = sim_time_s;
  state_ = EpisodeState::kAwaitingObservation;
  latest_observation_.reset();
  return {};
}

// Current: kAwaitingObservation or kReady; other states cannot admit samples.
// Transition: valid sample -> kReady; expiry/supersession -> kFailed.
// Return: accepted/rejected diagnostic, one EpisodeFinished, or empty if terminal.
EpisodeActions EpisodeController::onObservation(
  const ObservationEnvelope & observation, TimePoint now, double sim_time_s)
{
  current_sim_time_s_ = sim_time_s;
  // Idle and terminal episodes cannot be revived by observations.
  if (state_ == EpisodeState::kIdle || state_ == EpisodeState::kFinished ||
    state_ == EpisodeState::kFailed)
  {
    return {};
  }
  if (state_ != EpisodeState::kAwaitingObservation && state_ != EpisodeState::kReady) {
    return rejectedObservation(observation.sample_sequence, "EPISODE_NOT_READY");
  }
  // Check expiry before a late sample can refresh the watchdog.
  if ((state_ == EpisodeState::kReady && now - last_sample_at_ >= watchdog_timeout_) ||
    (state_ == EpisodeState::kAwaitingObservation &&
    now - reset_started_at_ >= awaiting_observation_timeout_))
  {
    return finishFromFailure("OBSERVATION_STALE");
  }

  if (observation.bridge_session != bridge_session_) {
    return rejectedObservation(observation.sample_sequence, "BRIDGE_SESSION_MISMATCH");
  }
  if (observation.generation > generation_) {
    return finishFromFailure("RESET_SUPERSEDED");
  }
  if (observation.generation != generation_ ||
    observation.sample_sequence <= last_sample_sequence_)
  {
    return rejectedObservation(observation.sample_sequence, "OBSERVATION_STALE");
  }

  last_sample_sequence_ = observation.sample_sequence;
  last_sample_at_ = now;
  latest_observation_ = observation;
  state_ = EpisodeState::kReady;
  EpisodeActions actions;
  actions.diagnostics.push_back(
    DiagnosticEvent{DiagnosticEvent::Kind::kObservationAccepted, "OBSERVATION_ACCEPTED",
      observation.sample_sequence});
  return actions;
}

// Current: any lifecycle state.
// Transition: expired active state -> kFailed; otherwise unchanged.
// Return: pending ResetRequest, one EpisodeFinished, or empty actions.
EpisodeActions EpisodeController::tickLifecycle(TimePoint wall_now)
{
  if (state_ == EpisodeState::kIdle || state_ == EpisodeState::kFinished ||
    state_ == EpisodeState::kFailed)
  {
    return {};
  }

  // Ready watches sample freshness; earlier states time out from reset start.
  const bool awaiting_observation = state_ == EpisodeState::kAwaitingObservation;
  const auto wait_limit = awaiting_observation ? awaiting_observation_timeout_ : watchdog_timeout_;
  if (state_ == EpisodeState::kReady) {
    if (wall_now - last_sample_at_ >= watchdog_timeout_) {
      return finishFromFailure("OBSERVATION_STALE");
    }
  } else if (wall_now - reset_started_at_ >= wait_limit) {
    return finishFromFailure(awaiting_observation ? "OBSERVATION_STALE" : "RESET_UNAVAILABLE");
  }

  // Retry the same send intent until the adapter confirms submission.
  if (state_ == EpisodeState::kResetPending) {
    EpisodeActions actions;
    actions.reset_request = ResetRequest{request_id_};
    return actions;
  }

  return {};
}

// Current: any state; IK/FSM runs only in kReady with a new sample.
// Transition: FSM stays ready, retries via kResetPending, or finishes terminally.
// Return: current target before transition, reset/outcome as needed, or empty.
EpisodeActions EpisodeController::tick(double sim_time_s, TimePoint wall_now)
{
  current_sim_time_s_ = sim_time_s;
  EpisodeActions actions = tickLifecycle(wall_now);
  // Run IK/FSM only for a new observation in the ready state.
  if (!actions.empty() || state_ != EpisodeState::kReady || !latest_observation_ ||
    consumed_sample_sequence_ == last_sample_sequence_)
  {
    return actions;
  }

  consumed_sample_sequence_ = last_sample_sequence_;
  const auto & frame = latest_observation_->frame;
  if (diff_ik_source_) {diff_ik_source_->setSeed(frame.arm.positions);}

  JointTarget target;
  try {
    if (diff_ik_source_ && frame.object_frame_id != "world") {
      throw std::invalid_argument("Object pose must be in world frame");
    }
    target = waypoint_source_.jointTargetFor(phase_, frame.object_pose, place_);
  } catch (const std::exception & e) {
    actions = finishEpisode(false, "IK_FAILED", true);
    actions.diagnostics.push_back(
      DiagnosticEvent{DiagnosticEvent::Kind::kIkFailed, e.what(), last_sample_sequence_});
    return actions;
  }
  actions.target = TargetCommand{phase_, target};
  if (diff_ik_source_ && diff_ik_source_->diagnostics() &&
    logged_target_phase_ != phase_)
  {
    actions.target_diagnostics = *diff_ik_source_->diagnostics();
    logged_target_phase_ = phase_;
  }

  FsmInputs in;
  in.phase = phase_;
  in.arm = frame.arm;
  in.gripper_width_m = frame.gripper_width_m;
  in.grasp_signals = frame.grasp_signals;
  in.attachment_state = frame.attachment_state;
  in.box_x_m = frame.object_pose.x;
  in.box_y_m = frame.object_pose.y;
  in.elapsed_in_phase_s = sim_time_s - phase_start_sim_time_s_;
  in.retry_count = retry_count_;
  const auto decision = step(in, target, fsm_params_);
  if (decision.next_phase == phase_) {return actions;}

  PhaseTelemetry telemetry;
  telemetry.phase_name = phaseName(phase_);
  telemetry.duration_s = in.elapsed_in_phase_s;
  telemetry.joint_tracking_error_rad = 0.0;
  for (size_t i = 0; i < target.arm_positions.size(); ++i) {
    telemetry.joint_tracking_error_rad = std::max(
      telemetry.joint_tracking_error_rad,
      std::abs(frame.arm.positions[i] - target.arm_positions[i]));
  }
  double rotation_error = std::numeric_limits<double>::quiet_NaN();
  if (diff_ik_source_ && diff_ik_source_->diagnostics()) {
    const auto & data = *diff_ik_source_->diagnostics();
    telemetry.target_tcp_x_m = data.tcp_target.translation().x();
    telemetry.target_tcp_y_m = data.tcp_target.translation().y();
    telemetry.target_tcp_z_m = data.tcp_target.translation().z();
    telemetry.ik_position_error_m = data.ik.position_error;
    telemetry.actual_tcp_position_error_m =
      (data.tcp_target.translation() - frame.world_to_hand_tcp.translation()).norm();
    rotation_error = Eigen::Quaterniond(data.tcp_target.linear()).angularDistance(
      Eigen::Quaterniond(frame.world_to_hand_tcp.linear()));
  }
  telemetry_.append(telemetry);
  actions.transition = PhaseTransition{
    phase_, decision.next_phase, decision.exit_reason, target, telemetry, rotation_error};
  if (decision.next_phase == Phase::kRecover) {
    last_failure_reason_ = decision.exit_reason;
  }
  phase_ = decision.next_phase;
  logged_target_phase_.reset();
  phase_start_sim_time_s_ = sim_time_s;
  if (decision.is_retry) {
    actions.reset_request = beginRetry(wall_now).reset_request;
  } else if (phase_ == Phase::kFailed) {
    actions.finished = finishEpisode(false, exitReasonName(last_failure_reason_)).finished;
  } else if (phase_ == Phase::kDone) {
    actions.finished = finishEpisode(true).finished;
  }
  return actions;
}

// Current: any nonterminal state (terminal states are unchanged).
// Transition: -> kFinished or kFailed.
// Return: one EpisodeFinished, then empty actions on later calls.
EpisodeActions EpisodeController::finishEpisode(
  bool success, std::string failure_code, bool append_failed_phase)
{
  if (state_ == EpisodeState::kFinished || state_ == EpisodeState::kFailed) {
    return {};
  }
  state_ = success ? EpisodeState::kFinished : EpisodeState::kFailed;
  if (append_failed_phase) {
    PhaseTelemetry failed_phase;
    failed_phase.phase_name = phaseName(phase_);
    failed_phase.duration_s = current_sim_time_s_ - phase_start_sim_time_s_;
    telemetry_.append(std::move(failed_phase));
  }
  EpisodeActions actions;
  actions.finished = EpisodeFinished{
    success, std::move(failure_code), retry_count_, telemetry_};
  actions.diagnostics.push_back(
    DiagnosticEvent{DiagnosticEvent::Kind::kEpisodeFinished,
      actions.finished->success ? "EPISODE_SUCCEEDED" : actions.finished->failure_code, 0});
  return actions;
}

// Current: any state; used by lifecycle and observation failures.
// Transition: nonterminal -> kFailed; terminal stays unchanged.
// Return: one failed EpisodeFinished, or empty if already terminal.
EpisodeActions EpisodeController::finishFromFailure(const char * failure_code)
{
  return finishEpisode(false, failure_code == nullptr ? "UNKNOWN" : failure_code, true);
}

// Current: any state.
// Transition: none.
// Return: one observation-rejected diagnostic.
EpisodeActions EpisodeController::rejectedObservation(
  uint64_t sample_sequence, const char * code) const
{
  EpisodeActions actions;
  actions.diagnostics.push_back(
    DiagnosticEvent{DiagnosticEvent::Kind::kObservationRejected,
      code == nullptr ? "OBSERVATION_REJECTED" : code, sample_sequence});
  return actions;
}

}  // namespace task_executor
