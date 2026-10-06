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

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

#include "task_executor/episode_controller.hpp"

namespace task_executor
{
namespace
{

class FakeWaypointSource final : public WaypointSource
{
public:
  bool throw_on_target = false;
  JointTarget jointTargetFor(Phase phase, const ObjectPose & object_pose) const override
  {
    if (throw_on_target) {throw std::runtime_error("unreachable waypoint");}
    JointTarget target;
    target.arm_positions[0] = object_pose.x + static_cast<double>(phase);
    target.gripper_width_m = 0.04;
    return target;
  }
};

ObservationEnvelope observation(uint64_t generation, uint64_t sequence)
{
  ObservationEnvelope result;
  result.bridge_session = 7;
  result.generation = generation;
  result.sample_sequence = sequence;
  result.frame.object_pose.x = 0.25;
  return result;
}

void ready(EpisodeController & controller, uint64_t generation = 4)
{
  const auto now = EpisodeController::TimePoint{};
  const auto start = controller.startEpisode(now);
  ASSERT_TRUE(controller.onResetRequestSent(start.reset_request->request_id));
  ASSERT_TRUE(
    controller.onResetResponse(
      start.reset_request->request_id, ResetReceipt{true, 7, generation}, 0.0).empty());
}

TEST(EpisodeController, TransitionPublishesOldPhaseTargetBeforeRecordingNewPhase)
{
  FakeWaypointSource source;
  EpisodeController controller(source);
  ready(controller);
  auto frame = observation(4, 1);
  frame.frame.arm.positions[0] = 0.25;
  controller.onObservation(frame, EpisodeController::TimePoint{}, 0.5);
  const auto actions = controller.tick(0.5, EpisodeController::TimePoint{});
  ASSERT_TRUE(actions.target.has_value());
  ASSERT_TRUE(actions.transition.has_value());
  EXPECT_EQ(actions.target->phase, Phase::kHome);
  EXPECT_EQ(actions.transition->from, Phase::kHome);
  EXPECT_EQ(actions.transition->to, Phase::kPregrasp);
  EXPECT_EQ(controller.phase(), Phase::kPregrasp);
  ASSERT_EQ(controller.telemetry().phases.size(), 1u);
  EXPECT_EQ(controller.telemetry().phases.front().phase_name, "HOME");
  EXPECT_FALSE(controller.tick(0.6, EpisodeController::TimePoint{}).target.has_value());
}

TEST(EpisodeController, IkFailureProducesNoTargetAndOneOutcome)
{
  FakeWaypointSource source;
  source.throw_on_target = true;
  EpisodeController controller(source);
  ready(controller);
  controller.onObservation(observation(4, 1), EpisodeController::TimePoint{}, 0.5);
  const auto actions = controller.tick(0.5, EpisodeController::TimePoint{});
  EXPECT_FALSE(actions.target.has_value());
  ASSERT_TRUE(actions.finished.has_value());
  EXPECT_EQ(actions.finished->failure_code, "IK_FAILED");
  ASSERT_EQ(actions.finished->telemetry.phases.size(), 1u);
  EXPECT_EQ(actions.finished->telemetry.phases.front().phase_name, "HOME");
  EXPECT_TRUE(controller.tick(0.6, EpisodeController::TimePoint{}).empty());
}

TEST(EpisodeController, LateObservationCannotReviveExpiredWatchdog)
{
  FakeWaypointSource source;
  EpisodeController controller(source);
  ready(controller);
  const auto late = controller.onObservation(
    observation(4, 1), EpisodeController::TimePoint{} + std::chrono::seconds(5), 5.0);
  ASSERT_TRUE(late.finished.has_value());
  EXPECT_EQ(late.finished->failure_code, "OBSERVATION_STALE");
  EXPECT_FALSE(late.target.has_value());
}

TEST(EpisodeController, TimeoutRetryAndRetryLimitPreserveFailureReason)
{
  FakeWaypointSource source;
  FsmParams params;
  params.phase_timeout_s = 0.1;
  params.max_retries = 1;
  EpisodeController controller(source, params);
  ready(controller);
  controller.onObservation(observation(4, 1), EpisodeController::TimePoint{}, 0.2);
  const auto timeout = controller.tick(0.2, EpisodeController::TimePoint{});
  ASSERT_TRUE(timeout.target.has_value());
  ASSERT_TRUE(timeout.transition.has_value());
  EXPECT_EQ(timeout.transition->to, Phase::kRecover);
  controller.onObservation(observation(4, 2), EpisodeController::TimePoint{}, 0.25);
  const auto retry = controller.tick(0.25, EpisodeController::TimePoint{});
  ASSERT_TRUE(retry.target.has_value());
  ASSERT_TRUE(retry.reset_request.has_value());
  EXPECT_EQ(retry.target->phase, Phase::kRecover);
  EXPECT_EQ(controller.retryCount(), 1);
  EXPECT_EQ(controller.phase(), Phase::kHome);
  ASSERT_TRUE(controller.onResetRequestSent(retry.reset_request->request_id));
  ASSERT_TRUE(
    controller.onResetResponse(
      retry.reset_request->request_id, ResetReceipt{true, 7, 5}, 0.3).empty());
  controller.onObservation(observation(5, 1), EpisodeController::TimePoint{}, 0.5);
  EXPECT_EQ(
    controller.tick(0.5, EpisodeController::TimePoint{}).transition->to, Phase::kRecover);
  controller.onObservation(observation(5, 2), EpisodeController::TimePoint{}, 0.55);
  const auto failed = controller.tick(0.55, EpisodeController::TimePoint{});
  ASSERT_TRUE(failed.target.has_value());
  ASSERT_TRUE(failed.finished.has_value());
  EXPECT_EQ(failed.finished->failure_code, "TIMEOUT");
  EXPECT_EQ(failed.finished->retry_count, 1);
  EXPECT_TRUE(controller.tick(0.6, EpisodeController::TimePoint{}).empty());
}

TEST(EpisodeController, IdleTickHasNoActions)
{
  FakeWaypointSource source;
  EpisodeController controller(source);
  EXPECT_TRUE(controller.tick(0.0, EpisodeController::TimePoint{}).empty());
  EXPECT_EQ(controller.state(), EpisodeState::kIdle);
}

TEST(EpisodeController, PendingResetOffersSameRequestUntilSent)
{
  FakeWaypointSource source;
  EpisodeController controller(source);
  const auto now = EpisodeController::TimePoint{};

  const auto start = controller.startEpisode(now);
  ASSERT_TRUE(start.reset_request.has_value());
  const auto request_id = start.reset_request->request_id;
  EXPECT_EQ(controller.state(), EpisodeState::kResetPending);
  EXPECT_TRUE(controller.onResetResponse(request_id, ResetReceipt{true, 7, 1}).empty());
  EXPECT_EQ(controller.state(), EpisodeState::kResetPending);

  const auto retry = controller.tick(0.0, now + std::chrono::seconds(1));
  ASSERT_TRUE(retry.reset_request.has_value());
  EXPECT_EQ(retry.reset_request->request_id, request_id);
  EXPECT_TRUE(controller.onResetRequestSent(request_id));
  EXPECT_FALSE(controller.onResetRequestSent(request_id));
  EXPECT_EQ(controller.state(), EpisodeState::kAwaitingResetResponse);
  EXPECT_TRUE(controller.tick(0.0, now + std::chrono::seconds(2)).empty());
}

TEST(EpisodeController, NewStartInvalidatesOldRequestAndReceipt)
{
  FakeWaypointSource source;
  EpisodeController controller(source);
  const auto now = EpisodeController::TimePoint{};

  const auto first = controller.startEpisode(now);
  ASSERT_TRUE(controller.onResetRequestSent(first.reset_request->request_id));

  const auto second = controller.startEpisode(now);
  ASSERT_TRUE(second.reset_request.has_value());
  EXPECT_EQ(second.reset_request->request_id, 2u);
  EXPECT_FALSE(controller.onResetRequestSent(first.reset_request->request_id));
  EXPECT_TRUE(controller.onResetResponse(1, ResetReceipt{true, 7, 1}).empty());
  EXPECT_EQ(controller.state(), EpisodeState::kResetPending);
  EXPECT_TRUE(controller.onResetRequestSent(second.reset_request->request_id));
  EXPECT_TRUE(controller.onResetResponse(1, ResetReceipt{true, 7, 1}).empty());
  EXPECT_EQ(controller.state(), EpisodeState::kAwaitingResetResponse);
}

TEST(EpisodeController, ResetFailureFinishesExactlyOnce)
{
  FakeWaypointSource source;
  EpisodeController controller(source);
  const auto start = controller.startEpisode(EpisodeController::TimePoint{});
  ASSERT_TRUE(controller.onResetRequestSent(start.reset_request->request_id));

  const auto failed = controller.onResetResponse(
    start.reset_request->request_id, ResetReceipt{false, 7, 0});
  ASSERT_TRUE(failed.finished.has_value());
  EXPECT_FALSE(failed.finished->success);
  EXPECT_EQ(failed.finished->failure_code, "RESET_FAILED");
  EXPECT_EQ(controller.state(), EpisodeState::kFailed);
  EXPECT_TRUE(controller.tick(0.0, EpisodeController::TimePoint{}).empty());
  EXPECT_TRUE(
    controller.onResetResponse(
      start.reset_request->request_id, ResetReceipt{false, 7, 0}).empty());
}

TEST(EpisodeController, ResetSuccessRequiresFreshObservationBeforeTarget)
{
  FakeWaypointSource source;
  EpisodeController controller(source);
  const auto now = EpisodeController::TimePoint{};
  const auto start = controller.startEpisode(now);
  ASSERT_TRUE(controller.onResetRequestSent(start.reset_request->request_id));

  EXPECT_TRUE(
    controller.onResetResponse(
      start.reset_request->request_id, ResetReceipt{true, 7, 4}).empty());
  EXPECT_EQ(controller.state(), EpisodeState::kAwaitingObservation);
  EXPECT_TRUE(controller.tick(0.0, now).empty());

  const auto rejected = controller.onObservation(observation(3, 10), now);
  ASSERT_FALSE(rejected.diagnostics.empty());
  EXPECT_EQ(rejected.diagnostics.front().kind, DiagnosticEvent::Kind::kObservationRejected);
  EXPECT_EQ(controller.state(), EpisodeState::kAwaitingObservation);
}

TEST(EpisodeController, FreshObservationEnablesWaypointOnTick)
{
  FakeWaypointSource source;
  EpisodeController controller(source);
  const auto now = EpisodeController::TimePoint{};
  const auto start = controller.startEpisode(now);
  ASSERT_TRUE(controller.onResetRequestSent(start.reset_request->request_id));
  ASSERT_TRUE(
    controller.onResetResponse(
      start.reset_request->request_id, ResetReceipt{true, 7, 4}).empty());

  const auto accepted = controller.onObservation(observation(4, 10), now);
  ASSERT_FALSE(accepted.diagnostics.empty());
  EXPECT_EQ(accepted.diagnostics.front().kind, DiagnosticEvent::Kind::kObservationAccepted);
  EXPECT_EQ(controller.state(), EpisodeState::kReady);

  const auto tick = controller.tick(3.0, now);
  ASSERT_TRUE(tick.target.has_value());
  EXPECT_EQ(tick.target->phase, Phase::kHome);
  EXPECT_DOUBLE_EQ(tick.target->target.arm_positions[0], 0.25);
  EXPECT_DOUBLE_EQ(tick.target->target.gripper_width_m, 0.04);
  EXPECT_FALSE(controller.tick(3.05, now + std::chrono::milliseconds(50)).target.has_value());
  EXPECT_EQ(controller.telemetry().phases.size(), 0u);
  ASSERT_EQ(controller.onObservation(observation(4, 11), now).diagnostics.size(), 1u);
  EXPECT_TRUE(controller.tick(3.1, now).target.has_value());
}

TEST(EpisodeController, SupersededGenerationFailsAndOldSamplesNeverPublish)
{
  FakeWaypointSource source;
  EpisodeController controller(source);
  const auto now = EpisodeController::TimePoint{};
  const auto start = controller.startEpisode(now);
  ASSERT_TRUE(controller.onResetRequestSent(start.reset_request->request_id));
  ASSERT_TRUE(
    controller.onResetResponse(
      start.reset_request->request_id, ResetReceipt{true, 7, 4}).empty());

  const auto failed = controller.onObservation(observation(5, 11), now);
  ASSERT_TRUE(failed.finished.has_value());
  EXPECT_EQ(failed.finished->failure_code, "RESET_SUPERSEDED");
  EXPECT_EQ(controller.state(), EpisodeState::kFailed);
  EXPECT_TRUE(controller.tick(0.0, now).empty());
}

TEST(EpisodeController, TerminalStateSuppressesLaterActions)
{
  FakeWaypointSource source;
  EpisodeController controller(source);
  const auto now = EpisodeController::TimePoint{};
  const auto start = controller.startEpisode(now);
  ASSERT_TRUE(controller.onResetRequestSent(start.reset_request->request_id));
  ASSERT_TRUE(
    controller.onResetResponse(
      start.reset_request->request_id, ResetReceipt{true, 7, 4}).empty());
  ASSERT_EQ(controller.onObservation(observation(4, 10), now).diagnostics.size(), 1u);

  const auto finished = controller.finishEpisode(true);
  ASSERT_TRUE(finished.finished.has_value());
  EXPECT_TRUE(finished.finished->success);
  EXPECT_EQ(controller.state(), EpisodeState::kFinished);
  EXPECT_TRUE(controller.tick(0.0, now).empty());
  EXPECT_TRUE(controller.finishEpisode(true).empty());
  EXPECT_TRUE(controller.onObservation(observation(4, 11), now).empty());
}

TEST(EpisodeController, TimeoutsBelongToTheCurrentEpisodeState)
{
  FakeWaypointSource source;
  const auto now = EpisodeController::TimePoint{};
  EpisodeController pending(source);
  pending.startEpisode(now);
  EXPECT_TRUE(pending.tick(0.0, now + std::chrono::seconds(4)).reset_request.has_value());
  const auto unavailable = pending.tick(0.0, now + std::chrono::seconds(5));
  ASSERT_TRUE(unavailable.finished.has_value());
  EXPECT_EQ(unavailable.finished->failure_code, "RESET_UNAVAILABLE");
  EXPECT_FALSE(unavailable.reset_request.has_value());
  EXPECT_TRUE(pending.tick(0.0, now + std::chrono::seconds(6)).empty());

  EpisodeController awaiting(source);
  const auto start = awaiting.startEpisode(now);
  ASSERT_TRUE(awaiting.onResetRequestSent(start.reset_request->request_id));
  ASSERT_TRUE(
    awaiting.onResetResponse(
      start.reset_request->request_id, ResetReceipt{true, 7, 4}).empty());
  const auto stale = awaiting.tick(0.0, now + std::chrono::seconds(5));
  ASSERT_TRUE(stale.finished.has_value());
  EXPECT_EQ(stale.finished->failure_code, "OBSERVATION_STALE");

  // A longer wait before the first observation does not lengthen the stream watchdog.
  EpisodeController patient(
    source, {}, nullptr, std::chrono::seconds(5), std::chrono::seconds(12));
  const auto patient_start = patient.startEpisode(now);
  ASSERT_TRUE(patient.onResetRequestSent(patient_start.reset_request->request_id));
  ASSERT_TRUE(
    patient.onResetResponse(
      patient_start.reset_request->request_id, ResetReceipt{true, 7, 4}).empty());
  EXPECT_TRUE(patient.tick(0.0, now + std::chrono::seconds(11)).empty());
  const auto patient_stale = patient.tick(0.0, now + std::chrono::seconds(12));
  ASSERT_TRUE(patient_stale.finished.has_value());
  EXPECT_EQ(patient_stale.finished->failure_code, "OBSERVATION_STALE");
  EXPECT_TRUE(
    patient.onObservation(observation(4, 1), now + std::chrono::seconds(20)).diagnostics.empty())
    << "a finished episode is not revived by a late observation";

  EpisodeController patient_ready(
    source, {}, nullptr, std::chrono::seconds(5), std::chrono::seconds(12));
  const auto patient_ready_start = patient_ready.startEpisode(now);
  ASSERT_TRUE(patient_ready.onResetRequestSent(patient_ready_start.reset_request->request_id));
  ASSERT_TRUE(
    patient_ready.onResetResponse(
      patient_ready_start.reset_request->request_id, ResetReceipt{true, 7, 4}).empty());
  const auto admitted =
    patient_ready.onObservation(observation(4, 10), now + std::chrono::seconds(9));
  ASSERT_EQ(admitted.diagnostics.size(), 1u);
  const auto patient_stream_stale = patient_ready.tick(0.0, now + std::chrono::seconds(14));
  ASSERT_TRUE(patient_stream_stale.finished.has_value()) << "the 5 s stream watchdog still applies";

  EpisodeController ready(source);
  const auto ready_start = ready.startEpisode(now);
  ASSERT_TRUE(ready.onResetRequestSent(ready_start.reset_request->request_id));
  ASSERT_TRUE(
    ready.onResetResponse(
      ready_start.reset_request->request_id, ResetReceipt{true, 7, 4}).empty());
  ASSERT_EQ(ready.onObservation(observation(4, 10), now).diagnostics.size(), 1u);
  const auto stream_stale = ready.tick(0.0, now + std::chrono::seconds(5));
  ASSERT_TRUE(stream_stale.finished.has_value());
  EXPECT_EQ(stream_stale.finished->failure_code, "OBSERVATION_STALE");
  EXPECT_FALSE(stream_stale.target.has_value());
}

TEST(EpisodeController, SessionAndSequenceGatePreservesLatestFreshObservation)
{
  FakeWaypointSource source;
  EpisodeController controller(source);
  const auto now = EpisodeController::TimePoint{};
  const auto start = controller.startEpisode(now);
  ASSERT_TRUE(controller.onResetRequestSent(start.reset_request->request_id));
  ASSERT_TRUE(
    controller.onResetResponse(
      start.reset_request->request_id, ResetReceipt{true, 7, 4}).empty());

  auto wrong_session = observation(4, 10);
  wrong_session.bridge_session = 6;
  EXPECT_EQ(
    controller.onObservation(wrong_session, now).diagnostics.front().code,
    "BRIDGE_SESSION_MISMATCH");
  ASSERT_EQ(controller.onObservation(observation(4, 10), now).diagnostics.size(), 1u);
  EXPECT_EQ(
    controller.onObservation(observation(4, 10), now).diagnostics.front().code,
    "OBSERVATION_STALE");
  EXPECT_EQ(
    controller.onObservation(observation(3, 11), now).diagnostics.front().code,
    "OBSERVATION_STALE");
  ASSERT_TRUE(controller.latestObservation().has_value());
  EXPECT_EQ(controller.latestObservation()->sample_sequence, 10u);
  EXPECT_TRUE(controller.tick(0.0, now).target.has_value());
}

TEST(EpisodeController, RetryInvalidatesOldReceiptAndGeneration)
{
  FakeWaypointSource source;
  EpisodeController controller(source);
  const auto now = EpisodeController::TimePoint{};
  const auto first = controller.startEpisode(now);
  ASSERT_TRUE(controller.onResetRequestSent(first.reset_request->request_id));
  ASSERT_TRUE(
    controller.onResetResponse(
      first.reset_request->request_id, ResetReceipt{true, 7, 4}).empty());
  ASSERT_EQ(controller.onObservation(observation(4, 10), now).diagnostics.size(), 1u);

  const auto retry = controller.beginRetry(now + std::chrono::seconds(1));
  ASSERT_TRUE(retry.reset_request.has_value());
  EXPECT_EQ(controller.retryCount(), 1);
  EXPECT_EQ(controller.state(), EpisodeState::kResetPending);
  EXPECT_TRUE(
    controller.onResetResponse(
      first.reset_request->request_id, ResetReceipt{true, 7, 4}).empty());
  EXPECT_EQ(
    controller.onObservation(observation(4, 11), now).diagnostics.front().code,
    "EPISODE_NOT_READY");

  ASSERT_TRUE(controller.onResetRequestSent(retry.reset_request->request_id));
  ASSERT_TRUE(
    controller.onResetResponse(
      retry.reset_request->request_id, ResetReceipt{true, 8, 1}).empty());
  EXPECT_EQ(
    controller.onObservation(observation(4, 12), now).diagnostics.front().code,
    "BRIDGE_SESSION_MISMATCH");
  auto fresh = observation(1, 1);
  fresh.bridge_session = 8;
  EXPECT_EQ(
    controller.onObservation(fresh, now).diagnostics.front().kind,
    DiagnosticEvent::Kind::kObservationAccepted);
  EXPECT_EQ(controller.generation(), 1u);
  EXPECT_EQ(controller.bridgeSession(), 8u);
  const auto finished = controller.finishEpisode(false, "PLACE_MISSED");
  ASSERT_TRUE(finished.finished.has_value());
  EXPECT_EQ(finished.finished->retry_count, 1);
}

TEST(EpisodeController, ResponseWaitTimesOutAndLateReceiptIsIgnored)
{
  FakeWaypointSource source;
  EpisodeController controller(source);
  const auto now = EpisodeController::TimePoint{};
  const auto start = controller.startEpisode(now);
  ASSERT_TRUE(controller.onResetRequestSent(start.reset_request->request_id));

  const auto timeout = controller.tick(0.0, now + std::chrono::seconds(5));
  ASSERT_TRUE(timeout.finished.has_value());
  EXPECT_EQ(timeout.finished->failure_code, "RESET_UNAVAILABLE");
  EXPECT_TRUE(
    controller.onResetResponse(
      start.reset_request->request_id, ResetReceipt{true, 7, 4}).empty());
  EXPECT_TRUE(controller.tick(0.0, now + std::chrono::seconds(6)).empty());
}

}  // namespace
}  // namespace task_executor
