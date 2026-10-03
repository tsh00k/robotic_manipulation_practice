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

#include <cmath>
#include <stdexcept>

#include "mujoco_perception/object_tracker.hpp"

namespace mujoco_perception
{
namespace
{

TrackingSample sample(uint64_t sequence, double time)
{
  TrackingSample result;
  result.session = 7;
  result.generation = 1;
  result.sequence = sequence;
  result.time_s = time;
  return result;
}

PoseEstimate pose(double x = 0.5, double confidence = 0.9)
{
  PoseEstimate result;
  result.geometry_valid = true;
  result.position = Eigen::Vector3d(x, 0.0, 0.24);
  result.orientation = Eigen::Quaterniond::Identity();
  result.confidence = confidence;
  result.residual_m = 0.001;
  result.inlier_ratio = 0.95;
  result.point_count = 100;
  return result;
}

TEST(ObjectTracker, OneValidCandidateIsMeasured)
{
  ObjectTracker tracker;
  const auto result = tracker.update(sample(1, 1.0), {pose()});
  EXPECT_EQ(result.state, EvidenceState::kMeasured);
  EXPECT_TRUE(result.reason.empty());
  EXPECT_EQ(result.diagnostic_stage, DiagnosticStage::kNone);
  EXPECT_EQ(result.candidate_count, 1U);
  EXPECT_EQ(result.eligible_candidate_count, 1U);
  EXPECT_EQ(result.measurement_sequence, 1U);
  EXPECT_FALSE(result.attachment_valid);
  EXPECT_EQ(result.grasp_state, GraspState::kNotAttached);
}

TEST(ObjectTracker, EmptyCandidatesAreOccludedWithoutPrediction)
{
  ObjectTracker tracker;
  tracker.update(sample(1, 1.0), {pose()});
  const auto result = tracker.update(sample(2, 1.1), {});
  EXPECT_EQ(result.state, EvidenceState::kOccluded);
  EXPECT_EQ(result.reason, "NO_CANDIDATE");
  EXPECT_EQ(result.diagnostic_stage, DiagnosticStage::kGeometry);
  EXPECT_EQ(result.measurement_sequence, 1U);
  EXPECT_FALSE(result.attachment_valid);
}

TEST(ObjectTracker, InvalidAndAmbiguousCandidatesAreRejected)
{
  ObjectTracker tracker;
  auto invalid = pose();
  invalid.geometry_valid = false;
  auto result = tracker.update(sample(1, 1.0), {invalid});
  EXPECT_EQ(result.state, EvidenceState::kRejected);
  EXPECT_EQ(result.reason, "CANDIDATE_INVALID");
  EXPECT_EQ(result.diagnostic_stage, DiagnosticStage::kGeometry);
  result = tracker.update(sample(2, 1.1), {pose(), pose(0.52)});
  EXPECT_EQ(result.state, EvidenceState::kRejected);
  EXPECT_EQ(result.reason, "MULTIPLE_CANDIDATES");
  EXPECT_EQ(result.diagnostic_stage, DiagnosticStage::kAssociation);
  EXPECT_EQ(result.candidate_count, 2U);
  EXPECT_EQ(result.eligible_candidate_count, 2U);
}

TEST(ObjectTracker, ConfidenceGateIsTheOnlyTrackerQualityGate)
{
  TrackerConfig config;
  config.min_confidence = 0.8;
  ObjectTracker tracker(config);
  EXPECT_EQ(
    tracker.update(sample(1, 1.0), {pose(0.5, 0.79)}).reason, "CANDIDATE_INVALID");
  EXPECT_EQ(
    tracker.update(sample(2, 1.1), {pose(0.5, 0.8)}).state, EvidenceState::kMeasured);
}

TEST(ObjectTracker, AttachmentStateComesOnlyFromBridge)
{
  ObjectTracker tracker;
  auto attached = sample(1, 1.0);
  attached.attachment_state = AttachmentState::kAttached;
  auto result = tracker.update(attached, {});
  EXPECT_EQ(result.grasp_state, GraspState::kHeld);
  EXPECT_TRUE(result.attachment_valid);

  auto flicker = attached;
  flicker.sequence = 2;
  flicker.time_s = 1.1;
  result = tracker.update(flicker, {});
  EXPECT_EQ(result.grasp_state, GraspState::kHeld);
  EXPECT_TRUE(result.attachment_valid);
}

TEST(ObjectTracker, ReleasedBridgeStateEndsAttachmentWithoutInference)
{
  ObjectTracker tracker;
  auto attached = sample(1, 1.0);
  attached.attachment_state = AttachmentState::kAttached;
  ASSERT_TRUE(tracker.update(attached, {}).attachment_valid);
  auto released = sample(2, 1.1);
  released.attachment_state = AttachmentState::kReleased;
  const auto result = tracker.update(released, {});
  EXPECT_EQ(result.grasp_state, GraspState::kReleased);
  EXPECT_FALSE(result.attachment_valid);
  EXPECT_EQ(result.state, EvidenceState::kOccluded);
}

TEST(ObjectTracker, AttachedStateDoesNotCreatePredictedPose)
{
  ObjectTracker tracker;
  auto attached = sample(1, 1.0);
  attached.attachment_state = AttachmentState::kAttached;
  const auto result = tracker.update(attached, {});
  EXPECT_EQ(result.state, EvidenceState::kOccluded);
  EXPECT_TRUE(std::isnan(result.estimate.residual_m));
}

TEST(ObjectTracker, LifecycleAndInputErrorsAreExplicit)
{
  ObjectTracker tracker;
  tracker.reset(7, 1);
  auto wrong = sample(1, 1.0);
  wrong.session = 8;
  auto result = tracker.update(wrong, {pose()});
  EXPECT_EQ(result.state, EvidenceState::kRejected);
  EXPECT_EQ(result.reason, "LIFECYCLE_MISMATCH");
  auto invalid = sample(2, 1.1);
  invalid.input_failure = "MISSING_ROBOT_TRANSFORM";
  result = tracker.update(invalid, {});
  EXPECT_EQ(result.state, EvidenceState::kRejected);
  EXPECT_EQ(result.reason, "MISSING_ROBOT_TRANSFORM");
  EXPECT_EQ(result.diagnostic_stage, DiagnosticStage::kInput);
}

TEST(ObjectTracker, ReleaseAndResetRequireFreshMeasurement)
{
  ObjectTracker tracker;
  ASSERT_EQ(tracker.update(sample(1, 1.0), {pose()}).state, EvidenceState::kMeasured);
  tracker.reset(7, 1);
  EXPECT_EQ(tracker.update(sample(2, 1.1), {}).state, EvidenceState::kOccluded);
  auto fresh = sample(3, 1.2);
  EXPECT_EQ(tracker.update(fresh, {pose(0.51)}).state, EvidenceState::kMeasured);
}

TEST(ObjectTracker, OutOfOrderSamplesCannotRefreshHistory)
{
  ObjectTracker tracker;
  tracker.update(sample(5, 1.0), {pose()});
  const auto result = tracker.update(sample(4, 1.1), {pose(0.51)});
  EXPECT_EQ(result.state, EvidenceState::kRejected);
  EXPECT_EQ(result.reason, "OUT_OF_ORDER");
}

TEST(ObjectTracker, InvalidConfigurationIsRejected)
{
  TrackerConfig config;
  config.min_confidence = -0.1;
  EXPECT_THROW(ObjectTracker tracker(config), std::invalid_argument);
  config.min_confidence = 1.1;
  EXPECT_THROW(ObjectTracker tracker(config), std::invalid_argument);
}

}  // namespace
}  // namespace mujoco_perception
