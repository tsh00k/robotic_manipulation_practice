// Copyright 2026 anby
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "mujoco_perception/object_tracker.hpp"

namespace mujoco_perception
{
namespace
{

TrackingSample sample(uint64_t sequence, double time, double tcp_z = 0.24, double width = 0.08)
{
  TrackingSample s;
  s.session = 7;
  s.generation = 1;
  s.sequence = sequence;
  s.time_s = time;
  s.world_from_tcp.translation() = Eigen::Vector3d(0.5, 0.0, tcp_z);
  s.gripper_width_m = width;
  return s;
}

PoseEstimate pose(double z = 0.24, double x = 0.5, std::size_t points = 120)
{
  PoseEstimate p;
  p.accepted = true;
  p.position = Eigen::Vector3d(x, 0.0, z);
  p.point_count = points;
  p.confidence = 0.8;
  p.residual_m = 0.001;
  p.inlier_ratio = 1.0;
  return p;
}

TEST(ObjectTracker, ApproachOcclusionPartialReappearanceAndExpiry)
{
  ObjectTracker tracker;
  auto r = tracker.update(sample(1, 1.0), {pose()});
  ASSERT_EQ(r.state, EvidenceState::kMeasured);
  r = tracker.update(sample(2, 1.1), {});
  ASSERT_EQ(r.state, EvidenceState::kPredicted);
  EXPECT_FALSE(r.estimate.accepted);
  EXPECT_TRUE(std::isnan(r.estimate.residual_m));
  EXPECT_EQ(r.estimate.point_count, 0U);
  EXPECT_EQ(r.measurement_sequence, 1U);
  EXPECT_NEAR(r.prediction_age_s, 0.1, 1e-12);
  const double uncertainty = r.uncertainty_m;
  r = tracker.update(sample(3, 1.2), {});
  EXPECT_GT(r.uncertainty_m, uncertainty);
  r = tracker.update(sample(4, 1.25), {pose(0.24, 0.502, 45)});
  ASSERT_EQ(r.state, EvidenceState::kMeasured);
  EXPECT_EQ(r.measurement_sequence, 4U);
  EXPECT_EQ(r.prediction_age_s, 0.0);
  r = tracker.update(sample(5, 1.6), {});
  EXPECT_EQ(r.state, EvidenceState::kOccluded);
  EXPECT_FALSE(r.pose_valid);
}

TEST(ObjectTracker, NoHistoryOrPixelSupportCannotMeasure)
{
  ObjectTracker tracker;
  auto p = pose();
  p.point_count = 0;
  const auto r = tracker.update(sample(1, 1), {p});
  EXPECT_EQ(r.state, EvidenceState::kOccluded);
  EXPECT_FALSE(r.pose_valid);
  EXPECT_EQ(r.measurement_sequence, 0U);
}

TEST(ObjectTracker, SelectsMatchingCandidateInsteadOfLargestCluster)
{
  ObjectTracker tracker;
  tracker.update(sample(1, 1), {pose()});
  const auto r = tracker.update(sample(2, 1.1), {pose(0.24, 0.69, 1000), pose()});
  EXPECT_EQ(r.state, EvidenceState::kMeasured);
  EXPECT_EQ(r.candidate_index, 1);
}

TEST(ObjectTracker, RejectsAmbiguityAndWrongTargetWithoutPrediction)
{
  ObjectTracker tracker;
  EXPECT_EQ(
    tracker.update(sample(1, 1), {pose(), pose(0.24, 0.51)}).reason,
    "ASSOCIATION_AMBIGUOUS");
  tracker.update(sample(2, 1.1), {pose()});
  auto r = tracker.update(sample(3, 1.2), {pose(0.24, 0.69)});
  EXPECT_EQ(r.reason, "ASSOCIATION_JUMP");
  EXPECT_FALSE(r.pose_valid);
  r = tracker.update(sample(4, 1.3), {pose(), pose(0.24, 0.51)});
  EXPECT_EQ(r.reason, "ASSOCIATION_AMBIGUOUS");
}

void attach(ObjectTracker & tracker)
{
  tracker.update(sample(1, 1), {pose()});
  tracker.update(sample(2, 1.1, 0.24, 0.04), {});
  const auto result = tracker.update(sample(3, 1.5, 0.24, 0.04), {});
  ASSERT_TRUE(result.attachment_valid);
  EXPECT_EQ(result.grasp_state, GraspState::kHeld);
}

TEST(ObjectTracker, StableClosureAttachesWithoutVisibleComotion)
{
  ObjectTracker tracker;
  attach(tracker);
  const auto r = tracker.update(sample(4, 1.7, 0.32, 0.04), {});
  ASSERT_EQ(r.state, EvidenceState::kPredicted);
  EXPECT_NEAR(r.estimate.position.z(), 0.32, 1e-12);
  EXPECT_EQ(r.measurement_sequence, 1U);
  EXPECT_EQ(r.attachment_measurement_sequence, 1U);
  EXPECT_TRUE(std::isnan(r.estimate.residual_m));
  EXPECT_FALSE(tracker.anchorToSupport(sample(5, 1.8, 0.32, 0.04)));
}

TEST(ObjectTracker, VisibleDropInvalidatesAttachmentButRetainsMeasurement)
{
  ObjectTracker tracker;
  attach(tracker);
  auto r = tracker.update(sample(4, 1.7, 0.32, 0.04), {pose(0.25)});
  ASSERT_EQ(r.state, EvidenceState::kMeasured);
  EXPECT_FALSE(r.attachment_valid);
  EXPECT_EQ(r.grasp_reason, "VISIBLE_ATTACHMENT_DEVIATION");
  EXPECT_NEAR(r.estimate.position.z(), 0.25, 1e-12);
  r = tracker.update(sample(5, 1.8, 0.35, 0.08), {pose(0.24)});
  EXPECT_EQ(r.state, EvidenceState::kMeasured);
  EXPECT_TRUE(tracker.anchorToSupport(sample(5, 1.4)));
}

TEST(ObjectTracker, ReleaseStopsTcpAttachmentAndExpiresSooner)
{
  ObjectTracker tracker;
  attach(tracker);
  tracker.update(sample(4, 1.6, 0.28, 0.04), {});
  auto r = tracker.update(sample(5, 1.7, 0.28, 0.08), {});
  EXPECT_FALSE(r.attachment_valid);
  EXPECT_EQ(r.grasp_state, GraspState::kReleased);
  EXPECT_EQ(r.state, EvidenceState::kPredicted);
  EXPECT_NEAR(r.estimate.position.z(), 0.28, 1e-12);
  r = tracker.update(sample(6, 1.9, 0.4, 0.08), {});
  EXPECT_EQ(r.state, EvidenceState::kOccluded);
}

TEST(ObjectTracker, ReleaseReacquiresObjectFarFromOriginalGrasp)
{
  ObjectTracker tracker;
  attach(tracker);
  auto s = sample(4, 1.7, 0.24, 0.04);
  s.world_from_tcp.translation().y() = 0.3;
  tracker.update(s, {});
  s.sequence = 5;
  s.time_s = 1.8;
  s.gripper_width_m = 0.08;
  tracker.update(s, {});
  auto p = pose();
  p.position.y() = 0.3;
  s.sequence = 6;
  s.time_s = 2.0;
  const auto r = tracker.update(s, {p});
  EXPECT_EQ(r.state, EvidenceState::kMeasured);
  EXPECT_FALSE(r.attachment_valid);
}

TEST(ObjectTracker, OpeningDetachesBeforeFullyOpenAndAllowsTableReappearance)
{
  ObjectTracker tracker;
  attach(tracker);
  auto s = sample(4, 1.7, 0.29, 0.051);
  auto r = tracker.update(s, {});
  EXPECT_FALSE(r.attachment_valid);
  EXPECT_EQ(r.grasp_state, GraspState::kReleased);
  s.sequence = 5;
  s.time_s = 1.8;
  s.gripper_width_m = 0.08;
  tracker.update(s, {});
  EXPECT_TRUE(tracker.anchorToSupport(s));
  s.sequence = 6;
  s.time_s = 2.0;
  r = tracker.update(s, {pose()});
  EXPECT_EQ(r.state, EvidenceState::kMeasured);
}

TEST(ObjectTracker, OpenAfterGeometryConflictReacquiresTableTopMeasurement)
{
  ObjectTracker tracker;
  attach(tracker);
  tracker.update(sample(4, 1.6, 0.29, 0.04), {pose(0.29)});
  auto conflict = pose();
  conflict.accepted = false;
  conflict.model_conflict = true;
  auto r = tracker.update(sample(5, 1.7, 0.29, 0.04), {conflict});
  ASSERT_EQ(r.grasp_state, GraspState::kInvalidated);
  EXPECT_FALSE(tracker.anchorToSupport(sample(6, 1.8, 0.29, 0.04)));

  const auto opened = sample(6, 1.8, 0.35, 0.08);
  tracker.observeRobot(opened);
  EXPECT_TRUE(tracker.anchorToSupport(opened));
  r = tracker.update(opened, {});
  EXPECT_FALSE(r.pose_valid);
  EXPECT_FALSE(r.attachment_valid);
  EXPECT_EQ(r.measurement_sequence, 4U);

  pcl::PointCloud<pcl::PointXYZ> top;
  for (int x = 0; x <= 10; ++x) {
    for (int y = 0; y <= 10; ++y) {
      top.push_back(pcl::PointXYZ(0.48f + x * 0.004f, -0.02f + y * 0.004f, 0.26f));
    }
  }
  BoxModel model;
  model.anchor_z_to_plane = false;
  EXPECT_FALSE(estimateBoxPose(top, model, 0.22).accepted);
  model.anchor_z_to_plane = tracker.anchorToSupport(opened);
  const auto measurement = estimateBoxPose(top, model, 0.22);
  ASSERT_TRUE(measurement.accepted);
  r = tracker.update(sample(7, 1.9, 0.35, 0.08), {measurement});
  EXPECT_EQ(r.state, EvidenceState::kMeasured);
  EXPECT_NEAR(r.estimate.position.z(), 0.24, 1e-6);
  EXPECT_FALSE(r.attachment_valid);
  EXPECT_EQ(r.measurement_sequence, 7U);
}

TEST(ObjectTracker, OpenAfterConflictDoesNotBypassAssociation)
{
  ObjectTracker tracker;
  attach(tracker);
  auto conflict = pose();
  conflict.accepted = false;
  conflict.model_conflict = true;
  tracker.update(sample(4, 1.6, 0.29, 0.04), {conflict});
  const auto opened = sample(5, 1.7, 0.35, 0.08);
  tracker.observeRobot(opened);
  EXPECT_TRUE(tracker.anchorToSupport(opened));
  const auto r = tracker.update(opened, {pose(0.24, 0.9)});
  EXPECT_EQ(r.reason, "ASSOCIATION_JUMP");
  EXPECT_FALSE(r.pose_valid);
  EXPECT_FALSE(r.attachment_valid);
}

TEST(ObjectTracker, EmptyWrongSizeAndAlreadyClosedGripperCannotAttach)
{
  for (double width : {0.0, 0.01, 0.06}) {
    ObjectTracker tracker;
    tracker.update(sample(1, 1), {pose()});
    tracker.update(sample(2, 1.1, 0.24, width), {});
    EXPECT_FALSE(tracker.update(sample(3, 1.5, 0.24, width), {}).attachment_valid);
  }
  ObjectTracker tracker;
  tracker.update(sample(1, 1, 0.24, 0.04), {pose()});
  EXPECT_FALSE(tracker.update(sample(2, 1.4, 0.24, 0.04), {}).attachment_valid);
}

TEST(ObjectTracker, StaleOrDistantTargetAndMovingTcpPreventConfirmation)
{
  ObjectTracker tracker;
  tracker.update(sample(1, 1), {pose()});
  tracker.update(sample(2, 1.1, 0.24, 0.04), {});
  EXPECT_FALSE(tracker.update(sample(3, 1.5, 0.29, 0.04), {}).attachment_valid);
  auto s = sample(4, 1.9, 0.24, 0.04);
  s.world_from_tcp.translation().x() = 0.7;
  EXPECT_FALSE(tracker.update(s, {}).attachment_valid);
  EXPECT_FALSE(tracker.update(sample(5, 2.1, 0.24, 0.04), {}).attachment_valid);
}

TEST(ObjectTracker, InvisibleSlipCannotBeDetectedButAttachmentExpires)
{
  ObjectTracker tracker;
  attach(tracker);
  TrackingResult r;
  for (uint64_t i = 4; i <= 30; ++i) {
    r = tracker.update(sample(i, 1.5 + 0.2 * (i - 3), 0.35, 0.04), {});
    if (i == 8) {
      // Identical inputs for held and invisibly dropped objects are indistinguishable.
      EXPECT_TRUE(r.attachment_valid);
      EXPECT_EQ(r.measurement_sequence, 1U);
    }
  }
  EXPECT_FALSE(r.attachment_valid);
  EXPECT_EQ(r.state, EvidenceState::kOccluded);
  EXPECT_EQ(r.grasp_reason, "ATTACHMENT_EXPIRED");
}

TEST(ObjectTracker, WidthAnomalyInputFailureAndStateGapInvalidateUntilReopened)
{
  for (int mode = 0; mode < 3; ++mode) {
    ObjectTracker tracker;
    attach(tracker);
    auto s = sample(4, mode == 2 ? 2.2 : 1.7, 0.3, mode == 0 ? 0.0 : 0.04);
    if (mode == 1) {
      s.input_failure = "MISSING_ROBOT_TRANSFORM";
    }
    auto r = tracker.update(s, {});
    EXPECT_FALSE(r.attachment_valid);
    EXPECT_EQ(r.grasp_state, GraspState::kInvalidated);
    s.sequence++;
    s.time_s += 0.1;
    s.input_failure.clear();
    s.gripper_width_m = 0.04;
    EXPECT_FALSE(tracker.update(s, {pose(0.3)}).attachment_valid);
  }
}

TEST(ObjectTracker, WidthOscillationAndTcpRotationResetStableWindow)
{
  ObjectTracker tracker;
  tracker.update(sample(1, 1), {pose()});
  tracker.update(sample(2, 1.1, 0.24, 0.035), {});
  auto r = tracker.update(sample(3, 1.4, 0.24, 0.045), {});
  EXPECT_EQ(r.grasp_state, GraspState::kCandidate);
  EXPECT_FALSE(r.attachment_valid);
  auto s = sample(4, 1.7, 0.24, 0.045);
  s.world_from_tcp.linear() = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  r = tracker.update(s, {});
  EXPECT_FALSE(r.attachment_valid);
}

TEST(ObjectTracker, AttachedPoseRotatesWithTcpAndSmallMeasurementCorrectsIt)
{
  ObjectTracker tracker;
  attach(tracker);
  auto s = sample(4, 1.7, 0.32, 0.04);
  s.world_from_tcp.linear() = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto r = tracker.update(s, {});
  ASSERT_TRUE(r.attachment_valid);
  EXPECT_NEAR(Eigen::AngleAxisd(r.estimate.orientation).angle(), 0.5, 1e-12);
  s.sequence = 5;
  s.time_s = 1.8;
  r = tracker.update(s, {pose(0.32, 0.502)});
  EXPECT_TRUE(r.attachment_valid);
  EXPECT_EQ(r.state, EvidenceState::kMeasured);
  s.sequence = 6;
  s.time_s = 1.9;
  r = tracker.update(s, {});
  EXPECT_NEAR(r.estimate.position.x(), 0.502, 1e-12);
}

TEST(ObjectTracker, ResetClearsConfirmedAttachmentAndOldFramesCannotRestoreIt)
{
  ObjectTracker tracker;
  attach(tracker);
  tracker.reset(7, 2);
  EXPECT_FALSE(tracker.update(sample(4, 1.7, 0.32, 0.04), {}).attachment_valid);
  auto s = sample(5, 1.8, 0.32, 0.04);
  s.generation = 2;
  const auto r = tracker.update(s, {});
  EXPECT_EQ(r.grasp_state, GraspState::kUnconfirmed);
  EXPECT_EQ(r.measurement_sequence, 0U);
}

TEST(ObjectTracker, SameWidthWrongObjectCannotBeProvenAbsentWithoutExtraSensing)
{
  ObjectTracker actual_target;
  ObjectTracker same_width_obstacle;
  attach(actual_target);
  attach(same_width_obstacle);
  // If all observable inputs are identical, both cases yield the same conditional estimate.
  const auto a = actual_target.update(sample(4, 1.7, 0.32, 0.04), {});
  const auto b = same_width_obstacle.update(sample(4, 1.7, 0.32, 0.04), {});
  EXPECT_EQ(a.grasp_state, b.grasp_state);
  EXPECT_EQ(a.state, EvidenceState::kPredicted);
  EXPECT_EQ(a.measurement_sequence, b.measurement_sequence);
}

TEST(ObjectTracker, ContinuousRobotFeedbackBridgesSparseImageUpdates)
{
  ObjectTracker tracker;
  tracker.update(sample(1, 1), {pose()});
  for (uint64_t i = 2; i <= 51; ++i) {
    tracker.observeRobot(sample(i, 1.0 + 0.01 * (i - 1), 0.24, 0.04));
  }
  const auto r = tracker.update(sample(52, 1.51, 0.3, 0.04), {});
  EXPECT_TRUE(r.attachment_valid);
  EXPECT_EQ(r.state, EvidenceState::kPredicted);
  EXPECT_EQ(r.measurement_sequence, 1U);
  EXPECT_NEAR(r.estimate.position.z(), 0.3, 1e-12);
}

TEST(ObjectTracker, FreshMeasurementsCannotRenewAttachmentLifetime)
{
  ObjectTracker tracker;
  attach(tracker);
  TrackingResult r;
  for (uint64_t i = 4; i <= 30; ++i) {
    r = tracker.update(sample(i, 1.5 + 0.2 * (i - 3), 0.32, 0.04), {pose(0.32)});
  }
  EXPECT_EQ(r.state, EvidenceState::kMeasured);
  EXPECT_FALSE(r.attachment_valid);
  EXPECT_EQ(r.grasp_reason, "ATTACHMENT_EXPIRED");
}

TEST(ObjectTracker, CandidateCannotConfirmBeyondHeldPredictionBudget)
{
  TrackerConfig config;
  config.held_prediction_age_s = 0.2;
  ObjectTracker tracker(config);
  tracker.update(sample(1, 1), {pose()});
  tracker.update(sample(2, 1.1, 0.24, 0.04), {});
  const auto r = tracker.update(sample(3, 1.5, 0.24, 0.04), {});
  EXPECT_FALSE(r.attachment_valid);
  EXPECT_EQ(r.grasp_reason, "ATTACHMENT_EXPIRED");
}

TEST(ObjectTracker, MissingTfAndInvalidInputAreRejectedRatherThanPredicted)
{
  ObjectTracker tracker;
  tracker.update(sample(1, 1), {pose()});
  auto s = sample(2, 1.1);
  s.input_failure = "MISSING_ROBOT_TRANSFORM";
  EXPECT_EQ(tracker.update(s, {}).state, EvidenceState::kRejected);
  s = sample(3, 1.2);
  s.gripper_width_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(tracker.update(s, {}).reason, "INVALID_INPUT");
}

TEST(ObjectTracker, OversizedGeometryCannotBeHiddenByPrediction)
{
  ObjectTracker tracker;
  tracker.update(sample(1, 1), {pose()});
  auto p = pose();
  p.accepted = false;
  p.model_conflict = true;
  p.rejection = RejectionReason::kModelExtentMismatch;
  const auto r = tracker.update(sample(2, 1.1), {p});
  EXPECT_EQ(r.state, EvidenceState::kRejected);
  EXPECT_EQ(r.reason, "MODEL_INCONSISTENT");
  EXPECT_FALSE(r.pose_valid);
}

TEST(ObjectTracker, ResetAndRestartClearHistoryAndRejectOldFrames)
{
  ObjectTracker tracker;
  tracker.update(sample(1, 1), {pose()});
  tracker.reset(7, 2);
  EXPECT_EQ(tracker.update(sample(2, 1.1), {pose()}).reason, "LIFECYCLE_MISMATCH");
  auto s = sample(3, 1.2);
  s.generation = 2;
  EXPECT_EQ(tracker.update(s, {}).state, EvidenceState::kOccluded);
  tracker.reset(8, 0);
  s.session = 8;
  s.generation = 0;
  EXPECT_EQ(tracker.update(s, {}).state, EvidenceState::kOccluded);
}

TEST(ObjectTracker, OldSequenceCannotRefreshMeasurementOrAge)
{
  ObjectTracker tracker;
  tracker.update(sample(5, 1), {pose()});
  EXPECT_EQ(tracker.update(sample(4, 1.1), {pose()}).reason, "OUT_OF_ORDER");
  EXPECT_EQ(tracker.update(sample(6, 0.9), {pose()}).reason, "OUT_OF_ORDER");
  const auto r = tracker.update(sample(7, 1.2), {});
  EXPECT_EQ(r.measurement_sequence, 5U);
  EXPECT_NEAR(r.prediction_age_s, 0.2, 1e-12);
}

TEST(ObjectTracker, LimitsRejectBadConfigurationAndBoundAssociationAfterLongGap)
{
  TrackerConfig config;
  config.max_prediction_age_s = -1;
  EXPECT_THROW(ObjectTracker tracker(config), std::invalid_argument);
  ObjectTracker tracker;
  tracker.update(sample(1, 1), {pose()});
  EXPECT_EQ(tracker.update(sample(2, 10), {pose(0.24, 0.9)}).reason, "ASSOCIATION_JUMP");
}

TEST(ObjectTracker, ActualPointCloudPartialSurfaceDoesNotBecomeAnElevatedMeasurement)
{
  pcl::PointCloud<pcl::PointXYZ> face;
  for (int x = 0; x <= 8; ++x) {
    for (int y = 0; y <= 8; ++y) {
      face.emplace_back(0.48F + 0.005F * x, -0.02F + 0.005F * y, 0.26F);
    }
  }
  ObjectTracker tracker;
  BoxModel model;
  auto measurement = estimateBoxPose(face, model, 0.22);
  ASSERT_TRUE(measurement.accepted);
  ASSERT_EQ(tracker.update(sample(1, 1), {measurement}).state, EvidenceState::kMeasured);
  pcl::PointCloud<pcl::PointXYZ> partial;
  for (const auto & point : face) {
    if (point.x > 0.49F) {
      partial.push_back(point);
    }
  }
  measurement = estimateBoxPose(partial, model, 0.22);
  ASSERT_TRUE(measurement.accepted);
  EXPECT_EQ(tracker.update(sample(2, 1.1), {measurement}).state, EvidenceState::kMeasured);
  for (auto & point : face) {
    point.z += 0.08F;
  }
  model.anchor_z_to_plane = false;
  measurement = estimateBoxPose(face, model, 0.22);
  ASSERT_FALSE(measurement.accepted);
  const auto r = tracker.update(sample(3, 1.2, 0.32, 0.04), {measurement});
  EXPECT_EQ(r.state, EvidenceState::kPredicted);
  EXPECT_EQ(r.measurement_sequence, 2U);
  EXPECT_TRUE(std::isnan(r.estimate.residual_m));
  EXPECT_TRUE(r.estimate.orientation_ambiguous);
}

}  // namespace
}  // namespace mujoco_perception
