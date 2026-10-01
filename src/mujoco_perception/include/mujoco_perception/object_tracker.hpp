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

#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "mujoco_perception/geometry_pipeline.hpp"

namespace mujoco_perception
{

enum class EvidenceState : uint8_t {kRejected = 0, kMeasured = 1, kPredicted = 2, kOccluded = 3};
enum class MotionModel {kSupported, kTransport, kReleased};
enum class GraspState : uint8_t {kUnconfirmed = 0, kCandidate = 1, kHeld = 2, kReleased = 3,
  kInvalidated = 4};

struct TrackerConfig
{
  double association_slack_m = 0.025;
  double max_speed_m_s = 0.6;
  double min_confidence = 0.20;
  double max_prediction_age_s = 0.3;
  double released_prediction_age_s = 0.15;
  double initial_uncertainty_m = 0.005;
  double uncertainty_growth_m_s = 0.08;
  double max_uncertainty_m = 0.03;
  double supported_z_m = 0.24;
  double support_tolerance_m = 0.012;
  double closed_width_m = 0.055;
  double tcp_distance_m = 0.065;
  double object_width_m = 0.04;
  double width_tolerance_m = 0.008;
  double width_stability_m = 0.002;
  double grasp_stable_s = 0.3;
  double grasp_measurement_max_age_s = 1.0;
  double grasp_tcp_motion_m = 0.012;
  double grasp_tcp_angle_rad = 0.1;
  double held_prediction_age_s = 5.0;
  double held_uncertainty_growth_m_s = 0.003;
  double slip_tolerance_m = 0.025;
  double max_sample_gap_s = 0.5;
};

// Deliberately excludes object truth, contacts and task phase.
struct TrackingSample
{
  uint64_t session = 0;
  uint64_t generation = 0;
  uint64_t sequence = 0;
  double time_s = 0.0;
  Eigen::Isometry3d world_from_tcp = Eigen::Isometry3d::Identity();
  double gripper_width_m = 0.08;
  std::string input_failure;
};

struct TrackingResult
{
  TrackingResult()
  {
    estimate.residual_m = std::numeric_limits<double>::quiet_NaN();
    estimate.inlier_ratio = std::numeric_limits<double>::quiet_NaN();
  }
  EvidenceState state = EvidenceState::kRejected;
  bool pose_valid = false;
  PoseEstimate estimate;
  uint64_t measurement_sequence = 0;
  double measurement_time_s = 0.0;
  double prediction_age_s = 0.0;
  double uncertainty_m = std::numeric_limits<double>::infinity();
  std::string reason;
  int candidate_index = -1;
  GraspState grasp_state = GraspState::kUnconfirmed;
  bool attachment_valid = false;
  double grasp_evidence_time_s = 0.0;
  double attachment_time_s = 0.0;
  uint64_t attachment_measurement_sequence = 0;
  std::string grasp_reason = "NO_CLOSING_EVIDENCE";
  std::string attachment_basis;
};

class ObjectTracker
{
public:
  explicit ObjectTracker(const TrackerConfig & config = {});
  void reset(uint64_t session, uint64_t generation);
  void observeRobot(const TrackingSample & sample);
  bool anchorToSupport(const TrackingSample & sample) const;
  TrackingResult update(
    const TrackingSample & sample, const std::vector<PoseEstimate> & candidates);

private:
  void invalidateAttachment(const std::string & reason);
  void updateGrasp(const TrackingSample & sample, double previous_time_s);
  void fillGrasp(TrackingResult & result) const;
  TrackerConfig config_;
  bool lifecycle_known_ = false;
  bool history_ = false;
  uint64_t session_ = 0;
  uint64_t generation_ = 0;
  uint64_t sequence_ = 0;
  double time_s_ = -1.0;
  TrackingSample measured_sample_;
  PoseEstimate measured_pose_;
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  Eigen::Isometry3d tcp_from_object_ = Eigen::Isometry3d::Identity();
  MotionModel model_ = MotionModel::kSupported;
  GraspState grasp_state_ = GraspState::kUnconfirmed;
  bool seen_open_ = false;
  double candidate_since_s_ = 0.0;
  double candidate_width_m_ = 0.0;
  Eigen::Isometry3d candidate_tcp_ = Eigen::Isometry3d::Identity();
  double grasp_evidence_time_s_ = 0.0;
  double attachment_time_s_ = 0.0;
  uint64_t attachment_measurement_sequence_ = 0;
  std::string grasp_reason_ = "NO_CLOSING_EVIDENCE";
  std::string attachment_basis_;
  Eigen::Vector3d released_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond released_orientation_ = Eigen::Quaterniond::Identity();
  double release_time_s_ = 0.0;
  double release_uncertainty_m_ = 0.0;
  double robot_time_s_ = -1.0;
  uint64_t robot_sequence_ = 0;
};

}  // namespace mujoco_perception
