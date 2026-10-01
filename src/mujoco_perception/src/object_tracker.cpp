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

#include "mujoco_perception/object_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mujoco_perception
{

ObjectTracker::ObjectTracker(const TrackerConfig & config)
: config_(config)
{
  for (double value : {config.association_slack_m, config.max_speed_m_s,
      config.max_prediction_age_s, config.released_prediction_age_s,
      config.initial_uncertainty_m, config.uncertainty_growth_m_s,
      config.max_uncertainty_m, config.support_tolerance_m, config.closed_width_m,
      config.tcp_distance_m, config.object_width_m, config.width_tolerance_m,
      config.width_stability_m, config.grasp_stable_s, config.grasp_measurement_max_age_s,
      config.grasp_tcp_motion_m, config.grasp_tcp_angle_rad, config.held_prediction_age_s,
      config.held_uncertainty_growth_m_s, config.slip_tolerance_m, config.max_sample_gap_s})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("Tracker limits must be finite and positive");
    }
  }
  if (!std::isfinite(config.min_confidence) || config.min_confidence < 0.0 ||
    config.min_confidence > 1.0 || !std::isfinite(config.supported_z_m) ||
    config.initial_uncertainty_m >= config.max_uncertainty_m ||
    config.width_tolerance_m >= config.object_width_m ||
    config.object_width_m + config.width_tolerance_m >= config.closed_width_m)
  {
    throw std::invalid_argument("Invalid tracker quality/support configuration");
  }
}

void ObjectTracker::reset(uint64_t session, uint64_t generation)
{
  lifecycle_known_ = true;
  session_ = session;
  generation_ = generation;
  sequence_ = 0;
  time_s_ = -1.0;
  robot_time_s_ = -1.0;
  robot_sequence_ = 0;
  history_ = false;
  velocity_.setZero();
  model_ = MotionModel::kSupported;
  grasp_state_ = GraspState::kUnconfirmed;
  seen_open_ = false;
  grasp_evidence_time_s_ = 0.0;
  attachment_time_s_ = 0.0;
  attachment_measurement_sequence_ = 0;
  attachment_basis_.clear();
  grasp_reason_ = "NO_CLOSING_EVIDENCE";
}

void ObjectTracker::invalidateAttachment(const std::string & reason)
{
  grasp_state_ = GraspState::kInvalidated;
  grasp_reason_ = reason;
  model_ = MotionModel::kReleased;
  seen_open_ = false;
  velocity_.setZero();
}

void ObjectTracker::fillGrasp(TrackingResult & result) const
{
  result.grasp_state = grasp_state_;
  result.attachment_valid = grasp_state_ == GraspState::kHeld;
  result.grasp_reason = grasp_reason_;
  result.grasp_evidence_time_s = grasp_evidence_time_s_;
  result.attachment_time_s = attachment_time_s_;
  result.attachment_measurement_sequence = attachment_measurement_sequence_;
  result.attachment_basis = attachment_basis_;
}

void ObjectTracker::updateGrasp(const TrackingSample & sample, double previous_time_s)
{
  if (previous_time_s >= 0.0 && sample.time_s - previous_time_s > config_.max_sample_gap_s &&
    (grasp_state_ == GraspState::kHeld || grasp_state_ == GraspState::kCandidate))
  {
    invalidateAttachment("ROBOT_STATE_GAP");
    return;
  }
  const bool opening_from_attachment =
    (grasp_state_ == GraspState::kHeld || grasp_state_ == GraspState::kReleased) &&
    sample.gripper_width_m > config_.object_width_m + config_.width_tolerance_m;
  if (sample.gripper_width_m >= config_.closed_width_m || opening_from_attachment) {
    if (grasp_state_ == GraspState::kHeld) {
      const auto released = sample.world_from_tcp * tcp_from_object_;
      released_position_ = released.translation();
      released_orientation_ = Eigen::Quaterniond(released.linear());
      release_time_s_ = sample.time_s;
      release_uncertainty_m_ = config_.initial_uncertainty_m +
        config_.held_uncertainty_growth_m_s * (sample.time_s - measured_sample_.time_s);
      grasp_state_ = GraspState::kReleased;
      grasp_reason_ = "GRIPPER_OPENING_OR_LOSS_OF_WIDTH_SUPPORT";
      model_ = MotionModel::kReleased;
      velocity_.setZero();
    } else if (grasp_state_ != GraspState::kReleased) {
      grasp_state_ = GraspState::kUnconfirmed;
      grasp_reason_ = "GRIPPER_OPEN";
    }
    if (sample.gripper_width_m >= config_.closed_width_m) {
      seen_open_ = true;
    }
    return;
  }
  const bool in_band = std::abs(sample.gripper_width_m - config_.object_width_m) <=
    config_.width_tolerance_m;
  if (grasp_state_ == GraspState::kHeld) {
    if (!in_band) {
      invalidateAttachment("HELD_WIDTH_ANOMALY");
      return;
    }
    if (sample.time_s - attachment_time_s_ > config_.held_prediction_age_s ||
      sample.time_s - measured_sample_.time_s > config_.held_prediction_age_s ||
      config_.initial_uncertainty_m + config_.held_uncertainty_growth_m_s *
      (sample.time_s - measured_sample_.time_s) > config_.max_uncertainty_m)
    {
      invalidateAttachment("ATTACHMENT_EXPIRED");
      return;
    }
    grasp_evidence_time_s_ = sample.time_s;
    return;
  }
  if (grasp_state_ == GraspState::kInvalidated || !seen_open_) {
    return;
  }
  const bool reliable_supported_target = history_ &&
    sample.time_s - measured_sample_.time_s <= config_.grasp_measurement_max_age_s +
    (grasp_state_ == GraspState::kCandidate ? config_.grasp_stable_s : 0.0) &&
    model_ == MotionModel::kSupported &&
    (sample.world_from_tcp.translation() - measured_pose_.position).norm() <=
    config_.tcp_distance_m;
  if (!in_band || !reliable_supported_target) {
    grasp_state_ = GraspState::kUnconfirmed;
    grasp_reason_ = !in_band ? "WIDTH_OUTSIDE_OBJECT_BAND" : "NO_RECENT_SUPPORTED_TARGET";
    return;
  }
  const bool stable = grasp_state_ == GraspState::kCandidate &&
    std::abs(sample.gripper_width_m - candidate_width_m_) <= config_.width_stability_m &&
    (sample.world_from_tcp.translation() - candidate_tcp_.translation()).norm() <=
    config_.grasp_tcp_motion_m &&
    Eigen::AngleAxisd(
    candidate_tcp_.linear().transpose() *
    sample.world_from_tcp.linear()).angle() <=
    config_.grasp_tcp_angle_rad;
  if (!stable) {
    grasp_state_ = GraspState::kCandidate;
    candidate_since_s_ = sample.time_s;
    candidate_width_m_ = sample.gripper_width_m;
    candidate_tcp_ = sample.world_from_tcp;
    grasp_reason_ = "WAITING_FOR_WIDTH_AND_TCP_STABILITY";
    return;
  }
  if (sample.time_s - candidate_since_s_ < config_.grasp_stable_s) {
    return;
  }
  if (sample.time_s - measured_sample_.time_s > config_.held_prediction_age_s ||
    config_.initial_uncertainty_m + config_.held_uncertainty_growth_m_s *
    (sample.time_s - measured_sample_.time_s) > config_.max_uncertainty_m)
  {
    invalidateAttachment("ATTACHMENT_EXPIRED");
    return;
  }
  // A recent supported object is assumed stationary until closure completes.
  // This is an explicit prior, not a new image measurement at confirmation.
  Eigen::Isometry3d world_from_object = Eigen::Isometry3d::Identity();
  world_from_object.translation() = measured_pose_.position;
  world_from_object.linear() = measured_pose_.orientation.toRotationMatrix();
  tcp_from_object_ = sample.world_from_tcp.inverse() * world_from_object;
  grasp_state_ = GraspState::kHeld;
  model_ = MotionModel::kTransport;
  grasp_evidence_time_s_ = sample.time_s;
  attachment_time_s_ = sample.time_s;
  attachment_measurement_sequence_ = measured_sample_.sequence;
  attachment_basis_ = "RECENT_SUPPORTED_TARGET_ASSUMED_STATIONARY_DURING_CLOSURE";
  grasp_reason_ = "STABLE_OBJECT_WIDTH_NEAR_RECENT_TARGET";
  seen_open_ = false;
}

void ObjectTracker::observeRobot(const TrackingSample & sample)
{
  if (!lifecycle_known_) {
    reset(sample.session, sample.generation);
  }
  if (sample.session != session_ || sample.generation != generation_ ||
    sample.sequence <= robot_sequence_ || sample.time_s <= robot_time_s_)
  {
    return;
  }
  if (!sample.input_failure.empty() || !std::isfinite(sample.time_s) ||
    !sample.world_from_tcp.matrix().allFinite() || !std::isfinite(sample.gripper_width_m) ||
    sample.gripper_width_m < 0.0)
  {
    invalidateAttachment("INVALID_ROBOT_STATE");
    return;
  }
  updateGrasp(sample, robot_time_s_);
  robot_time_s_ = sample.time_s;
  robot_sequence_ = sample.sequence;
}

bool ObjectTracker::anchorToSupport(const TrackingSample & sample) const
{
  if (sample.gripper_width_m >= config_.closed_width_m) {
    // Opening permits support reacquisition even after attachment invalidation.
    // The node still checks candidate height before enabling the support prior.
    return true;
  }
  if (!history_) {
    return true;
  }
  const bool near_tcp = (sample.world_from_tcp.translation() -
    measured_pose_.position).norm() < config_.tcp_distance_m;
  return model_ == MotionModel::kSupported &&
         !(sample.gripper_width_m < config_.closed_width_m && near_tcp &&
         sample.world_from_tcp.translation().z() >
         config_.supported_z_m + config_.support_tolerance_m);
}

TrackingResult ObjectTracker::update(
  const TrackingSample & sample, const std::vector<PoseEstimate> & candidates)
{
  TrackingResult out;
  if (!lifecycle_known_) {
    reset(sample.session, sample.generation);
  }
  if (sample.session != session_ || sample.generation != generation_) {
    out.reason = "LIFECYCLE_MISMATCH";
    return out;
  }
  if (!std::isfinite(sample.time_s) || sample.time_s < 0.0 ||
    !sample.world_from_tcp.matrix().allFinite() ||
    !std::isfinite(sample.gripper_width_m) || sample.gripper_width_m < 0.0)
  {
    out.reason = "INVALID_INPUT";
    if (grasp_state_ == GraspState::kHeld || grasp_state_ == GraspState::kCandidate) {
      invalidateAttachment(out.reason);
    }
    fillGrasp(out);
    return out;
  }
  if (time_s_ >= 0.0 && (sample.sequence <= sequence_ || sample.time_s <= time_s_)) {
    out.reason = "OUT_OF_ORDER";
    return out;
  }
  sequence_ = sample.sequence;
  time_s_ = sample.time_s;
  if (history_) {
    out.measurement_sequence = measured_sample_.sequence;
    out.measurement_time_s = measured_sample_.time_s;
    out.prediction_age_s = sample.time_s - measured_sample_.time_s;
    out.uncertainty_m = config_.initial_uncertainty_m +
      config_.uncertainty_growth_m_s * out.prediction_age_s;
  }
  if (!sample.input_failure.empty()) {
    // Broken input is not an occlusion and must not be covered by prediction.
    out.reason = sample.input_failure;
    if (grasp_state_ == GraspState::kHeld || grasp_state_ == GraspState::kCandidate) {
      invalidateAttachment(sample.input_failure);
    }
    fillGrasp(out);
    return out;
  }
  observeRobot(sample);
  fillGrasp(out);
  if (out.attachment_valid) {
    out.uncertainty_m = config_.initial_uncertainty_m +
      config_.held_uncertainty_growth_m_s * out.prediction_age_s;
  } else if (grasp_state_ == GraspState::kReleased) {
    out.uncertainty_m = release_uncertainty_m_ + config_.uncertainty_growth_m_s *
      (sample.time_s - release_time_s_);
  }
  Eigen::Vector3d expected = measured_pose_.position;
  Eigen::Quaterniond expected_orientation = measured_pose_.orientation;
  if (history_ && model_ == MotionModel::kTransport) {
    const Eigen::Isometry3d pose = sample.world_from_tcp * tcp_from_object_;
    expected = pose.translation();
    expected_orientation = Eigen::Quaterniond(pose.linear());
  } else if (history_ && model_ == MotionModel::kReleased) {
    if (grasp_state_ == GraspState::kReleased) {
      expected = released_position_;
      expected_orientation = released_orientation_;
    } else {
      expected += velocity_ * std::min(out.prediction_age_s, config_.released_prediction_age_s);
    }
  }
  const double gate = config_.association_slack_m + config_.max_speed_m_s *
    std::min(out.prediction_age_s, config_.max_prediction_age_s);
  std::vector<int> eligible;
  bool quality_candidate = false;
  bool model_conflict = false;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const auto & c = candidates[i];
    model_conflict = model_conflict || c.model_conflict ||
      c.rejection == RejectionReason::kHighResidual ||
      c.rejection == RejectionReason::kLowInlierRatio ||
      c.rejection == RejectionReason::kInvalidInput;
    if (!c.accepted || c.point_count < 3 || !c.position.allFinite() ||
      !c.orientation.coeffs().allFinite() || std::abs(c.orientation.norm() - 1.0) > 1e-3 ||
      !std::isfinite(c.confidence) || c.confidence < config_.min_confidence ||
      !std::isfinite(c.residual_m) || !std::isfinite(c.inlier_ratio))
    {
      continue;
    }
    quality_candidate = true;
    if (!history_ || ((c.position - expected).norm() <= gate &&
      (out.attachment_valid || grasp_state_ == GraspState::kReleased ||
      (c.position - measured_pose_.position).norm() <= gate)))
    {
      eligible.push_back(static_cast<int>(i));
    }
  }
  if (eligible.empty() && model_conflict) {
    out.reason = "MODEL_INCONSISTENT";
    invalidateAttachment(out.reason);
    fillGrasp(out);
    return out;
  }
  if (eligible.size() > 1 || (eligible.empty() && quality_candidate)) {
    out.reason = eligible.size() > 1 ? "ASSOCIATION_AMBIGUOUS" : "ASSOCIATION_JUMP";
    invalidateAttachment(out.reason);
    fillGrasp(out);
    return out;
  }
  if (eligible.size() == 1) {
    out.candidate_index = eligible.front();
    const auto & pose = candidates[static_cast<std::size_t>(out.candidate_index)];
    const bool supported = std::abs(pose.position.z() - config_.supported_z_m) <=
      config_.support_tolerance_m;
    Eigen::Isometry3d world_from_object = Eigen::Isometry3d::Identity();
    world_from_object.linear() = pose.orientation.toRotationMatrix();
    world_from_object.translation() = pose.position;
    const Eigen::Isometry3d relative = sample.world_from_tcp.inverse() * world_from_object;
    if (out.attachment_valid && (pose.position - expected).norm() > config_.slip_tolerance_m) {
      invalidateAttachment("VISIBLE_ATTACHMENT_DEVIATION");
    }
    if (history_) {
      velocity_ = (pose.position - measured_pose_.position) / out.prediction_age_s;
    }
    model_ = grasp_state_ == GraspState::kHeld ? MotionModel::kTransport :
      (supported ? MotionModel::kSupported : MotionModel::kReleased);
    if (grasp_state_ == GraspState::kHeld) {
      tcp_from_object_ = relative;
    }
    measured_pose_ = pose;
    measured_sample_ = sample;
    if (grasp_state_ == GraspState::kReleased) {
      released_position_ = pose.position;
      released_orientation_ = pose.orientation;
      release_time_s_ = sample.time_s;
      release_uncertainty_m_ = config_.initial_uncertainty_m;
    }
    history_ = true;
    out.state = EvidenceState::kMeasured;
    out.pose_valid = true;
    out.estimate = pose;
    out.measurement_sequence = sample.sequence;
    out.measurement_time_s = sample.time_s;
    out.prediction_age_s = 0.0;
    out.uncertainty_m = config_.initial_uncertainty_m;
    out.reason = "NONE";
    fillGrasp(out);
    return out;
  }

  const double age_limit = out.attachment_valid ? config_.held_prediction_age_s :
    (model_ == MotionModel::kReleased ?
    config_.released_prediction_age_s : config_.max_prediction_age_s);
  out.state = EvidenceState::kOccluded;
  out.reason = history_ ? "PREDICTION_EXPIRED" : "NO_TRACK_HISTORY";
  const double effective_age = grasp_state_ == GraspState::kReleased ?
    sample.time_s - release_time_s_ : out.prediction_age_s;
  if (history_ && grasp_state_ != GraspState::kInvalidated && effective_age <= age_limit &&
    out.uncertainty_m <= config_.max_uncertainty_m)
  {
    out.state = EvidenceState::kPredicted;
    out.pose_valid = true;
    out.estimate.position = expected;
    out.estimate.orientation = expected_orientation;
    out.estimate.orientation_ambiguous = measured_pose_.orientation_ambiguous;
    out.estimate.confidence = measured_pose_.confidence *
      std::max(0.0, 1.0 - effective_age / age_limit);
    out.reason = "NO_CURRENT_MEASUREMENT";
  }
  out.estimate.residual_m = std::numeric_limits<double>::quiet_NaN();
  out.estimate.inlier_ratio = std::numeric_limits<double>::quiet_NaN();
  return out;
}

}  // namespace mujoco_perception
