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

#include "mujoco_perception/object_tracker.hpp"

#include <cmath>
#include <stdexcept>

namespace mujoco_perception
{

namespace
{
DiagnosticStage stageFor(const std::string & reason)
{
  if (reason == "LIFECYCLE_MISMATCH" || reason == "OUT_OF_ORDER") {
    return DiagnosticStage::kLifecycle;
  }
  if (reason == "INVALID_INPUT" || reason == "MISSING_ROBOT_TRANSFORM") {
    return DiagnosticStage::kInput;
  }
  if (reason == "MULTIPLE_CANDIDATES") {
    return DiagnosticStage::kAssociation;
  }
  if (reason == "NO_CANDIDATE" || reason == "CANDIDATE_INVALID") {
    return DiagnosticStage::kGeometry;
  }
  return DiagnosticStage::kNone;
}

bool validCandidate(const PoseEstimate & candidate, double min_confidence)
{
  return candidate.geometry_valid && candidate.point_count >= 3 &&
         candidate.position.allFinite() && candidate.orientation.coeffs().allFinite() &&
         std::abs(candidate.orientation.norm() - 1.0) <= 1e-3 &&
         std::isfinite(candidate.confidence) && candidate.confidence >= min_confidence &&
         std::isfinite(candidate.residual_m) && std::isfinite(candidate.inlier_ratio);
}
}  // namespace

ObjectTracker::ObjectTracker(const TrackerConfig & config)
: config_(config)
{
  if (!std::isfinite(config_.min_confidence) || config_.min_confidence < 0.0 ||
    config_.min_confidence > 1.0)
  {
    throw std::invalid_argument("min_confidence must be finite and in [0, 1]");
  }
}

void ObjectTracker::reset(uint64_t session, uint64_t generation)
{
  lifecycle_known_ = true;
  session_ = session;
  generation_ = generation;
  history_ = false;
  sequence_ = 0;
  time_s_ = -1.0;
  measured_sample_ = TrackingSample{};
}

bool ObjectTracker::anchorToSupport(const TrackingSample & sample) const
{
  // The camera is active only before attachment and after a release. There is no
  // old transport model to carry across either lifecycle edge.
  return sample.attachment_state != AttachmentState::kAttached;
}

void ObjectTracker::fillAttachment(const TrackingSample & sample, TrackingResult & result) const
{
  if (sample.attachment_state == AttachmentState::kAttached) {
    result.grasp_state = GraspState::kHeld;
    result.attachment_valid = true;
  } else if (sample.attachment_state == AttachmentState::kReleased) {
    result.grasp_state = GraspState::kReleased;
  } else {
    result.grasp_state = GraspState::kNotAttached;
  }
}

TrackingResult ObjectTracker::update(
  const TrackingSample & sample, const std::vector<PoseEstimate> & candidates)
{
  TrackingResult result;
  result.candidate_count = candidates.size();
  result.support_prior_used = sample.support_prior_used;
  fillAttachment(sample, result);

  if (!lifecycle_known_) {
    reset(sample.session, sample.generation);
  }
  if (sample.session != session_ || sample.generation != generation_) {
    result.reason = "LIFECYCLE_MISMATCH";
    result.diagnostic_stage = stageFor(result.reason);
    return result;
  }
  if (sample.sequence == 0 || !std::isfinite(sample.time_s) || sample.time_s < 0.0) {
    result.reason = "INVALID_INPUT";
    result.diagnostic_stage = stageFor(result.reason);
    return result;
  }
  if (time_s_ >= 0.0 && (sample.sequence <= sequence_ || sample.time_s <= time_s_)) {
    result.reason = "OUT_OF_ORDER";
    result.diagnostic_stage = stageFor(result.reason);
    return result;
  }
  sequence_ = sample.sequence;
  time_s_ = sample.time_s;
  if (!sample.input_failure.empty()) {
    result.reason = sample.input_failure;
    result.diagnostic_stage = stageFor(result.reason);
    return result;
  }

  if (candidates.empty()) {
    result.state = EvidenceState::kOccluded;
    result.reason = "NO_CANDIDATE";
    result.diagnostic_stage = stageFor(result.reason);
    if (history_) {
      result.measurement_sequence = measured_sample_.sequence;
    }
    return result;
  }

  std::vector<int> valid;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (validCandidate(candidates[i], config_.min_confidence)) {
      valid.push_back(static_cast<int>(i));
    }
  }
  result.eligible_candidate_count = valid.size();
  if (valid.empty()) {
    result.state = EvidenceState::kRejected;
    result.reason = "CANDIDATE_INVALID";
    result.diagnostic_stage = stageFor(result.reason);
    return result;
  }
  if (valid.size() != 1) {
    result.state = EvidenceState::kRejected;
    result.reason = "MULTIPLE_CANDIDATES";
    result.diagnostic_stage = stageFor(result.reason);
    return result;
  }

  result.candidate_index = valid.front();
  result.estimate = candidates[static_cast<std::size_t>(result.candidate_index)];
  result.measurement_sequence = sample.sequence;
  result.state = EvidenceState::kMeasured;
  result.reason.clear();
  result.diagnostic_stage = DiagnosticStage::kNone;
  measured_sample_ = sample;
  history_ = true;
  return result;
}

}  // namespace mujoco_perception
