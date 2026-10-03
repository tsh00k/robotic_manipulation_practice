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
enum class DiagnosticStage : uint8_t {kNone = 0, kInput = 1, kGeometry = 2, kAssociation = 3,
  kPrediction = 4, kGrasp = 5, kLifecycle = 6};
enum class AttachmentState : uint8_t {kNotAttached = 0, kAttached = 1, kReleased = 2};
enum class GraspState : uint8_t {kNotAttached = 0, kHeld = 1, kReleased = 2};

struct TrackerConfig
{
  double min_confidence = 0.20;
};

// Deliberately excludes object truth, contacts and task phase.
struct TrackingSample
{
  uint64_t session = 0;
  uint64_t generation = 0;
  uint64_t sequence = 0;
  double time_s = 0.0;
  AttachmentState attachment_state = AttachmentState::kNotAttached;
  bool support_prior_used = false;
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
  PoseEstimate estimate;
  uint64_t measurement_sequence = 0;
  std::string reason;
  DiagnosticStage diagnostic_stage = DiagnosticStage::kNone;
  int candidate_index = -1;
  std::size_t candidate_count = 0;
  std::size_t eligible_candidate_count = 0;
  bool support_prior_used = false;
  GraspState grasp_state = GraspState::kNotAttached;
  bool attachment_valid = false;
};

class ObjectTracker
{
public:
  explicit ObjectTracker(const TrackerConfig & config = {});
  void reset(uint64_t session, uint64_t generation);
  bool anchorToSupport(const TrackingSample & sample) const;
  TrackingResult update(
    const TrackingSample & sample, const std::vector<PoseEstimate> & candidates);

private:
  void fillAttachment(const TrackingSample & sample, TrackingResult & result) const;
  TrackerConfig config_;
  bool lifecycle_known_ = false;
  bool history_ = false;
  uint64_t session_ = 0;
  uint64_t generation_ = 0;
  uint64_t sequence_ = 0;
  double time_s_ = -1.0;
  TrackingSample measured_sample_;
};

}  // namespace mujoco_perception
