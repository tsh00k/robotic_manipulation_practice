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

#include "mujoco_bridge/grasp_criteria.hpp"

#include <cmath>

namespace mujoco_bridge
{

bool confirmsAttachment(const GraspSignals & s, const GraspCriteria & c)
{
  return std::isfinite(s.gripper_width_m) && std::isfinite(s.box_to_tcp_horizontal_m) &&
         std::abs(s.gripper_width_m - c.box_width_m) < c.width_epsilon_m &&
         s.left_finger_contact && s.right_finger_contact &&
         s.box_to_tcp_horizontal_m >= 0.0 && s.box_to_tcp_horizontal_m < c.region_radius_m;
}

const char * AttachmentConfirmer::missing(
  const AttachmentSignals & s, const AttachmentParams & p)
{
  if (!s.closing_commanded) {return "NO_CLOSE_COMMAND";}
  if (!std::isfinite(s.gripper_width_m) ||
    std::abs(s.gripper_width_m - p.box_width_m) >= p.width_epsilon_m)
  {
    return "WIDTH_OUTSIDE_BOX";
  }
  if (!std::isfinite(s.finger_speed_m_s) || s.finger_speed_m_s >= p.max_finger_speed_m_s) {
    return "FINGERS_STILL_MOVING";
  }
  return "";
}

bool AttachmentConfirmer::update(const AttachmentSignals & signals, double sim_time_s)
{
  if (missing(signals, params_)[0] != '\0') {
    holding_since_valid_ = false;
    return false;
  }
  if (!holding_since_valid_) {
    holding_since_valid_ = true;
    holding_since_s_ = sim_time_s;
  }
  // 1 ns of slack: stamps are whole nanoseconds turned into seconds.
  return sim_time_s - holding_since_s_ >= params_.hold_s - 1e-9;
}

GraspOutcome classifyGrasp(const GraspSignals & s, const GraspCriteria & c)
{
  const bool width_brackets_box = std::abs(s.gripper_width_m - c.box_width_m) < c.width_epsilon_m;
  const bool both_fingers_touch = s.left_finger_contact && s.right_finger_contact;
  const bool any_finger_touches = s.left_finger_contact || s.right_finger_contact;
  const bool lifted = s.box_height_m > c.lift_height_threshold_m;
  const bool near_tcp = s.box_to_tcp_horizontal_m < c.region_radius_m;

  if (width_brackets_box && both_fingers_touch && lifted && near_tcp) {
    return GraspOutcome::kSuccess;
  }
  if (!width_brackets_box && !any_finger_touches) {
    // Closed on nothing (or not yet closed): "was there ever an object near the
    // gripper" is the only thing that tells this apart from kGraspEmpty.
    return near_tcp ? GraspOutcome::kGraspEmpty : GraspOutcome::kNoObject;
  }
  if (any_finger_touches != both_fingers_touch || (width_brackets_box != both_fingers_touch)) {
    // Touching on one side only, or the width and contact signals disagree with each
    // other (e.g. width reads "holding the box" but nothing is actually touching, or
    // vice versa) -- this is the model contract being violated, not a clean
    // success/miss. Most likely misalignment or contact with something unexpected.
    return GraspOutcome::kUnexpectedContact;
  }
  // width_brackets_box && both_fingers_touch, but lifted/near_tcp failed: the grip
  // itself looks right, the object's position does not. See the docstring in
  // grasp_criteria.hpp for why this function cannot tell "slipped" from "never
  // lifted" apart.
  return GraspOutcome::kSlip;
}

}  // namespace mujoco_bridge
