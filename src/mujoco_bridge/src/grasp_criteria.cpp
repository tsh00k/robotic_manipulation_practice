#include "mujoco_bridge/grasp_criteria.hpp"

#include <cmath>

namespace mujoco_bridge
{

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
