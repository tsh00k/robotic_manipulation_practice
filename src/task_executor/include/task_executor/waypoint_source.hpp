#pragma once

#include <array>

#include "task_executor/phase.hpp"

namespace task_executor
{

// Plain struct, not geometry_msgs::msg::Pose -- same "Layer 1 stays message-free"
// discipline mujoco_bridge's grasp_criteria.hpp uses for GraspSignals (week2.md
// Stage F/H). Orientation is carried for the interface's sake (a future IK-based
// source needs it) but KeyframeWaypointSource below ignores it entirely.
struct ObjectPose
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
};

// What one phase commands: the 7 arm joints (joint1..joint7, mujoco_bridge's own
// order) plus the gripper's total finger-to-finger opening in meters -- the same
// two-part split as mujoco_bridge's ~/joint_command vs ~/gripper_command (Stage H).
struct JointTarget
{
  std::array<double, 7> arm_positions{};
  double gripper_width_m = 0.0;
};

// Swappable target-lookup abstraction (week2.md Stage I). This week's only
// implementation, KeyframeWaypointSource, is a fixed lookup table that ignores
// object_pose entirely -- it exists so the *interface* is already the one week3's
// diff-IK WaypointSource will implement, and the FSM (fsm.hpp/cpp) that calls
// jointTargetFor() never needs to change when that swap happens.
class WaypointSource
{
public:
  virtual ~WaypointSource() = default;

  virtual JointTarget jointTargetFor(Phase phase, const ObjectPose & object_pose) const = 0;
};

}  // namespace task_executor
