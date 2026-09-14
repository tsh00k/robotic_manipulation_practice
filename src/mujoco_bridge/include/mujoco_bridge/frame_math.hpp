#pragma once

#include "mujoco_bridge/mujoco_dl.hpp"

namespace mujoco_bridge
{

// pos = translation in the parent frame; quat = (w, x, y, z), MuJoCo's convention.
struct Pose
{
  double pos[3];
  double quat[4];
};

// Composes T_parent^-1 * T_child: turns two absolute (world-frame) poses into
// child's pose relative to parent. Depends only on MujocoApi's quaternion helpers,
// not on mjModel/mjData or any ROS type, so it can be unit tested without loading
// a model.
Pose relativePose(const MujocoApi & api, const Pose & child, const Pose & parent);

}  // namespace mujoco_bridge
