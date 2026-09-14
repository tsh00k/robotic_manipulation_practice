#pragma once

#include "mujoco_bridge/mujoco_dl.hpp"

namespace mujoco_bridge
{

// Restores qpos/qvel/act/ctrl/mocap from keyframe `key`, keeps mjData::time
// monotonic (mj_resetDataKeyframe rewinds it to the keyframe's own time, which this
// function undoes), and calls mj_forward so every quantity derived from qpos/qvel
// (xpos/xquat, qfrc_actuator, ...) is refreshed before the caller reads mjData again.
//
// Returns false without touching d if key < 0; the caller (which owns the logger)
// decides how to report that.
bool resetToKeyframe(const MujocoApi & api, mjModel * m, mjData * d, int key);

}  // namespace mujoco_bridge
