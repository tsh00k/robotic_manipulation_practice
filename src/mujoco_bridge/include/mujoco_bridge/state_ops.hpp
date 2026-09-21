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
