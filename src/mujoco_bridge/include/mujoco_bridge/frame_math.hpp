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
