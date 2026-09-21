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

#include "mujoco_bridge/frame_math.hpp"

namespace mujoco_bridge
{

Pose relativePose(const MujocoApi & api, const Pose & child, const Pose & parent)
{
  Pose result;

  mjtNum parent_quat_inv[4];
  api.negQuat(parent_quat_inv, parent.quat);

  const mjtNum delta[3] = {
    child.pos[0] - parent.pos[0],
    child.pos[1] - parent.pos[1],
    child.pos[2] - parent.pos[2],
  };
  api.rotVecQuat(result.pos, delta, parent_quat_inv);
  api.mulQuat(result.quat, parent_quat_inv, child.quat);

  return result;
}

}  // namespace mujoco_bridge
