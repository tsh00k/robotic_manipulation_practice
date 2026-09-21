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

#include "mujoco_bridge/grasp_state.hpp"

namespace mujoco_bridge
{

double gripperWidth(const mjData * d, int finger1_qpos_adr, int finger2_qpos_adr)
{
  return d->qpos[finger1_qpos_adr] + d->qpos[finger2_qpos_adr];
}

bool bodiesInContact(const mjModel * m, const mjData * d, int body_a, int body_b)
{
  for (int i = 0; i < d->ncon; ++i) {
    const mjContact & c = d->contact[i];
    const int b0 = m->geom_bodyid[c.geom[0]];
    const int b1 = m->geom_bodyid[c.geom[1]];
    if ((b0 == body_a && b1 == body_b) || (b0 == body_b && b1 == body_a)) {
      return true;
    }
  }
  return false;
}

}  // namespace mujoco_bridge
