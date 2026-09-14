#include "mujoco_bridge/state_ops.hpp"

namespace mujoco_bridge
{

bool resetToKeyframe(const MujocoApi & api, mjModel * m, mjData * d, int key)
{
  if (key < 0) {
    return false;
  }

  const mjtNum time_before = d->time;
  api.resetDataKeyframe(m, d, key);
  d->time = time_before;
  api.forward(m, d);

  return true;
}

}  // namespace mujoco_bridge
