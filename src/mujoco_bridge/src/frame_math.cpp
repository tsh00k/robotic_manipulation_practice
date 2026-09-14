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
