#include "mujoco_bridge/frame_math.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace
{

// Quaternions are a double cover of rotations (q and -q represent the same
// rotation), so component-wise comparison is wrong in general. |q1 . q2| ~= 1
// is the equivalence test that survives the sign ambiguity.
void expectQuatEquiv(const double * q1, const double * q2, double tol = 1e-9)
{
  double dot = 0.0;
  for (int i = 0; i < 4; ++i) {
    dot += q1[i] * q2[i];
  }
  EXPECT_NEAR(std::abs(dot), 1.0, tol);
}

}  // namespace

// Parent at origin, identity orientation; child translated 1m along parent's x and
// rotated 90deg about z. Since parent is identity, the relative pose is just the
// child's own pose restated -- this pins the "compose with an identity parent is a
// no-op on orientation, subtracts on position" case without needing to hand-derive
// a non-trivial rotation.
TEST(FrameMath, IdentityParentReturnsChildPoseVerbatim)
{
  auto & api = mujoco_bridge::loadMujocoApi();
  mujoco_bridge::Pose parent{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};
  const double half = M_PI / 4.0;  // 90 degree rotation about z: (cos45, 0,0,sin45)
  mujoco_bridge::Pose child{{1.0, 0.0, 0.0}, {std::cos(half), 0.0, 0.0, std::sin(half)}};

  const mujoco_bridge::Pose rel = mujoco_bridge::relativePose(api, child, parent);

  EXPECT_NEAR(rel.pos[0], 1.0, 1e-9);
  EXPECT_NEAR(rel.pos[1], 0.0, 1e-9);
  EXPECT_NEAR(rel.pos[2], 0.0, 1e-9);
  expectQuatEquiv(rel.quat, child.quat);
}

// Parent translated 1m along x with a 90deg yaw about z. Child sits 1m away from
// parent along world +y, with no additional rotation. The parent's local x-axis
// now points along world +y (that's what a +90deg yaw does), so the world-frame
// delta (0, 1, 0) is purely along the parent's local x-axis: expected relative
// position is (1, 0, 0). The relative orientation is the inverse of the parent's
// rotation (child is world-identity, parent contributes -90deg once inverted).
TEST(FrameMath, RotatedParentRotatesRelativePosition)
{
  auto & api = mujoco_bridge::loadMujocoApi();
  const double half = M_PI / 4.0;
  mujoco_bridge::Pose parent{{1.0, 0.0, 0.0}, {std::cos(half), 0.0, 0.0, std::sin(half)}};
  mujoco_bridge::Pose child{{1.0, 1.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};

  const mujoco_bridge::Pose rel = mujoco_bridge::relativePose(api, child, parent);

  EXPECT_NEAR(rel.pos[0], 1.0, 1e-9);
  EXPECT_NEAR(rel.pos[1], 0.0, 1e-9);
  EXPECT_NEAR(rel.pos[2], 0.0, 1e-9);
  const double expected_quat[4] = {std::cos(half), 0.0, 0.0, -std::sin(half)};
  expectQuatEquiv(rel.quat, expected_quat);
}

// Both previous cases have one side (parent or child) at identity, and quaternion
// multiplication commutes with the identity -- so those two alone would not catch
// mulQuat's arguments being swapped (verified: swapping them left both tests
// green). This case has parent and child rotated about *different* axes, where
// q_parent_inv * q_child != q_child * q_parent_inv, closing that gap. Positions are
// identical (delta = 0) to isolate the quaternion math from the translation math.
TEST(FrameMath, NonCommutingRotationsCatchArgumentOrderSwap)
{
  auto & api = mujoco_bridge::loadMujocoApi();
  const double half = M_PI / 4.0;
  // parent: +90deg about z, at the same position as child so pos delta is zero
  // and this test isolates the quaternion math.
  mujoco_bridge::Pose parent{{0.0, 0.0, 0.0}, {std::cos(half), 0.0, 0.0, std::sin(half)}};
  // child: +90deg about x
  mujoco_bridge::Pose child{{0.0, 0.0, 0.0}, {std::cos(half), std::sin(half), 0.0, 0.0}};

  const mujoco_bridge::Pose rel = mujoco_bridge::relativePose(api, child, parent);

  // Hand-computed q_parent_inv * q_child via the Hamilton product formula, with
  // q_parent_inv = (cos45, 0, 0, -sin45).
  const double expected_quat[4] = {0.5, 0.5, -0.5, -0.5};
  expectQuatEquiv(rel.quat, expected_quat);
}
