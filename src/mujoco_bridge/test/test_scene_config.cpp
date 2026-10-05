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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "scene_config.hpp"

namespace mujoco_bridge
{
namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// The numbers of pick_place_scene.xml, written out here so this file stays a pure
// geometry test; test_scene_ops.cpp checks they are what the model actually contains.
SceneGeometry modelGeometry()
{
  SceneGeometry geometry;
  geometry.box_half_extents = Eigen::Vector3d(0.02, 0.02, 0.02);
  geometry.bin_min = Eigen::Vector3d(-0.076, -0.071, -0.006);
  geometry.bin_max = Eigen::Vector3d(0.076, 0.071, 0.012);
  geometry.table_top_z = 0.22;
  geometry.table_min_xy = Eigen::Vector2d(0.2, -0.4);
  geometry.table_max_xy = Eigen::Vector2d(0.8, 0.4);
  return geometry;
}

// Legacy layout: box at the MJCF position, bin at its default pose, both upright and
// at the automatic support height.
SceneRequest legacyRequest()
{
  SceneRequest request;
  request.box.x = kDefaultBoxX;
  request.box.y = kDefaultBoxY;
  request.box.z = 0.241;
  request.bin.x = kDefaultBinX;
  request.bin.y = kDefaultBinY;
  request.bin.z = 0.227;
  return request;
}

// Re-derives z for the pose's current orientation, as the node does when z is omitted.
void useAutoZ(PoseRequest & pose, const Eigen::Vector3d & lo, const Eigen::Vector3d & hi)
{
  pose.z = autoSupportZ(
    rotationFromRpy(pose.roll, pose.pitch, pose.yaw), lo, hi, modelGeometry().table_top_z);
}

void useAutoBoxZ(PoseRequest & pose)
{
  useAutoZ(pose, -modelGeometry().box_half_extents, modelGeometry().box_half_extents);
}

void useAutoBinZ(PoseRequest & pose)
{
  useAutoZ(pose, modelGeometry().bin_min, modelGeometry().bin_max);
}

void expectRejected(const SceneRequest & request, const std::string & fragment)
{
  try {
    resolveScene(request, modelGeometry());
    FAIL() << "expected std::invalid_argument containing `" << fragment << "`";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find(fragment), std::string::npos)
      << "message was: " << e.what();
  }
}

}  // namespace

TEST(RotationConvention, IsRzRyRx)
{
  // yaw alone turns +x into +y.
  const Eigen::Vector3d x_after_yaw =
    rotationFromRpy(0.0, 0.0, M_PI / 2) * Eigen::Vector3d::UnitX();
  EXPECT_NEAR((x_after_yaw - Eigen::Vector3d::UnitY()).norm(), 0.0, 1e-12);

  // Rx is applied first: roll 90 deg turns +y into +z, and the later yaw about z leaves
  // it there. With the opposite order +y would end up along -x instead.
  const Eigen::Vector3d y_after =
    rotationFromRpy(M_PI / 2, 0.0, M_PI / 2) * Eigen::Vector3d::UnitY();
  EXPECT_NEAR((y_after - Eigen::Vector3d::UnitZ()).norm(), 0.0, 1e-12);
}

TEST(AutoSupportZ, UprightBoxAndBinMatchTheMjcfDefaults)
{
  const SceneGeometry g = modelGeometry();
  // Box centre: table 0.22 + half size 0.02 + 1 mm gap = the MJCF's 0.241.
  EXPECT_NEAR(
    autoSupportZ(Eigen::Matrix3d::Identity(), -g.box_half_extents, g.box_half_extents, 0.22),
    0.241, 1e-12);
  // Bin origin is the inner floor surface; the floor is 6 mm thick below it:
  // 0.22 + 0.006 + 0.001 = the MJCF's 0.227.
  EXPECT_NEAR(autoSupportZ(Eigen::Matrix3d::Identity(), g.bin_min, g.bin_max, 0.22), 0.227, 1e-12);
}

TEST(AutoSupportZ, RolledBoxRestsItsLowestCornerOneMillimetreAboveTheTable)
{
  // Roll only: a corner's z is y*sin(roll) + z*cos(roll), lowest at
  // -0.02 * (sin(roll) + cos(roll)) for roll in (0, pi/2). Worked out by hand, not by
  // the code under test.
  const double roll = 0.5;
  const double expected = 0.22 + 0.02 * (std::sin(roll) + std::cos(roll)) + 0.001;
  EXPECT_NEAR(expected, 0.2481401620, 1e-9);

  const SceneGeometry g = modelGeometry();
  EXPECT_NEAR(
    autoSupportZ(rotationFromRpy(roll, 0.0, 0.0), -g.box_half_extents, g.box_half_extents, 0.22),
    expected, 1e-12);
}

TEST(AutoSupportZ, YawDoesNotChangeTheSupportHeight)
{
  const SceneGeometry g = modelGeometry();
  EXPECT_NEAR(
    autoSupportZ(rotationFromRpy(0.0, 0.0, 0.7), -g.box_half_extents, g.box_half_extents, 0.22),
    0.241, 1e-12);
}

TEST(ResolveScene, LegacyLayoutIsAccepted)
{
  const ScenePoses poses = resolveScene(legacyRequest(), modelGeometry());
  EXPECT_NEAR((poses.box.translation() - Eigen::Vector3d(0.5, 0.0, 0.241)).norm(), 0.0, 1e-12);
  EXPECT_NEAR((poses.bin.translation() - Eigen::Vector3d(0.5, 0.3, 0.227)).norm(), 0.0, 1e-12);
  EXPECT_TRUE(poses.box.linear().isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(poses.bin.linear().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(ResolveScene, ReturnsTheRequestedRotation)
{
  SceneRequest request = legacyRequest();
  request.box.roll = 0.3;
  request.box.pitch = 0.2;
  request.box.yaw = 0.7;
  useAutoBoxZ(request.box);
  const ScenePoses poses = resolveScene(request, modelGeometry());
  EXPECT_TRUE(poses.box.linear().isApprox(rotationFromRpy(0.3, 0.2, 0.7), 1e-12));
}

TEST(ResolveScene, RejectsNonFiniteValuesNamingTheParameter)
{
  {
    SceneRequest request = legacyRequest();
    request.box.x = kNaN;
    expectRejected(request, "scene.box.x");
  }
  {
    SceneRequest request = legacyRequest();
    request.bin.y = kInf;
    expectRejected(request, "scene.bin.y");
  }
  {
    // A non-finite angle must be reported as the angle, not as the z derived from it.
    SceneRequest request = legacyRequest();
    request.box.roll = kNaN;
    request.box.z = kNaN;
    expectRejected(request, "scene.box.roll");
  }
  {
    SceneRequest request = legacyRequest();
    request.bin.yaw = kInf;
    expectRejected(request, "scene.bin.yaw");
  }
  {
    SceneRequest request = legacyRequest();
    request.box.z = kNaN;
    expectRejected(request, "scene.box.z");
  }
}

TEST(ResolveScene, RejectsAnExplicitZThatPenetratesTheTable)
{
  SceneRequest request = legacyRequest();
  request.box.z = 0.2398;  // lowest corner 0.2198, 0.2 mm below the table top
  expectRejected(request, "scene.box.z");

  request = legacyRequest();
  request.bin.z = 0.21;
  expectRejected(request, "scene.bin.z");
}

TEST(ResolveScene, AcceptsAnExplicitZWithinNumericalTolerance)
{
  SceneRequest request = legacyRequest();
  request.box.z = 0.23995;  // lowest corner 0.05 mm below the table top, tolerance 0.1 mm
  EXPECT_NO_THROW(resolveScene(request, modelGeometry()));
}

TEST(ResolveScene, AcceptsAnExplicitZAboveTheTable)
{
  // Dropped onto the table at start-up: allowed, it settles.
  SceneRequest request = legacyRequest();
  request.box.z = 0.30;
  EXPECT_NO_THROW(resolveScene(request, modelGeometry()));
}

TEST(ResolveScene, RejectsPositionsOutsideTheTable)
{
  SceneRequest request = legacyRequest();
  request.box.x = 0.79;  // x extent reaches 0.81 > 0.8
  expectRejected(request, "scene.box");

  request = legacyRequest();
  request.bin.y = 0.35;  // y extent reaches 0.421 > 0.4
  expectRejected(request, "scene.bin");

  request = legacyRequest();
  request.box.x = 0.77;
  EXPECT_NO_THROW(resolveScene(request, modelGeometry()));
}

TEST(ResolveScene, TableCheckUsesTheRotatedFootprint)
{
  // Yawed 45 degrees the bin reaches (0.076 + 0.071) * sqrt(2) / 2 = 0.1039 m from its
  // centre. At y = 0.3 that is 0.4039 > 0.4; unrotated it would only reach 0.371.
  SceneRequest request = legacyRequest();
  request.bin.yaw = M_PI / 4;
  expectRejected(request, "scene.bin");

  request.bin.y = 0.29;  // 0.3939 < 0.4
  EXPECT_NO_THROW(resolveScene(request, modelGeometry()));
}

TEST(ResolveScene, RejectsABinTiltedBeyondTwentyDegrees)
{
  // up-axis z component of Rz*Ry(p)*Rx(r) is cos(p) * cos(r).
  SceneRequest request = legacyRequest();
  request.bin.roll = 0.25;
  request.bin.pitch = 0.25;  // 0.9689^2 = 0.9388 < 0.94
  useAutoBinZ(request.bin);
  expectRejected(request, "tilt");

  request.bin.roll = 0.2;
  request.bin.pitch = 0.2;  // 0.9801^2 = 0.9605
  useAutoBinZ(request.bin);
  EXPECT_NO_THROW(resolveScene(request, modelGeometry()));
}

TEST(ResolveScene, ABoxMayBeTiltedArbitrarily)
{
  // A tilted box falls over and settles; only the bin has a tilt limit.
  SceneRequest request = legacyRequest();
  request.box.roll = 0.8;
  request.box.pitch = 0.6;
  useAutoBoxZ(request.box);
  EXPECT_NO_THROW(resolveScene(request, modelGeometry()));
}

TEST(ResolveScene, RejectsABoxOnOrNextToTheBin)
{
  SceneRequest request = legacyRequest();
  request.box.x = 0.5;
  request.box.y = 0.3;  // right on top of the bin
  expectRejected(request, "scene.box");

  // Bin footprint spans y in [0.229, 0.371], keep-out 0.02 m wider: [0.209, 0.391]. A
  // box (half size 0.02) with its far edge at 0.21 is inside it, at 0.208 outside.
  request = legacyRequest();
  request.box.y = 0.19;
  expectRejected(request, "scene.box");

  request.box.y = 0.188;
  EXPECT_NO_THROW(resolveScene(request, modelGeometry()));
}

TEST(ResolveScene, OverlapCheckUsesTheRotatedBoxFootprint)
{
  // Yawed 45 degrees the box reaches 0.02 * sqrt(2) = 0.0283 m in y: 0.18 + 0.0283 =
  // 0.2083 clears the keep-out edge at 0.209, 0.185 + 0.0283 = 0.2133 does not.
  SceneRequest request = legacyRequest();
  request.box.yaw = M_PI / 4;
  request.box.y = 0.18;
  EXPECT_NO_THROW(resolveScene(request, modelGeometry()));

  request.box.y = 0.185;
  expectRejected(request, "scene.box");
}

TEST(ResolveScene, OverlapCheckUsesTheBinFrame)
{
  // Yawed 90 degrees the bin is 0.071 m, not 0.076 m, from its centre along world x,
  // so the keep-out edge moves from x = 0.596 to x = 0.591. A box centred at 0.613 has
  // its near edge at 0.593: outside the rotated keep-out, inside the unrotated one.
  SceneRequest request = legacyRequest();
  request.bin.yaw = M_PI / 2;
  request.box.x = 0.613;
  request.box.y = 0.3;
  EXPECT_NO_THROW(resolveScene(request, modelGeometry()));

  request.box.x = 0.609;  // near edge 0.589 < 0.591
  expectRejected(request, "scene.box");
}

TEST(RejectPoseOverrides, DisabledSceneRejectsPoseParameters)
{
  EXPECT_THROW(
    rejectPoseOverridesWhenDisabled(false, {"use_sim_time", "scene.box.x"}),
    std::invalid_argument);
  EXPECT_THROW(rejectPoseOverridesWhenDisabled(false, {"scene.bin.yaw"}), std::invalid_argument);
  try {
    rejectPoseOverridesWhenDisabled(false, {"scene.bin.yaw"});
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("scene.bin.yaw"), std::string::npos) << e.what();
  }
}

TEST(RejectPoseOverrides, OtherParametersAndEnabledSceneAreFine)
{
  EXPECT_NO_THROW(rejectPoseOverridesWhenDisabled(false, {}));
  EXPECT_NO_THROW(
    rejectPoseOverridesWhenDisabled(false, {"scene.enabled", "use_sim_time", "model_path"}));
  EXPECT_NO_THROW(
    rejectPoseOverridesWhenDisabled(true, {"scene.enabled", "scene.box.x", "scene.bin.yaw"}));
  // The prefix includes the dot: a parameter that merely starts with the same letters
  // is not a pose parameter.
  EXPECT_NO_THROW(rejectPoseOverridesWhenDisabled(false, {"scene.boxer"}));
}

}  // namespace mujoco_bridge
