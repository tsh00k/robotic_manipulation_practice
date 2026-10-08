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

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>

#include "arm_kinematics/forward_kinematics.hpp"
#include "task_executor/diff_ik_waypoint_source.hpp"

namespace task_executor
{
namespace
{

// The legacy scene's place target on the table.
const PlaceTarget kTable{};

DiffIkWaypointSource makeSource()
{
  return DiffIkWaypointSource(
    arm_kinematics::loadFrankaFerModel(KINEMATICS_YAML_PATH, JOINT_LIMITS_YAML_PATH),
    std::make_shared<PickPlaceCartesianWaypointSource>());
}

std::array<double, 7> home()
{
  return {0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, -0.7853};
}

// The arm part of the MJCF keyframe the bridge starts in and resets to.
std::array<double, 7> mjcfResetKeyframeArm()
{
  std::ifstream file(PICK_PLACE_SCENE_MJCF_PATH);
  std::stringstream text;
  text << file.rdbuf();
  const std::string xml = text.str();
  std::smatch match;
  const std::regex key(R"re(<key\s+name="pick_place_home"\s+qpos="([^"]*)")re");
  if (!std::regex_search(xml, match, key)) {
    ADD_FAILURE() << "no pick_place_home keyframe in " << PICK_PLACE_SCENE_MJCF_PATH;
    return {};
  }
  std::istringstream values(match[1].str());
  std::array<double, 7> arm{};
  for (double & q : arm) {
    values >> q;
  }
  return arm;
}

TEST(DiffIkWaypointSource, HomeIsTheBridgeResetKeyframe)
{
  const std::array<double, 7> keyframe = mjcfResetKeyframeArm();
  for (std::size_t i = 0; i < keyframe.size(); ++i) {
    EXPECT_NEAR(kFrankaReadyPose[i], keyframe[i], 1e-4) << "joint" << i + 1;
  }
}

TEST(DiffIkWaypointSource, HomeAndVerifyCommandTheHomeJointsWithoutIk)
{
  auto source = makeSource();
  // A seed far from HOME: an IK solution would depend on it, the joint target must not.
  source.setSeed({0.5, 0.3, -0.2, -1.2, 0.1, 1.9, -0.4});
  const ObjectPose box{0.52, 0.01, 0.241};
  for (const Phase phase : {Phase::kHome, Phase::kVerify}) {
    source.beginEpisode();
    const JointTarget target = source.jointTargetFor(phase, box, kTable);
    EXPECT_EQ(target.arm_positions, kFrankaReadyPose) << phaseName(phase);
    EXPECT_DOUBLE_EQ(target.gripper_width_m, 0.08) << phaseName(phase);
    ASSERT_TRUE(source.diagnostics());
    // The ready pose's TCP, with the tool pointing down.
    EXPECT_NEAR(source.diagnostics()->tcp_target.translation().x(), 0.307, 0.002);
    EXPECT_NEAR(source.diagnostics()->tcp_target.translation().y(), 0.0, 1e-6);
    EXPECT_NEAR(source.diagnostics()->tcp_target.translation().z(), 0.487, 0.002);
    EXPECT_NEAR(source.diagnostics()->tcp_target.linear()(2, 2), -1.0, 1e-6);
  }
}

TEST(DiffIkWaypointSource, RejectsAHomeOutsideTheJointLimits)
{
  std::array<double, 7> home = kFrankaReadyPose;
  home[3] = 0.0;  // joint4's upper limit is -0.0698
  EXPECT_THROW(
    DiffIkWaypointSource(
      arm_kinematics::loadFrankaFerModel(KINEMATICS_YAML_PATH, JOINT_LIMITS_YAML_PATH),
      std::make_shared<PickPlaceCartesianWaypointSource>(), home),
    std::invalid_argument);
}

// Week 5 Stage 3: the grasp descent of HELD-A layout 14 as a TCP straight line.
TEST(DiffIkWaypointSource, CartesianLineKeepsTheTcpOnTheLine)
{
  const auto source = makeSource();
  const std::array<double, 7> pregrasp{0.011, 0.210, -0.217, -1.639, 0.047, 1.844, -1.495};
  const std::array<double, 7> grasp{0.000, 0.323, -0.200, -1.875, 0.078, 2.190, -1.515};
  const auto model =
    arm_kinematics::loadFrankaFerModel(KINEMATICS_YAML_PATH, JOINT_LIMITS_YAML_PATH);
  const auto tcp = [&model](const std::array<double, 7> & q) {
      arm_kinematics::JointVector v;
      for (std::size_t i = 0; i < 7; ++i) {
        v(static_cast<Eigen::Index>(i)) = q[i];
      }
      return arm_kinematics::fk(model, v).hand_tcp;
    };
  const auto path = source.cartesianLine(pregrasp, grasp);
  ASSERT_GE(path.size(), 30u);  // 150 mm at 5 mm
  EXPECT_EQ(path.front(), pregrasp);
  const Eigen::Vector3d a = tcp(pregrasp).translation(), b = tcp(grasp).translation();
  const Eigen::Vector3d u = (b - a).normalized();
  double worst_off = 0.0, worst_jump = 0.0;
  for (std::size_t k = 0; k < path.size(); ++k) {
    const Eigen::Vector3d d = tcp(path[k]).translation() - a;
    worst_off = std::max(worst_off, (d - d.dot(u) * u).norm());
    if (k > 0) {
      for (std::size_t i = 0; i < 7; ++i) {
        worst_jump = std::max(worst_jump, std::abs(path[k][i] - path[k - 1][i]));
      }
    }
  }
  EXPECT_LT(worst_off, 2e-4);
  EXPECT_LT(worst_jump, 0.05);
  EXPECT_LT((tcp(path.back()).translation() - b).norm(), 2e-4);
  EXPECT_LT(
    Eigen::Quaterniond(tcp(path.back()).linear()).angularDistance(
      Eigen::Quaterniond(tcp(grasp).linear())), 2e-3);
}

TEST(DiffIkWaypointSource, UsesObjectPositionAndHoldsTargetForPhase)
{
  auto source = makeSource();
  source.setSeed(home());
  const ObjectPose box{0.52, 0.01, 0.241};
  const JointTarget first = source.jointTargetFor(Phase::kPregrasp, box, kTable);
  ASSERT_TRUE(source.diagnostics());
  const double shifted_x = source.diagnostics()->tcp_target.translation().x();
  source.beginEpisode();
  source.setSeed(home());
  source.jointTargetFor(Phase::kPregrasp, {0.5, 0.0, 0.241}, kTable);
  EXPECT_NEAR(shifted_x - source.diagnostics()->tcp_target.translation().x(), 0.02, 1e-9);
  source.beginEpisode();
  source.setSeed(home());
  source.jointTargetFor(Phase::kPregrasp, box, kTable);
  const JointTarget cached = source.jointTargetFor(Phase::kPregrasp, {0.7, 0.0, 0.241}, kTable);
  EXPECT_EQ(first.arm_positions, cached.arm_positions);
  EXPECT_EQ(first.gripper_width_m, cached.gripper_width_m);
}

TEST(DiffIkWaypointSource, ResetRelatchesObjectPosition)
{
  auto source = makeSource();
  source.setSeed(home());
  const JointTarget first = source.jointTargetFor(Phase::kPregrasp, {0.5, 0.0, 0.241}, kTable);
  source.beginEpisode();
  source.setSeed(home());
  const JointTarget shifted = source.jointTargetFor(Phase::kPregrasp, {0.52, 0.0, 0.241}, kTable);
  EXPECT_GT(
    std::abs(first.arm_positions[1] - shifted.arm_positions[1]) +
    std::abs(first.arm_positions[3] - shifted.arm_positions[3]), 1e-4);
}

TEST(DiffIkWaypointSource, RejectsUnreachableObjectWithoutPublishingACommand)
{
  auto source = makeSource();
  source.setSeed(home());
  EXPECT_THROW(
    source.jointTargetFor(Phase::kPregrasp, {5.0, 5.0, 5.0}, kTable), std::runtime_error);
}

TEST(DiffIkWaypointSource, TheWristKeepsThePregraspRotationWhileTheBoxTurnsInTheHand)
{
  auto source = makeSource();
  source.setSeed(home());
  const double half = 20.0 * M_PI / 360.0;
  const ObjectPose box{0.5, 0.0, 0.241, std::cos(half), 0.0, 0.0, std::sin(half)};
  source.jointTargetFor(Phase::kPregrasp, box, kTable);
  source.setSeed(home());
  source.jointTargetFor(Phase::kPlace, box, kTable);
  const Eigen::Matrix3d planned = source.diagnostics()->tcp_target.linear();

  auto turned_source = makeSource();
  turned_source.setSeed(home());
  turned_source.jointTargetFor(Phase::kPregrasp, box, kTable);
  turned_source.setSeed(home());
  // While carried the box has turned by 15 degrees; the place rotation must not follow.
  const double turned = 35.0 * M_PI / 360.0;
  turned_source.jointTargetFor(
    Phase::kPlace, {0.5, 0.3, 0.30, std::cos(turned), 0.0, 0.0, std::sin(turned)}, kTable);
  EXPECT_TRUE(turned_source.diagnostics()->tcp_target.linear().isApprox(planned, 1e-12));
}

TEST(DiffIkWaypointSource, ConsumesInjectedCartesianTaskTarget)
{
  auto cartesian_source = std::make_shared<PickPlaceCartesianWaypointSource>();
  DiffIkWaypointSource source(
    arm_kinematics::loadFrankaFerModel(KINEMATICS_YAML_PATH, JOINT_LIMITS_YAML_PATH),
    cartesian_source);
  source.setSeed(home());
  const auto target = source.jointTargetFor(
    Phase::kPlace, {0.5, 0.0, 0.241}, PlaceTarget{0.48,
      0.28, kTable.support_z});
  ASSERT_TRUE(source.diagnostics());
  EXPECT_EQ(target.gripper_width_m, 0.0);
  EXPECT_NEAR(source.diagnostics()->tcp_target.translation().x(), 0.48, 1e-12);
  EXPECT_NEAR(source.diagnostics()->tcp_target.translation().y(), 0.28, 1e-12);
  EXPECT_LT(source.diagnostics()->ik.position_error, 1e-4);
}

}  // namespace
}  // namespace task_executor
