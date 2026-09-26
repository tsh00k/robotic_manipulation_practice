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

#include <array>
#include <cmath>
#include <memory>

#include "arm_kinematics/forward_kinematics.hpp"
#include "task_executor/diff_ik_waypoint_source.hpp"

namespace task_executor
{
namespace
{

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

TEST(DiffIkWaypointSource, UsesObjectPositionAndHoldsTargetForPhase)
{
  auto source = makeSource();
  source.setSeed(home());
  const ObjectPose box{0.52, 0.01, 0.241};
  const JointTarget first = source.jointTargetFor(Phase::kPregrasp, box);
  ASSERT_TRUE(source.diagnostics());
  const double shifted_x = source.diagnostics()->tcp_target.translation().x();
  source.beginEpisode();
  source.setSeed(home());
  source.jointTargetFor(Phase::kPregrasp, {0.5, 0.0, 0.241});
  EXPECT_NEAR(shifted_x - source.diagnostics()->tcp_target.translation().x(), 0.02, 1e-9);
  source.beginEpisode();
  source.setSeed(home());
  source.jointTargetFor(Phase::kPregrasp, box);
  const JointTarget cached = source.jointTargetFor(Phase::kPregrasp, {0.7, 0.0, 0.241});
  EXPECT_EQ(first.arm_positions, cached.arm_positions);
  EXPECT_EQ(first.gripper_width_m, cached.gripper_width_m);
}

TEST(DiffIkWaypointSource, ResetRelatchesObjectPosition)
{
  auto source = makeSource();
  source.setSeed(home());
  const JointTarget first = source.jointTargetFor(Phase::kPregrasp, {0.5, 0.0, 0.241});
  source.beginEpisode();
  source.setSeed(home());
  const JointTarget shifted = source.jointTargetFor(Phase::kPregrasp, {0.52, 0.0, 0.241});
  EXPECT_GT(
    std::abs(first.arm_positions[1] - shifted.arm_positions[1]) +
    std::abs(first.arm_positions[3] - shifted.arm_positions[3]), 1e-4);
}

TEST(DiffIkWaypointSource, RejectsUnreachableObjectWithoutPublishingACommand)
{
  auto source = makeSource();
  source.setSeed(home());
  EXPECT_THROW(
    source.jointTargetFor(Phase::kPregrasp, {5.0, 5.0, 5.0}), std::runtime_error);
}

TEST(DiffIkWaypointSource, ConsumesInjectedCartesianTaskTarget)
{
  PickPlaceGeometry geometry;
  geometry.place_x_m = 0.48;
  geometry.place_y_m = 0.28;
  auto cartesian_source = std::make_shared<PickPlaceCartesianWaypointSource>(geometry);
  DiffIkWaypointSource source(
    arm_kinematics::loadFrankaFerModel(KINEMATICS_YAML_PATH, JOINT_LIMITS_YAML_PATH),
    cartesian_source);
  source.setSeed(home());
  const auto target = source.jointTargetFor(Phase::kPlace, {0.5, 0.0, 0.241});
  ASSERT_TRUE(source.diagnostics());
  EXPECT_EQ(target.gripper_width_m, 0.0);
  EXPECT_NEAR(source.diagnostics()->tcp_target.translation().x(), 0.48, 1e-12);
  EXPECT_NEAR(source.diagnostics()->tcp_target.translation().y(), 0.28, 1e-12);
  EXPECT_LT(source.diagnostics()->ik.position_error, 1e-4);
}

}  // namespace
}  // namespace task_executor
