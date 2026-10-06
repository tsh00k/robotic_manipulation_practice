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

#include "task_executor/cartesian_waypoint_source.hpp"

namespace task_executor
{
namespace
{

// The legacy scene's place target on the table.
const PlaceTarget kTable{};

TEST(CartesianWaypointSource, GraspUsesWorldObjectPositionAndNormalizedDownwardRotation)
{
  const PickPlaceCartesianWaypointSource source;
  const ObjectPose box{0.5, 0.0, 0.241};
  const auto grasp = source.waypointFor(Phase::kGrasp, box, kTable);
  const auto hover = source.waypointFor(Phase::kPregrasp, box, kTable);
  const auto shifted = source.waypointFor(Phase::kGrasp, {0.52, 0.04, 0.241}, kTable);
  EXPECT_EQ(grasp.phase, Phase::kGrasp);
  EXPECT_NEAR(grasp.world_to_hand_tcp.translation().x(), box.x, 1e-12);
  EXPECT_NEAR(grasp.world_to_hand_tcp.translation().y(), box.y, 1e-12);
  EXPECT_NEAR(grasp.world_to_hand_tcp.translation().z(), box.z, 1e-12);
  EXPECT_NEAR(hover.world_to_hand_tcp.translation().z() - box.z, 0.15, 1e-12);
  EXPECT_NEAR(shifted.world_to_hand_tcp.translation().x() - box.x, 0.02, 1e-12);
  EXPECT_NEAR(shifted.world_to_hand_tcp.translation().y() - box.y, 0.04, 1e-12);
  EXPECT_NEAR(Eigen::Quaterniond(grasp.world_to_hand_tcp.linear()).norm(), 1.0, 1e-12);
  EXPECT_NEAR(grasp.world_to_hand_tcp.linear()(2, 2), -1.0, 1e-12);
  EXPECT_TRUE(
    grasp.world_to_hand_tcp.linear().isApprox(
      shifted.world_to_hand_tcp.linear(), 1e-12));
  EXPECT_TRUE(
    grasp.world_to_hand_tcp.linear().isApprox(
      source.waypointFor(Phase::kPlace, box, kTable).world_to_hand_tcp.linear(), 1e-12));
}

TEST(CartesianWaypointSource, ToolYawIsIndependentOfTargetPosition)
{
  PickPlaceGeometry geometry;
  geometry.tool_yaw_rad = 0.25;
  const PickPlaceCartesianWaypointSource source(geometry);
  const ObjectPose box{0.5, 0.0, 0.241};
  const auto grasp = source.waypointFor(Phase::kGrasp, box, kTable);
  const auto shifted = source.waypointFor(Phase::kGrasp, {0.5, 0.04, 0.241}, kTable);
  const auto place = source.waypointFor(Phase::kPlace, box, kTable);
  EXPECT_TRUE(
    grasp.world_to_hand_tcp.linear().isApprox(
      shifted.world_to_hand_tcp.linear(), 1e-12));
  EXPECT_TRUE(
    grasp.world_to_hand_tcp.linear().isApprox(
      place.world_to_hand_tcp.linear(), 1e-12));
  const Eigen::Matrix3d default_rotation =
    PickPlaceCartesianWaypointSource().waypointFor(
    Phase::kGrasp, box,
    kTable).world_to_hand_tcp.linear();
  EXPECT_NEAR(
    Eigen::Quaterniond(grasp.world_to_hand_tcp.linear()).angularDistance(
      Eigen::Quaterniond(default_rotation)), 0.25, 1e-12);
}

TEST(CartesianWaypointSource, PlaceIsAbsoluteAndGripperTimingMatchesPhases)
{
  const PickPlaceCartesianWaypointSource source;
  const ObjectPose box{0.61, -0.04, 0.25};
  const auto place = source.waypointFor(Phase::kPlace, box, kTable);
  const auto place_other_box = source.waypointFor(Phase::kPlace, {0.45, 0.07, 0.27}, kTable);
  EXPECT_NEAR(place.world_to_hand_tcp.translation().x(), 0.5, 1e-12);
  EXPECT_NEAR(place.world_to_hand_tcp.translation().y(), 0.3, 1e-12);
  EXPECT_NEAR(place.world_to_hand_tcp.translation().z(), 0.29, 1e-12);
  EXPECT_TRUE(
    place.world_to_hand_tcp.matrix().isApprox(
      place_other_box.world_to_hand_tcp.matrix(), 1e-12));
  for (const Phase phase : {Phase::kClose, Phase::kLift, Phase::kPreplace, Phase::kPlace}) {
    EXPECT_EQ(source.waypointFor(phase, box, kTable).gripper_width_m, 0.0);
  }
  for (const Phase phase : {Phase::kHome, Phase::kPregrasp, Phase::kGrasp,
      Phase::kOpen, Phase::kRetract})
  {
    EXPECT_EQ(source.waypointFor(phase, box, kTable).gripper_width_m, 0.08);
  }
  EXPECT_TRUE(
    place.world_to_hand_tcp.matrix().isApprox(
      source.waypointFor(Phase::kOpen, box, kTable).world_to_hand_tcp.matrix(), 1e-12));
}

TEST(CartesianWaypointSource, ThePlaceTargetAndItsSupportHeightComeFromTheEpisode)
{
  // A bin's inner floor at (0.56, -0.12, 0.227): PLACE and OPEN are above the floor, not the
  // table; PREPLACE and RETRACT hover above it by the same height as above the table.
  const PickPlaceCartesianWaypointSource source;
  const ObjectPose box{0.45, 0.10, 0.24};
  const PlaceTarget bin{0.56, -0.12, 0.227};
  const auto place = source.waypointFor(Phase::kPlace, box, bin);
  EXPECT_NEAR(place.world_to_hand_tcp.translation().x(), 0.56, 1e-12);
  EXPECT_NEAR(place.world_to_hand_tcp.translation().y(), -0.12, 1e-12);
  EXPECT_NEAR(place.world_to_hand_tcp.translation().z(), 0.227 + 0.02 + 0.05, 1e-12);
  for (const Phase phase : {Phase::kPreplace, Phase::kRetract}) {
    EXPECT_NEAR(
      source.waypointFor(phase, box, bin).world_to_hand_tcp.translation().z(),
      0.227 + 0.02 + 0.15, 1e-12);
  }
  // The grasp side does not move with the place target.
  EXPECT_NEAR(
    source.waypointFor(Phase::kGrasp, box, bin).world_to_hand_tcp.translation().x(), 0.45, 1e-12);
  EXPECT_THROW(
    source.waypointFor(Phase::kPlace, box, PlaceTarget{0.5, std::nan(""), 0.227}),
    std::invalid_argument);
}

TEST(CartesianWaypointSource, RejectsInvalidPoseAndGeometry)
{
  const PickPlaceCartesianWaypointSource source;
  ObjectPose box;
  box.qw = 0.0;
  EXPECT_THROW(source.waypointFor(Phase::kGrasp, box, kTable), std::invalid_argument);
  box.qw = 1.0;
  box.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(source.waypointFor(Phase::kGrasp, box, kTable), std::invalid_argument);
  PickPlaceGeometry bad;
  bad.hover_height_m = -0.1;
  EXPECT_THROW(PickPlaceCartesianWaypointSource invalid(bad), std::invalid_argument);
  bad = PickPlaceGeometry{};
  bad.tool_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(PickPlaceCartesianWaypointSource invalid(bad), std::invalid_argument);
}

}  // namespace
}  // namespace task_executor
