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

#include "task_executor/bin_containment.hpp"

namespace task_executor
{
namespace
{

constexpr double kFloor = 0.227;
constexpr double kOnFloor = kFloor + 0.02;

PlanarPose bin(double yaw_deg = 0.0)
{
  return {0.5, 0.2, kFloor, yaw_deg * M_PI / 180.0};
}

TEST(BinContainment, ACentredBoxIsInsideWithItsClearance)
{
  const auto result = boxInBin({0.5, 0.2, kOnFloor, 0.0}, bin());
  EXPECT_TRUE(result.inside);
  EXPECT_NEAR(result.min_clearance_m, 0.065 - 0.02, 1e-12);  // the short side limits it
  EXPECT_NEAR(result.height_error_m, 0.0, 1e-12);
}

TEST(BinContainment, ACornerOverTheWallIsOutside)
{
  // 30 mm off centre along the short side: the corner is 5 mm beyond the 65 mm inner face.
  const auto result = boxInBin({0.5, 0.2 + 0.050, kOnFloor, 0.0}, bin());
  EXPECT_FALSE(result.inside);
  EXPECT_NEAR(result.min_clearance_m, -0.005, 1e-12);
}

TEST(BinContainment, InsideButCloserToAWallThanTheMarginIsNotClaimed)
{
  // Really inside, 3 mm from the wall, but the detection could be 5 mm off.
  const auto result = boxInBin({0.5, 0.2 + 0.042, kOnFloor, 0.0}, bin());
  EXPECT_NEAR(result.min_clearance_m, 0.003, 1e-12);
  EXPECT_FALSE(result.inside);
}

TEST(BinContainment, ABoxRestingOnAWallRimIsNotInsideWhateverItsXy)
{
  // Centred over the bin but 12 mm too high: on the rim of a wall, not on the floor.
  EXPECT_FALSE(boxInBin({0.5, 0.2, kOnFloor + 0.012, 0.0}, bin()).inside);
  EXPECT_TRUE(boxInBin({0.5, 0.2, kOnFloor + 0.004, 0.0}, bin()).inside);
}

TEST(BinContainment, TheCornersAreTakenInTheBinsOwnFrameAndWithTheBoxYaw)
{
  // 44 mm off along world x, corners at 64 mm: 6 mm inside a bin whose 70 mm half-opening is
  // along x, but only 1 mm (under the margin) when the bin is turned by 90 degrees.
  const PlanarPose box{0.5 + 0.044, 0.2, kOnFloor, 0.0};
  EXPECT_TRUE(boxInBin(box, bin(0.0)).inside);
  EXPECT_FALSE(boxInBin(box, bin(90.0)).inside);
  // A box turned by 45 degrees reaches 28.3 mm instead of 20 mm: 40 mm off along x leaves
  // 1.7 mm to the wall, under the margin, where the unturned box leaves 10 mm.
  EXPECT_TRUE(boxInBin({0.5 + 0.040, 0.2, kOnFloor, 0.0}, bin()).inside);
  EXPECT_FALSE(boxInBin({0.5 + 0.040, 0.2, kOnFloor, M_PI / 4.0}, bin()).inside);
}

TEST(BinContainment, AnUnknownPoseIsNotInside)
{
  EXPECT_FALSE(boxInBin({std::nan(""), 0.2, kOnFloor, 0.0}, bin()).inside);
}

}  // namespace
}  // namespace task_executor
