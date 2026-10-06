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

// Tests for detectInitialBin() on synthetic depth images whose true answer is known by
// construction (workcell_camera.hpp renders the table, the bin and boxes by exact ray casting).
//
// Tolerances are the pixel-quantisation bounds written before the first run (Week 4.1 Stage 6,
// 6.3): the camera sees about 3.3 mm of table per pixel, so a 150 mm rectangle is placed to
// about 3 mm, and the direction of its 142 mm side to about atan(3.3 / 142) = 1.3 degrees,
// which is rounded up to 3.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "mujoco_perception/initial_bin_detector.hpp"
#include "workcell_camera.hpp"

namespace
{

using mujoco_perception::DetectionRejection;
using mujoco_perception::InitialBinConfig;
using mujoco_perception::InitialBinResult;
using mujoco_perception::detectInitialBin;
using mujoco_perception_test::SceneBox;

constexpr double kTableZ = 0.22;
constexpr double kFloorZ = 0.227;
constexpr double kPositionTolerance = 0.003;
constexpr double kHeightTolerance = 0.001;
constexpr double kYawToleranceDeg = 3.0;

InitialBinResult detect(const std::vector<SceneBox> & boxes, const InitialBinConfig & config = {})
{
  return detectInitialBin(
    mujoco_perception_test::renderDepth(boxes, kTableZ),
    mujoco_perception_test::workcellCameraInfo(),
    mujoco_perception_test::workcellWorldFromOptical(), config);
}

// Yaw error modulo 180 degrees, in [-90, 90).
double yawErrorDeg(const InitialBinResult & result, double truth_deg)
{
  double difference = std::fmod(result.yaw_rad * 180.0 / M_PI - truth_deg, 180.0);
  if (difference >= 90.0) {
    difference -= 180.0;
  }
  if (difference < -90.0) {
    difference += 180.0;
  }
  return difference;
}

TEST(InitialBin, AFlatBinIsMeasuredWhereItIsWithItsFloorHeight)
{
  const auto result = detect(mujoco_perception_test::binBoxes(0.55, 0.25, kFloorZ, 0.0));
  ASSERT_TRUE(result.measured()) << initialBinRejectionName(result.rejection);
  EXPECT_NEAR(result.position.x(), 0.55, kPositionTolerance);
  EXPECT_NEAR(result.position.y(), 0.25, kPositionTolerance);
  EXPECT_NEAR(result.position.z(), kFloorZ, kHeightTolerance);
  EXPECT_LT(std::abs(yawErrorDeg(result, 0.0)), kYawToleranceDeg);
  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_TRUE(result.candidates.front().matches);
  // The rectangle of pixel centres is a little smaller than the bin: 138.2 mm instead of 142 mm
  // for the short side here (on the recorded Stage 3 frames it is at most 1 mm short). A pixel
  // covers 3.3 to 4.7 mm of the table, so this is within one pixel; the cause was not isolated.
  // The detector's own rule is +-8 mm on both sides.
  EXPECT_NEAR(result.candidates.front().side_along_m, 0.152, InitialBinConfig{}.side_tolerance_m);
  EXPECT_NEAR(result.candidates.front().side_across_m, 0.142, InitialBinConfig{}.side_tolerance_m);
}

TEST(InitialBin, TheYawOfARotatedBinIsRecoveredModuloOneHundredAndEightyDegrees)
{
  // 100, 135 and 170 degrees are folded to -80, -45 and -10; the bin looks the same turned by 180.
  for (const double yaw_deg : {0.0, 30.0, 60.0, 89.0, 100.0, 135.0, 170.0}) {
    const auto result = detect(
      mujoco_perception_test::binBoxes(0.50, 0.15, kFloorZ, yaw_deg * M_PI / 180.0));
    ASSERT_TRUE(result.measured()) << "yaw " << yaw_deg << ": "
                                   << initialBinRejectionName(result.rejection);
    EXPECT_LT(std::abs(yawErrorDeg(result, yaw_deg)), kYawToleranceDeg) << "yaw " << yaw_deg;
    EXPECT_GE(result.yaw_rad, -M_PI / 2.0);
    EXPECT_LT(result.yaw_rad, M_PI / 2.0);
    EXPECT_NEAR(result.position.x(), 0.50, kPositionTolerance) << "yaw " << yaw_deg;
    EXPECT_NEAR(result.position.y(), 0.15, kPositionTolerance) << "yaw " << yaw_deg;
  }
}

TEST(InitialBin, ABoxBesideTheBinIsAnotherBlockAndDoesNotDisturbIt)
{
  // The lower part of the box's side faces is inside the bin's height band, so the box is a
  // block here too; it is not the bin's size.
  auto scene = mujoco_perception_test::binBoxes(0.58, 0.22, kFloorZ, 0.4);
  scene.push_back(
    SceneBox{Eigen::Vector3d(0.42, -0.05, kTableZ + 0.02), 0.3,
      Eigen::Vector3d(0.02, 0.02, 0.02)});
  const auto result = detect(scene);
  ASSERT_TRUE(result.measured()) << initialBinRejectionName(result.rejection);
  EXPECT_NEAR(result.position.x(), 0.58, kPositionTolerance);
  EXPECT_NEAR(result.position.y(), 0.22, kPositionTolerance);
  EXPECT_LT(std::abs(yawErrorDeg(result, 0.4 * 180.0 / M_PI)), kYawToleranceDeg);
  int matching = 0;
  for (const auto & candidate : result.candidates) {
    matching += candidate.matches ? 1 : 0;
  }
  EXPECT_EQ(matching, 1);
}

TEST(InitialBin, ABinCutByTheSearchRegionIsRefusedNotMeasuredFromItsVisiblePart)
{
  // Centred on the region's lower x edge (0.20): only half of the bin is inside.
  const auto result = detect(mujoco_perception_test::binBoxes(0.20, 0.0, kFloorZ, 0.0));
  EXPECT_EQ(result.rejection, DetectionRejection::kNoMatchingRectangle);
  ASSERT_FALSE(result.candidates.empty());
  EXPECT_FALSE(result.candidates.front().matches);
}

}  // namespace
