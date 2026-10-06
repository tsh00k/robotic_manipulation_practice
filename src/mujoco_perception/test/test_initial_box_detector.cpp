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

// Tests for detectInitialBox() on synthetic depth images whose true answer is known by
// construction (workcell_camera.hpp renders the table and boxes by exact ray casting).
//
// Tolerances: the camera sees about 3.3 mm of table per pixel (0.86 m / 257 px), and the
// rectangle can only touch pixel centres, so the position of a 40 mm face is good to about one
// pixel (3 mm here) and its direction to about atan(one pixel / 40 mm) = 4.7 degrees. These are
// the quantisation bounds of the measurement, not values tuned until the tests passed; the
// error tables of Week 4.1 Stage 3 (median 1 mm, 0.06 degrees) are far inside them.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "mujoco_perception/depth_window.hpp"
#include "mujoco_perception/initial_box_detector.hpp"
#include "workcell_camera.hpp"

namespace
{

using mujoco_perception::InitialBoxConfig;
using mujoco_perception::InitialBoxRejection;
using mujoco_perception::InitialBoxResult;
using mujoco_perception::detectInitialBox;
using mujoco_perception_test::SceneBox;

constexpr double kTableZ = 0.22;
constexpr double kBoxHalf = 0.02;
constexpr double kPositionTolerance = 0.003;
constexpr double kYawToleranceDeg = 5.0;

SceneBox flatBox(double x, double y, double yaw_deg)
{
  return SceneBox{Eigen::Vector3d(x, y, kTableZ + kBoxHalf), yaw_deg * M_PI / 180.0,
    Eigen::Vector3d(kBoxHalf, kBoxHalf, kBoxHalf)};
}

InitialBoxResult detect(const std::vector<float> & depth, const InitialBoxConfig & config = {})
{
  return detectInitialBox(
    depth, mujoco_perception_test::workcellCameraInfo(),
    mujoco_perception_test::workcellWorldFromOptical(), config);
}

InitialBoxResult detectScene(
  const std::vector<SceneBox> & boxes, const InitialBoxConfig & config = {})
{
  return detect(mujoco_perception_test::renderDepth(boxes, kTableZ), config);
}

double yawErrorDeg(const InitialBoxResult & result, double truth_deg)
{
  double difference = std::fmod(result.yaw_rad * 180.0 / M_PI - truth_deg, 90.0);
  if (difference >= 45.0) {
    difference -= 90.0;
  }
  if (difference < -45.0) {
    difference += 90.0;
  }
  return difference;
}

TEST(InitialBox, AFlatBoxIsMeasuredWhereItIs)
{
  const auto result = detectScene({flatBox(0.50, 0.00, 0.0)});
  ASSERT_TRUE(result.measured()) << initialBoxRejectionName(result.rejection);
  EXPECT_NEAR(result.position.x(), 0.50, kPositionTolerance);
  EXPECT_NEAR(result.position.y(), 0.00, kPositionTolerance);
  // The centre height comes from the top face: 0.22 table + 0.04 box - 0.02 half = 0.24.
  EXPECT_NEAR(result.position.z(), 0.24, 0.001);
  EXPECT_LT(std::abs(yawErrorDeg(result, 0.0)), kYawToleranceDeg);
  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_TRUE(result.candidates.front().matches_box);
}

TEST(InitialBox, TheYawOfARotatedBoxIsRecoveredAcrossTheWholeFoldedRange)
{
  for (const double yaw_deg : {-40.0, -25.0, -10.0, 10.0, 25.0, 40.0}) {
    const auto result = detectScene({flatBox(0.46, 0.08, yaw_deg)});
    ASSERT_TRUE(result.measured()) << "yaw " << yaw_deg << ": "
                                   << initialBoxRejectionName(result.rejection);
    EXPECT_LT(std::abs(yawErrorDeg(result, yaw_deg)), kYawToleranceDeg) << "yaw " << yaw_deg;
    EXPECT_NEAR(result.position.x(), 0.46, kPositionTolerance) << "yaw " << yaw_deg;
    EXPECT_NEAR(result.position.y(), 0.08, kPositionTolerance) << "yaw " << yaw_deg;
  }
}

TEST(InitialBox, YawIsReportedModuloNinetyDegreesAsASquareRepeats)
{
  // 80 degrees and -10 degrees are the same square; the result is always in [-45, 45).
  const auto result = detectScene({flatBox(0.50, 0.00, 80.0)});
  ASSERT_TRUE(result.measured());
  EXPECT_GE(result.yaw_rad, -M_PI / 4.0);
  EXPECT_LT(result.yaw_rad, M_PI / 4.0);
  EXPECT_LT(std::abs(result.yaw_rad * 180.0 / M_PI - (-10.0)), kYawToleranceDeg);
  const auto twin = detectScene({flatBox(0.50, 0.00, -10.0)});
  ASSERT_TRUE(twin.measured());
  EXPECT_NEAR(result.yaw_rad, twin.yaw_rad, 1e-3);
  EXPECT_NEAR(result.position.x(), twin.position.x(), 1e-4);
}

TEST(InitialBox, ATableWithoutAnythingOnItHasNoBandPixels)
{
  const auto result = detectScene({});
  EXPECT_EQ(result.rejection, InitialBoxRejection::kNoBandPixels);
  EXPECT_TRUE(result.candidates.empty());
  EXPECT_GT(result.valid_depth_pixels, 0U);
}

TEST(InitialBox, ABlockOfTheWrongSizeIsReportedWithItsSidesAndNotAccepted)
{
  // 60 x 60 mm and as tall as the box: it is in the band but is not the box.
  const SceneBox big{Eigen::Vector3d(0.50, 0.0, kTableZ + kBoxHalf), 0.0,
    Eigen::Vector3d(0.03, 0.03, kBoxHalf)};
  const auto result = detectScene({big});
  EXPECT_EQ(result.rejection, InitialBoxRejection::kNoMatchingRectangle);
  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_FALSE(result.candidates.front().matches_box);
  EXPECT_NEAR(result.candidates.front().side_along_m, 0.06, kPositionTolerance);
  EXPECT_NEAR(result.candidates.front().side_across_m, 0.06, kPositionTolerance);
}

TEST(InitialBox, TheBinIsNeverACandidateAndDoesNotDisturbTheBox)
{
  // Bin walls reach 0.239 m, below the band's lower edge 0.245 m, so the whole bin is out of it.
  auto scene = mujoco_perception_test::binBoxes(0.55, 0.30, 0.227, 0.4);
  scene.push_back(flatBox(0.45, -0.10, 15.0));
  const auto result = detectScene(scene);
  ASSERT_TRUE(result.measured()) << initialBoxRejectionName(result.rejection);
  EXPECT_EQ(result.candidates.size(), 1U);
  EXPECT_NEAR(result.position.x(), 0.45, kPositionTolerance);
  EXPECT_NEAR(result.position.y(), -0.10, kPositionTolerance);
  EXPECT_LT(std::abs(yawErrorDeg(result, 15.0)), kYawToleranceDeg);
}

TEST(InitialBox, ABoxStandingOnTheBinFloorKeepsItsMeasuredHeight)
{
  // Resting on the bin floor (top at 0.227 m) the box centre is 0.247 m, not the table 0.24 m;
  // the height is taken from the top face, so it follows.
  auto scene = mujoco_perception_test::binBoxes(0.50, 0.30, 0.227, 0.0);
  scene.push_back(
    SceneBox{Eigen::Vector3d(0.50, 0.30, 0.227 + kBoxHalf), 0.0,
      Eigen::Vector3d(kBoxHalf, kBoxHalf, kBoxHalf)});
  const auto result = detectScene(scene);
  ASSERT_TRUE(result.measured()) << initialBoxRejectionName(result.rejection);
  EXPECT_NEAR(result.position.z(), 0.247, 0.0015);
}

TEST(InitialBox, AveragingTenNoisyFramesRecoversTheBoxThatAGlancingNoisyFrameWould)
{
  // Independent 2 mm depth noise on every valid pixel of 10 frames of one static scene; the
  // window mean has about 0.63 mm of noise. Fixed seed, so the outcome is deterministic.
  const auto clean = mujoco_perception_test::renderDepth({flatBox(0.55, -0.05, 33.0)}, kTableZ);
  std::mt19937 generator(20261006);
  std::normal_distribution<float> noise(0.0F, 0.002F);
  mujoco_perception::DepthWindow window(10);
  for (int frame = 0; frame < 10; ++frame) {
    auto noisy = clean;
    for (auto & value : noisy) {
      if (std::isfinite(value)) {
        value += noise(generator);
      }
    }
    window.push(noisy);
  }
  ASSERT_TRUE(window.full());
  const auto result = detect(window.mean());
  ASSERT_TRUE(result.measured()) << initialBoxRejectionName(result.rejection);
  EXPECT_NEAR(result.position.x(), 0.55, kPositionTolerance);
  EXPECT_NEAR(result.position.y(), -0.05, kPositionTolerance);
  EXPECT_LT(std::abs(yawErrorDeg(result, 33.0)), kYawToleranceDeg);
}

}  // namespace
