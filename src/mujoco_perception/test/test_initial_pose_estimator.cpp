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

// Tests for InitialPoseEstimator: the state it reports while its window fills, that a miss is
// reported as a miss (not as "still warming up"), and that reset() really forgets the scene.
// The detectors and the window have their own tests; this file only tests how they are joined.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "mujoco_perception/initial_pose_estimator.hpp"
#include "workcell_camera.hpp"

namespace
{

using mujoco_perception::DetectionRejection;
using mujoco_perception::InitialBinConfig;
using mujoco_perception::InitialBoxConfig;
using mujoco_perception::InitialPoseEstimator;
using mujoco_perception::InitialPoseState;
using mujoco_perception_test::SceneBox;

constexpr double kTableZ = 0.22;

SceneBox boxAt(double x, double y, double yaw_deg)
{
  return SceneBox{Eigen::Vector3d(x, y, kTableZ + 0.02), yaw_deg * M_PI / 180.0,
    Eigen::Vector3d(0.02, 0.02, 0.02)};
}

// A box and a bin, as the bridge would place them.
std::vector<float> scene(double box_x, double box_y, double bin_x, double bin_y, double bin_yaw_deg)
{
  auto boxes = mujoco_perception_test::binBoxes(bin_x, bin_y, 0.227, bin_yaw_deg * M_PI / 180.0);
  boxes.push_back(boxAt(box_x, box_y, 20.0));
  return mujoco_perception_test::renderDepth(boxes, kTableZ);
}

mujoco_perception::InitialPoseEstimate feed(
  InitialPoseEstimator & estimator, const std::vector<float> & depth)
{
  return estimator.update(
    depth, mujoco_perception_test::workcellCameraInfo(),
    mujoco_perception_test::workcellWorldFromOptical());
}

InitialPoseEstimator makeEstimator(std::size_t frames)
{
  return InitialPoseEstimator(frames, InitialBoxConfig{}, InitialBinConfig{});
}

TEST(InitialPoseEstimator, WarmsUpUntilTheWindowIsFullAndThenMeasuresBoth)
{
  auto estimator = makeEstimator(3);
  const auto depth = scene(0.42, -0.10, 0.58, 0.20, 30.0);

  for (std::size_t frame = 1; frame <= 2; ++frame) {
    const auto estimate = feed(estimator, depth);
    EXPECT_EQ(estimate.boxState(), InitialPoseState::kWarmingUp) << "frame " << frame;
    EXPECT_EQ(estimate.binState(), InitialPoseState::kWarmingUp) << "frame " << frame;
    EXPECT_EQ(estimate.frames_averaged, frame);
    EXPECT_EQ(estimate.frames_required, 3U);
    EXPECT_TRUE(estimate.box.candidates.empty()) << "no detection is run before it is full";
    EXPECT_TRUE(estimate.bin.candidates.empty());
  }
  const auto estimate = feed(estimator, depth);
  ASSERT_EQ(estimate.boxState(), InitialPoseState::kMeasured);
  ASSERT_EQ(estimate.binState(), InitialPoseState::kMeasured);
  EXPECT_NEAR(estimate.box.position.x(), 0.42, 0.003);
  EXPECT_NEAR(estimate.box.position.y(), -0.10, 0.003);
  EXPECT_NEAR(estimate.bin.position.x(), 0.58, 0.003);
  EXPECT_NEAR(estimate.bin.position.y(), 0.20, 0.003);
}

TEST(InitialPoseEstimator, AFullWindowThatFindsNothingIsNotMeasuredNotWarmingUp)
{
  auto estimator = makeEstimator(2);
  const auto empty_table = mujoco_perception_test::renderDepth({}, kTableZ);
  feed(estimator, empty_table);
  const auto estimate = feed(estimator, empty_table);
  EXPECT_EQ(estimate.boxState(), InitialPoseState::kNotMeasured);
  EXPECT_EQ(estimate.binState(), InitialPoseState::kNotMeasured);
  EXPECT_EQ(estimate.box.rejection, DetectionRejection::kNoBandPixels);
  EXPECT_EQ(estimate.bin.rejection, DetectionRejection::kNoBandPixels);
  EXPECT_EQ(estimate.frames_averaged, 2U);
}

TEST(InitialPoseEstimator, ResetForgetsTheOldSceneSoANewSceneIsNotBlendedWithTheOldOne)
{
  auto estimator = makeEstimator(2);
  const auto before = scene(0.40, -0.15, 0.58, 0.20, 0.0);
  const auto after = scene(0.60, -0.10, 0.45, 0.22, 90.0);
  feed(estimator, before);
  ASSERT_EQ(feed(estimator, before).boxState(), InitialPoseState::kMeasured);

  estimator.reset();
  EXPECT_EQ(estimator.framesInWindow(), 0U);
  EXPECT_EQ(feed(estimator, after).boxState(), InitialPoseState::kWarmingUp);
  const auto estimate = feed(estimator, after);
  ASSERT_EQ(estimate.boxState(), InitialPoseState::kMeasured);
  ASSERT_EQ(estimate.binState(), InitialPoseState::kMeasured);
  // Only the new frames are in the window: both objects are where they are now.
  EXPECT_NEAR(estimate.box.position.x(), 0.60, 0.003);
  EXPECT_NEAR(estimate.bin.position.x(), 0.45, 0.003);
  EXPECT_NEAR(estimate.bin.position.y(), 0.22, 0.003);
}

}  // namespace
