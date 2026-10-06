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

// Tests for InitialBoxEstimator: the state it reports while its window fills, that a miss is
// reported as a miss (not as "still warming up"), and that reset() really forgets the scene.
// The detector and the window have their own tests; this file only tests how they are joined.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "mujoco_perception/initial_box_estimator.hpp"
#include "workcell_camera.hpp"

namespace
{

using mujoco_perception::InitialBoxConfig;
using mujoco_perception::InitialBoxEstimator;
using mujoco_perception::InitialBoxRejection;
using mujoco_perception::InitialBoxState;
using mujoco_perception_test::SceneBox;

constexpr double kTableZ = 0.22;

SceneBox boxAt(double x, double y, double yaw_deg)
{
  return SceneBox{Eigen::Vector3d(x, y, kTableZ + 0.02), yaw_deg * M_PI / 180.0,
    Eigen::Vector3d(0.02, 0.02, 0.02)};
}

std::vector<float> scene(const std::vector<SceneBox> & boxes)
{
  return mujoco_perception_test::renderDepth(boxes, kTableZ);
}

mujoco_perception::InitialBoxEstimate feed(
  InitialBoxEstimator & estimator, const std::vector<float> & depth)
{
  return estimator.update(
    depth, mujoco_perception_test::workcellCameraInfo(),
    mujoco_perception_test::workcellWorldFromOptical());
}

TEST(InitialBoxEstimator, WarmsUpUntilTheWindowIsFullAndThenMeasures)
{
  InitialBoxEstimator estimator(3, InitialBoxConfig{});
  const auto depth = scene({boxAt(0.50, 0.05, 20.0)});

  for (std::size_t frame = 1; frame <= 2; ++frame) {
    const auto estimate = feed(estimator, depth);
    EXPECT_EQ(estimate.state, InitialBoxState::kWarmingUp) << "frame " << frame;
    EXPECT_EQ(estimate.frames_averaged, frame);
    EXPECT_EQ(estimate.frames_required, 3U);
    EXPECT_TRUE(estimate.detection.candidates.empty()) << "no detection is run before it is full";
  }
  const auto estimate = feed(estimator, depth);
  ASSERT_EQ(estimate.state, InitialBoxState::kMeasured);
  EXPECT_EQ(estimate.frames_averaged, 3U);
  EXPECT_NEAR(estimate.detection.position.x(), 0.50, 0.003);
  EXPECT_NEAR(estimate.detection.position.y(), 0.05, 0.003);
}

TEST(InitialBoxEstimator, AFullWindowThatFindsNoBoxIsNotMeasuredNotWarmingUp)
{
  InitialBoxEstimator estimator(2, InitialBoxConfig{});
  const auto empty_table = scene({});
  feed(estimator, empty_table);
  const auto estimate = feed(estimator, empty_table);
  EXPECT_EQ(estimate.state, InitialBoxState::kNotMeasured);
  EXPECT_EQ(estimate.detection.rejection, InitialBoxRejection::kNoBandPixels);
  EXPECT_EQ(estimate.frames_averaged, 2U);
}

TEST(InitialBoxEstimator, ASingleFrameWindowMeasuresOnTheFirstFrame)
{
  InitialBoxEstimator estimator(1, InitialBoxConfig{});
  EXPECT_EQ(feed(estimator, scene({boxAt(0.45, -0.1, 0.0)})).state, InitialBoxState::kMeasured);
}

TEST(InitialBoxEstimator, ResetForgetsTheOldSceneSoANewBoxIsNotBlendedWithTheOldOne)
{
  InitialBoxEstimator estimator(2, InitialBoxConfig{});
  const auto before = scene({boxAt(0.40, -0.15, 0.0)});
  const auto after = scene({boxAt(0.60, 0.15, 0.0)});
  feed(estimator, before);
  ASSERT_EQ(feed(estimator, before).state, InitialBoxState::kMeasured);

  estimator.reset();
  EXPECT_EQ(estimator.framesInWindow(), 0U);
  EXPECT_EQ(feed(estimator, after).state, InitialBoxState::kWarmingUp);
  const auto estimate = feed(estimator, after);
  ASSERT_EQ(estimate.state, InitialBoxState::kMeasured);
  // Only the new frames are in the window: the box is where it is now, not between the two.
  EXPECT_NEAR(estimate.detection.position.x(), 0.60, 0.003);
  EXPECT_NEAR(estimate.detection.position.y(), 0.15, 0.003);
}

TEST(InitialBoxEstimator, WithoutResetTheWindowSlidesOverTheOldestFrame)
{
  // Contrast with the test above: no reset, so after the box moves it takes `frames` new frames
  // before the mean is the new scene alone. In between, the old and the new box are both in the
  // window; the mean of the two depths is not a box and must not be reported as one at the old
  // or the new place.
  InitialBoxEstimator estimator(2, InitialBoxConfig{});
  const auto before = scene({boxAt(0.40, -0.15, 0.0)});
  const auto after = scene({boxAt(0.60, 0.15, 0.0)});
  feed(estimator, before);
  feed(estimator, before);
  const auto mixed = feed(estimator, after);
  EXPECT_FALSE(
    mixed.state == InitialBoxState::kMeasured &&
    std::abs(mixed.detection.position.x() - 0.40) < 0.003)
    << "a half-old window reported the old position";
  const auto settled = feed(estimator, after);
  ASSERT_EQ(settled.state, InitialBoxState::kMeasured);
  EXPECT_NEAR(settled.detection.position.x(), 0.60, 0.003);
}

TEST(InitialBoxEstimator, NoFramesIsAConfigurationErrorNotAnEmptyEstimator)
{
  EXPECT_THROW(InitialBoxEstimator(0, InitialBoxConfig{}), std::invalid_argument);
}

TEST(InitialBoxEstimator, TheStateNamesAreTheOnesTheMessageUses)
{
  EXPECT_STREQ(initialBoxStateName(InitialBoxState::kWarmingUp), "WARMING_UP");
  EXPECT_STREQ(initialBoxStateName(InitialBoxState::kNotMeasured), "NOT_MEASURED");
  EXPECT_STREQ(initialBoxStateName(InitialBoxState::kMeasured), "MEASURED");
}

}  // namespace
