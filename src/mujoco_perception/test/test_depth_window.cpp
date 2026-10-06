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
#include <vector>

#include "mujoco_perception/depth_window.hpp"

namespace
{

using mujoco_perception::DepthWindow;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

TEST(DepthWindow, MeanOfTwoFramesIsTheArithmeticMean)
{
  // Depths in metres, a few millimetres apart as one surface seen twice would be.
  DepthWindow window(2);
  window.push({1.000F, 2.000F});
  window.push({1.010F, 2.006F});
  const auto mean = window.mean();
  ASSERT_EQ(mean.size(), 2U);
  EXPECT_FLOAT_EQ(mean[0], 1.005F);
  EXPECT_FLOAT_EQ(mean[1], 2.003F);
}

TEST(DepthWindow, IsFullOnlyAtCapacityAndSlidesOverTheOldestFrame)
{
  DepthWindow window(2);
  EXPECT_FALSE(window.full());
  window.push({1.000F});
  EXPECT_FALSE(window.full());
  window.push({1.002F});
  EXPECT_TRUE(window.full());
  window.push({1.004F});  // pushes out the 1.000
  EXPECT_EQ(window.size(), 2U);
  EXPECT_FLOAT_EQ(window.mean()[0], 1.003F);
}

TEST(DepthWindow, InvalidPixelsAreLeftOutOfTheMeanNotCountedAsZero)
{
  DepthWindow window(2, 0.5);
  window.push({1.000F, kNaN, 1.010F});
  window.push({1.004F, 1.002F, kNaN});
  const auto mean = window.mean();
  EXPECT_FLOAT_EQ(mean[0], 1.002F);  // valid in both frames
  EXPECT_FLOAT_EQ(mean[1], 1.002F);  // valid in 1 of 2 frames: 1 >= 0.5 * 2, mean of that one
  EXPECT_FLOAT_EQ(mean[2], 1.010F);
}

TEST(DepthWindow, AFlickeringPixelBelowTheValidFractionBecomesNaN)
{
  DepthWindow window(4, 0.75);  // needs 3 valid frames of 4
  window.push({1.0F, 1.0F});
  window.push({1.0F, kNaN});
  window.push({1.0F, kNaN});
  window.push({1.0F, 1.0F});
  const auto mean = window.mean();
  EXPECT_FLOAT_EQ(mean[0], 1.0F);
  EXPECT_TRUE(std::isnan(mean[1]));  // valid in only 2 of 4
}

TEST(DepthWindow, ClearEmptiesItAndTheMeanOfNothingIsNothing)
{
  DepthWindow window(2);
  EXPECT_TRUE(window.mean().empty());
  window.push({1.0F});
  window.clear();
  EXPECT_EQ(window.size(), 0U);
  EXPECT_FALSE(window.full());
  EXPECT_TRUE(window.mean().empty());
}

TEST(DepthWindow, APixelThatSwitchesBetweenTwoSurfacesIsNotAveragedIntoAFlyingPixel)
{
  // The Stage 5 online case: a box-edge pixel sees the box (0.947 m) in nine frames and the
  // table behind it (0.998 m) in one. The mean, 0.952 m, is a depth nothing is at.
  DepthWindow tolerant(10, 0.5, std::numeric_limits<double>::infinity());
  DepthWindow window(10);
  for (int frame = 0; frame < 10; ++frame) {
    const std::vector<float> pixels{frame == 0 ? 0.998F : 0.947F, 0.947F};
    tolerant.push(pixels);
    window.push(pixels);
  }
  EXPECT_NEAR(tolerant.mean()[0], 0.952F, 1e-3F);  // what the spread rule exists to prevent
  EXPECT_TRUE(std::isnan(window.mean()[0]));
  EXPECT_FLOAT_EQ(window.mean()[1], 0.947F);       // the steady pixel beside it is untouched
}

TEST(DepthWindow, TheSpreadLimitIsAboutTheLargestMinusTheSmallestValidDepth)
{
  DepthWindow window(2, 0.5, 0.02);
  window.push({1.000F, 1.000F});
  window.push({1.015F, 1.025F});  // spreads of 15 mm and 25 mm
  const auto mean = window.mean();
  EXPECT_FLOAT_EQ(mean[0], 1.0075F);
  EXPECT_TRUE(std::isnan(mean[1]));
}

TEST(DepthWindow, DepthNoiseOfTheSizeTheProtocolUsedNeverReachesTheSpreadLimit)
{
  // Alternating +-8 mm around the true depth is a 16 mm spread, below the 20 mm limit and about
  // what independent noise of 4 mm standard deviation spans over ten frames: the pixel survives
  // and its mean is exact.
  DepthWindow window(10);
  for (int frame = 0; frame < 10; ++frame) {
    window.push({1.0F + ((frame % 2 == 0) ? 0.008F : -0.008F)});
  }
  EXPECT_NEAR(window.mean()[0], 1.0F, 1e-6F);
}

TEST(DepthWindow, InvalidFramesDoNotCountTowardsTheSpread)
{
  // Zero is "no measurement", not a depth 1 m away from the others.
  DepthWindow window(3, 0.5);
  window.push({1.0F});
  window.push({0.0F});
  window.push({1.004F});
  EXPECT_NEAR(window.mean()[0], 1.002F, 1e-6F);
}

}  // namespace
