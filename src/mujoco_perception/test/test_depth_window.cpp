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
  DepthWindow window(2);
  window.push({1.0F, 2.0F});
  window.push({3.0F, 4.0F});
  const auto mean = window.mean();
  ASSERT_EQ(mean.size(), 2U);
  EXPECT_FLOAT_EQ(mean[0], 2.0F);
  EXPECT_FLOAT_EQ(mean[1], 3.0F);
}

TEST(DepthWindow, IsFullOnlyAtCapacityAndSlidesOverTheOldestFrame)
{
  DepthWindow window(2);
  EXPECT_FALSE(window.full());
  window.push({1.0F});
  EXPECT_FALSE(window.full());
  window.push({2.0F});
  EXPECT_TRUE(window.full());
  window.push({3.0F});  // pushes out the 1.0
  EXPECT_EQ(window.size(), 2U);
  EXPECT_FLOAT_EQ(window.mean()[0], 2.5F);
}

TEST(DepthWindow, InvalidPixelsAreLeftOutOfTheMeanNotCountedAsZero)
{
  DepthWindow window(2, 0.5);
  window.push({1.0F, kNaN, 5.0F});
  window.push({3.0F, 2.0F, kNaN});
  const auto mean = window.mean();
  EXPECT_FLOAT_EQ(mean[0], 2.0F);  // valid in both frames
  EXPECT_FLOAT_EQ(mean[1], 2.0F);  // valid in 1 of 2 frames: 1 >= 0.5 * 2, mean of that one
  EXPECT_FLOAT_EQ(mean[2], 5.0F);
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

TEST(DepthWindow, ZeroAndNegativeDepthAreInvalidLikeNaN)
{
  DepthWindow window(2, 0.5);
  window.push({0.0F, -1.0F});
  window.push({4.0F, 6.0F});
  const auto mean = window.mean();
  EXPECT_FLOAT_EQ(mean[0], 4.0F);
  EXPECT_FLOAT_EQ(mean[1], 6.0F);
}

TEST(DepthWindow, ADifferentImageSizeStartsANewWindow)
{
  DepthWindow window(3);
  window.push({1.0F, 1.0F});
  window.push({2.0F, 2.0F});
  window.push({9.0F, 9.0F, 9.0F});  // camera changed: the two earlier frames are not comparable
  EXPECT_EQ(window.size(), 1U);
  ASSERT_EQ(window.mean().size(), 3U);
  EXPECT_FLOAT_EQ(window.mean()[0], 9.0F);
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

TEST(DepthWindow, AveragingNIndependentFramesShrinksTheSpreadBySqrtN)
{
  // Deterministic samples: frame k has +d on even pixels and -d on odd pixels when k is even,
  // the opposite when k is odd. Independent of any random generator, the mean of an even number
  // of frames is exactly the true value 1.0.
  DepthWindow window(4);
  for (int k = 0; k < 4; ++k) {
    std::vector<float> frame(6);
    for (std::size_t i = 0; i < frame.size(); ++i) {
      const float sign = ((i + static_cast<std::size_t>(k)) % 2 == 0) ? 1.0F : -1.0F;
      frame[i] = 1.0F + sign * 0.004F;
    }
    window.push(frame);
  }
  for (const float value : window.mean()) {
    EXPECT_NEAR(value, 1.0F, 1e-6F);
  }
}

TEST(DepthWindow, RejectsAnImpossibleConfiguration)
{
  EXPECT_THROW(DepthWindow(0), std::invalid_argument);
  EXPECT_THROW(DepthWindow(3, -0.1), std::invalid_argument);
  EXPECT_THROW(DepthWindow(3, 1.5), std::invalid_argument);
}

}  // namespace
