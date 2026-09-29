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

#include "mujoco_bridge/camera_geometry.hpp"

TEST(CameraGeometry, DepthBufferToMetres)
{
  EXPECT_DOUBLE_EQ(mujoco_bridge::metricDepth(0.0, 0.01, 10.0), 0.01);
  EXPECT_NEAR(mujoco_bridge::metricDepth(0.5, 0.01, 10.0), 0.01998, 1e-5);
  EXPECT_TRUE(std::isnan(mujoco_bridge::metricDepth(1.0, 0.01, 10.0)));
}

TEST(CameraGeometry, OpticalProjection)
{
  const auto point = mujoco_bridge::backproject(159.5, 119.5, 0.61, 257.35, 257.35, 159.5, 119.5);
  EXPECT_DOUBLE_EQ(point.x, 0.0);
  EXPECT_DOUBLE_EQ(point.y, 0.0);
  EXPECT_DOUBLE_EQ(point.z, 0.61);
  const auto right = mujoco_bridge::backproject(169.5, 119.5, 0.61, 257.35, 257.35, 159.5, 119.5);
  EXPECT_GT(right.x, 0.0);
}
