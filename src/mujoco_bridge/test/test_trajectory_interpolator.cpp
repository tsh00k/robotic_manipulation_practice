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
#include <stdexcept>
#include <vector>

#include "mujoco_bridge/trajectory_interpolator.hpp"

namespace mujoco_bridge
{
namespace
{

TEST(TrajectoryInterpolator, IsExactAtSamplesAndLinearBetween)
{
  TrajectoryInterpolator trajectory;
  trajectory.set({0.0, 0.001, 0.002, 0.003}, {{0.0, 1.0}, {0.1, 1.0}, {0.3, 0.5}, {0.6, 0.0}});
  std::vector<double> q;
  ASSERT_TRUE(trajectory.sample(0.002, q));
  EXPECT_DOUBLE_EQ(q[0], 0.3);
  EXPECT_DOUBLE_EQ(q[1], 0.5);
  ASSERT_TRUE(trajectory.sample(0.0015, q));
  EXPECT_NEAR(q[0], 0.2, 1e-12);
  EXPECT_NEAR(q[1], 0.75, 1e-12);
  EXPECT_DOUBLE_EQ(trajectory.duration(), 0.003);
}

TEST(TrajectoryInterpolator, HoldsTheEndsOutsideTheTrajectory)
{
  TrajectoryInterpolator trajectory;
  trajectory.set({0.0, 0.5}, {{0.0}, {1.0}});
  std::vector<double> q;
  ASSERT_TRUE(trajectory.sample(-1.0, q));
  EXPECT_DOUBLE_EQ(q[0], 0.0);
  ASSERT_TRUE(trajectory.sample(0.5, q));
  EXPECT_DOUBLE_EQ(q[0], 1.0);
  ASSERT_TRUE(trajectory.sample(7.0, q));
  EXPECT_DOUBLE_EQ(q[0], 1.0);
}

TEST(TrajectoryInterpolator, IsInactiveUntilSetAndAfterClear)
{
  TrajectoryInterpolator trajectory;
  std::vector<double> q;
  EXPECT_FALSE(trajectory.active());
  EXPECT_FALSE(trajectory.sample(0.1, q));
  trajectory.set({0.0, 0.1}, {{0.0}, {1.0}});
  EXPECT_TRUE(trajectory.active());
  trajectory.clear();
  EXPECT_FALSE(trajectory.sample(0.1, q));
}

TEST(TrajectoryInterpolator, RejectsMalformedTrajectories)
{
  TrajectoryInterpolator trajectory;
  // not starting at 0, not increasing, ragged, a missing sample, non-finite
  EXPECT_THROW(trajectory.set({0.1, 0.2}, {{0.0}, {1.0}}), std::invalid_argument);
  EXPECT_THROW(trajectory.set({0.0, 0.0}, {{0.0}, {1.0}}), std::invalid_argument);
  EXPECT_THROW(trajectory.set({0.0, 0.1}, {{0.0}, {1.0, 2.0}}), std::invalid_argument);
  EXPECT_THROW(trajectory.set({0.0, 0.1}, {{0.0}}), std::invalid_argument);
  EXPECT_THROW(trajectory.set({0.0, 0.1}, {{0.0}, {std::nan("")}}), std::invalid_argument);
  EXPECT_FALSE(trajectory.active());
}

}  // namespace
}  // namespace mujoco_bridge
