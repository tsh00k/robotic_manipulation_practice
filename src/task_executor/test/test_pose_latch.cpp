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
#include <string>

#include "task_executor/pose_latch.hpp"

namespace task_executor
{
namespace
{

constexpr double kDeg = M_PI / 180.0;

LatchSample sample(uint64_t sequence, double x = 0.5, double y = 0.1, double yaw = 0.2)
{
  LatchSample result;
  result.bridge_session = 7;
  result.generation = 3;
  result.sequence = sequence;
  result.stamp_s = 0.1 * static_cast<double>(sequence);
  result.measured = true;
  result.x = x;
  result.y = y;
  result.z = 0.24;
  result.yaw_rad = yaw;
  return result;
}

LatchSample unmeasured(uint64_t sequence, const std::string & reason)
{
  LatchSample result = sample(sequence);
  result.measured = false;
  result.reason = reason;
  return result;
}

PoseLatch boundLatch(LatchParams params = {})
{
  PoseLatch latch(params);
  latch.reset(7, 3);
  return latch;
}

TEST(PoseLatch, LatchesTheNewestOfFiveConsistentEstimatesAndKeepsIt)
{
  auto latch = boundLatch();
  for (uint64_t i = 1; i <= 4; ++i) {
    latch.offer(sample(i, 0.5 + 0.0005 * static_cast<double>(i)));
    EXPECT_FALSE(latch.latched()) << "after " << i;
  }
  EXPECT_EQ(latch.status(), "WAITING:COLLECTING 4/5");
  latch.offer(sample(5, 0.5025));
  ASSERT_TRUE(latch.latched());
  EXPECT_EQ(latch.status(), "LATCHED");
  EXPECT_EQ(latch.pose()->sequence, 5U);
  EXPECT_DOUBLE_EQ(latch.pose()->x, 0.5025);  // the newest, not an average
  EXPECT_NEAR(latch.pose()->position_spread_m, 0.002, 1e-12);
  // Immutable until the next reset: a later, different estimate changes nothing.
  latch.offer(sample(6, 0.60));
  EXPECT_DOUBLE_EQ(latch.pose()->x, 0.5025);
  latch.reset(7, 4);
  EXPECT_FALSE(latch.latched());
}

TEST(PoseLatch, AnInconsistentOrUnmeasuredEstimateMakesTheRunStartAgain)
{
  auto latch = boundLatch();
  for (uint64_t i = 1; i <= 4; ++i) {
    latch.offer(sample(i));
  }
  latch.offer(sample(5, 0.5 + 0.005));  // 5 mm away from the others: more than the 3 mm allowed
  EXPECT_FALSE(latch.latched());
  EXPECT_EQ(latch.status().rfind("WAITING:INCONSISTENT 5.0 mm", 0), 0U) << latch.status();
  // The odd one stays in the last five until it has slid out.
  for (uint64_t i = 6; i <= 9; ++i) {
    latch.offer(sample(i));
    EXPECT_FALSE(latch.latched()) << "sample " << i;
  }
  latch.offer(sample(10));
  EXPECT_TRUE(latch.latched());

  auto other = boundLatch();
  for (uint64_t i = 1; i <= 4; ++i) {
    other.offer(sample(i));
  }
  other.offer(unmeasured(5, "NO_RECTANGLE_MATCHES_BIN"));
  EXPECT_EQ(other.status(), "WAITING:NO_RECTANGLE_MATCHES_BIN");
  for (uint64_t i = 6; i <= 9; ++i) {
    other.offer(sample(i));
    EXPECT_FALSE(other.latched()) << "five consecutive are needed again, sample " << i;
  }
  other.offer(sample(10));
  EXPECT_TRUE(other.latched());
}

TEST(PoseLatch, SamplesOfAnotherLifecycleOrAnOldSequenceAreIgnoredNotCounted)
{
  auto latch = boundLatch();
  for (uint64_t i = 1; i <= 3; ++i) {
    latch.offer(sample(i));
  }
  LatchSample old_generation = sample(100);
  old_generation.generation = 2;
  latch.offer(old_generation);
  LatchSample other_session = sample(101);
  other_session.bridge_session = 8;
  latch.offer(other_session);
  latch.offer(sample(2));  // sequence does not increase
  EXPECT_EQ(latch.status(), "WAITING:COLLECTING 3/5");  // none of the three counted or interrupted
  latch.offer(sample(4));
  latch.offer(sample(5));
  EXPECT_TRUE(latch.latched());
  EXPECT_EQ(latch.pose()->sequence, 5U);

  PoseLatch unbound(LatchParams{});
  for (uint64_t i = 1; i <= 6; ++i) {
    unbound.offer(sample(i));
  }
  EXPECT_FALSE(unbound.latched()) << "nothing is latched before reset() names a lifecycle";
}

TEST(PoseLatch, YawIsComparedAroundTheObjectsPeriodNotAsPlainNumbers)
{
  // The square box repeats every 90 degrees: -44 and +44 degrees are 2 degrees apart.
  LatchParams box;
  box.yaw_period_rad = M_PI / 2.0;
  auto latch = boundLatch(box);
  const double yaws[] = {-44.0, 44.0, -44.5, 44.5, -44.0};
  uint64_t sequence = 0;
  for (const double yaw : yaws) {
    latch.offer(sample(++sequence, 0.5, 0.1, yaw * kDeg));
  }
  ASSERT_TRUE(latch.latched()) << latch.status();
  // Modulo 90 degrees they are 46, 44, 45.5, 44.5 and 46: the smallest arc is 44..46.
  EXPECT_NEAR(latch.pose()->yaw_spread_rad / kDeg, 2.0, 1e-6);

  // The same two angles are far apart for the bin, whose period is 180 degrees.
  LatchParams bin;
  bin.yaw_period_rad = M_PI;
  auto bin_latch = boundLatch(bin);
  sequence = 0;
  for (const double yaw : yaws) {
    bin_latch.offer(sample(++sequence, 0.5, 0.1, yaw * kDeg));
  }
  EXPECT_FALSE(bin_latch.latched());
}

}  // namespace
}  // namespace task_executor
