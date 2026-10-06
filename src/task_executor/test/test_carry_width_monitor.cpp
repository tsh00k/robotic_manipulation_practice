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

#include "task_executor/carry_width_monitor.hpp"

namespace task_executor
{
namespace
{

// Feed a constant width from t0 to t1 at the bridge's 100 Hz.
void hold(
  CarryWidthMonitor & monitor, Phase phase, bool attached, double width, double t0,
  double t1)
{
  // Times from an integer step count: summing 0.01 drifts and moves a 0.2 s boundary.
  const int steps = static_cast<int>(std::lround((t1 - t0) / 0.01));
  for (int i = 0; i < steps; ++i) {
    monitor.update(phase, attached, width, t0 + 0.01 * i);
  }
}

TEST(CarryWidthMonitor, ANormalCarryRaisesNothingAndRecordsItsRange)
{
  CarryWidthMonitor monitor;
  hold(monitor, Phase::kLift, true, 0.0385, 0.0, 1.0);
  hold(monitor, Phase::kPreplace, true, 0.0392, 1.0, 2.0);
  EXPECT_FALSE(monitor.alert().has_value());
  EXPECT_DOUBLE_EQ(monitor.minWidth(), 0.0385);
  EXPECT_DOUBLE_EQ(monitor.maxWidth(), 0.0392);
}

TEST(CarryWidthMonitor, ClosingPastTheBoxForTheDurationRaisesOneAlert)
{
  CarryWidthMonitor monitor;
  hold(monitor, Phase::kLift, true, 0.0385, 0.0, 1.0);
  hold(monitor, Phase::kPreplace, true, 0.002, 1.0, 1.15);  // 0.15 s: not yet
  EXPECT_FALSE(monitor.alert().has_value());
  hold(monitor, Phase::kPreplace, true, 0.002, 1.15, 1.5);
  ASSERT_TRUE(monitor.alert().has_value());
  EXPECT_EQ(
    monitor.alert()->rfind("CARRY_WIDTH_LOW: 2.0 mm < 34.0 mm for 0.20 s in PREPLACE", 0),
    0U) << *monitor.alert();
  EXPECT_NEAR(monitor.alertTime(), 1.2, 0.011);
  const auto first = *monitor.alert();
  hold(monitor, Phase::kPlace, true, 0.001, 1.5, 3.0);
  EXPECT_EQ(*monitor.alert(), first) << "one alert per attempt";
  EXPECT_DOUBLE_EQ(monitor.minWidth(), 0.001);
}

TEST(CarryWidthMonitor, AShortDipThatRecoversDoesNotCount)
{
  CarryWidthMonitor monitor;
  for (int i = 0; i < 5; ++i) {
    hold(monitor, Phase::kPreplace, true, 0.030, i * 0.3, i * 0.3 + 0.15);
    hold(monitor, Phase::kPreplace, true, 0.0385, i * 0.3 + 0.15, i * 0.3 + 0.3);
  }
  EXPECT_FALSE(monitor.alert().has_value());
}

TEST(CarryWidthMonitor, OnlyCarryingIsWatched)
{
  CarryWidthMonitor monitor;
  // Closing on nothing before the attach, not attached during LIFT, or in another phase.
  hold(monitor, Phase::kClose, true, 0.0, 0.0, 1.0);
  hold(monitor, Phase::kLift, false, 0.0, 1.0, 2.0);
  hold(monitor, Phase::kOpen, true, 0.0, 2.0, 3.0);
  EXPECT_FALSE(monitor.alert().has_value());
  EXPECT_TRUE(std::isnan(monitor.minWidth()));
}

}  // namespace
}  // namespace task_executor
