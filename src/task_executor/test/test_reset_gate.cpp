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

#include "task_executor/reset_gate.hpp"

namespace task_executor
{
namespace
{

TEST(ResetGate, RejectsOldAndDelayedSamples)
{
  ResetGate gate;
  const auto now = ResetGate::TimePoint{};
  const auto request = gate.begin(now);
  ASSERT_TRUE(gate.markRequestSent());
  EXPECT_FALSE(gate.accept(7, 8, 20, now));
  ASSERT_TRUE(gate.onResetResponse(request, true, 7, 9));
  EXPECT_FALSE(gate.accept(7, 8, 21, now));
  EXPECT_TRUE(gate.accept(7, 9, 22, now));
  EXPECT_EQ(gate.state(), ResetGate::State::kReady);
  EXPECT_FALSE(gate.accept(7, 8, 23, now));
  EXPECT_FALSE(gate.accept(7, 9, 22, now));
  EXPECT_TRUE(gate.accept(7, 9, 23, now));
}

TEST(ResetGate, UnexpectedNewResetFailsCurrentEpisode)
{
  ResetGate gate;
  const auto request = gate.begin(ResetGate::TimePoint{});
  ASSERT_TRUE(gate.markRequestSent());
  ASSERT_TRUE(gate.onResetResponse(request, true, 7, 9));
  ASSERT_TRUE(gate.accept(7, 9, 20, ResetGate::TimePoint{}));
  EXPECT_FALSE(gate.accept(7, 10, 21, ResetGate::TimePoint{}));
  EXPECT_EQ(gate.state(), ResetGate::State::kFailed);
  EXPECT_STREQ(gate.failureCode(), "RESET_SUPERSEDED");
}

TEST(ResetGate, ConsecutiveAndRetryResetsInvalidatePreviousGenerations)
{
  ResetGate gate;
  const auto now = ResetGate::TimePoint{};
  const auto first = gate.begin(now);
  ASSERT_TRUE(gate.markRequestSent());
  const auto second = gate.begin(now);
  ASSERT_TRUE(gate.markRequestSent());
  EXPECT_FALSE(gate.onResetResponse(first, true, 7, 12));
  ASSERT_TRUE(gate.onResetResponse(second, true, 7, 13));
  EXPECT_FALSE(gate.accept(7, 12, 20, now));
  EXPECT_TRUE(gate.accept(7, 13, 21, now));
  const auto retry = gate.begin(now);
  ASSERT_TRUE(gate.markRequestSent());
  ASSERT_TRUE(gate.onResetResponse(retry, true, 7, 14));
  EXPECT_FALSE(gate.accept(7, 13, 22, now));
  EXPECT_TRUE(gate.accept(7, 14, 23, now));
}

TEST(ResetGate, ReportsMissingFreshObservation)
{
  ResetGate gate;
  const auto now = ResetGate::TimePoint{};
  const auto request = gate.begin(now);
  ASSERT_TRUE(gate.markRequestSent());
  ASSERT_TRUE(gate.onResetResponse(request, true, 7, 3));
  EXPECT_FALSE(gate.checkTimeout(now + std::chrono::seconds(4), std::chrono::seconds(5)));
  EXPECT_TRUE(gate.checkTimeout(now + std::chrono::seconds(5), std::chrono::seconds(5)));
  EXPECT_STREQ(gate.failureCode(), "OBSERVATION_STALE");
}

TEST(ResetGate, ReportsStoppedObservationStreamAfterReady)
{
  ResetGate gate;
  const auto now = ResetGate::TimePoint{};
  const auto request = gate.begin(now);
  ASSERT_TRUE(gate.markRequestSent());
  ASSERT_TRUE(gate.onResetResponse(request, true, 7, 3));
  ASSERT_TRUE(gate.accept(7, 3, 10, now));
  EXPECT_FALSE(gate.accept(7, 3, 10, now + std::chrono::seconds(4)));
  EXPECT_TRUE(gate.checkTimeout(now + std::chrono::seconds(5), std::chrono::seconds(5)));
  EXPECT_STREQ(gate.failureCode(), "OBSERVATION_STALE");
}

TEST(ResetGate, ReportsFailedOrUnavailableReset)
{
  ResetGate gate;
  const auto now = ResetGate::TimePoint{};
  const auto request = gate.begin(now);
  ASSERT_TRUE(gate.markRequestSent());
  ASSERT_TRUE(gate.onResetResponse(request, false, 7, 0));
  EXPECT_STREQ(gate.failureCode(), "RESET_FAILED");
  gate.begin(now);
  EXPECT_TRUE(gate.checkTimeout(now + std::chrono::seconds(5), std::chrono::seconds(5)));
  EXPECT_STREQ(gate.failureCode(), "RESET_UNAVAILABLE");
}

TEST(ResetGate, RejectsPreviousBridgeSessionWithSameGeneration)
{
  ResetGate gate;
  const auto now = ResetGate::TimePoint{};
  const auto request = gate.begin(now);
  ASSERT_TRUE(gate.markRequestSent());
  ASSERT_TRUE(gate.onResetResponse(request, true, 8, 1));
  EXPECT_FALSE(gate.accept(7, 1, 100, now));
  EXPECT_EQ(gate.state(), ResetGate::State::kAwaitingObservation);
  EXPECT_TRUE(gate.accept(8, 1, 1, now));
}

}  // namespace
}  // namespace task_executor
