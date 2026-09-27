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

#include "task_executor/episode_telemetry.hpp"

namespace task_executor
{
namespace
{

TEST(EpisodeTelemetry, SerializesCompletedAndFailedPhases)
{
  EpisodeTelemetry telemetry;
  PhaseTelemetry completed;
  completed.phase_name = "HOME";
  completed.duration_s = 1.25;
  completed.target_tcp_x_m = 0.5;
  completed.ik_position_error_m = 0.001;
  completed.joint_tracking_error_rad = 0.02;
  telemetry.append(completed);

  PhaseTelemetry failed;
  failed.phase_name = "PREGRASP";
  failed.duration_s = 0.3;
  telemetry.append(failed);

  manipulation_interfaces::msg::EpisodeOutcome outcome;
  telemetry.appendTo(outcome);
  EXPECT_EQ(outcome.phase_names, (std::vector<std::string>{"HOME", "PREGRASP"}));
  EXPECT_EQ(outcome.phase_durations_s, (std::vector<double>{1.25, 0.3}));
  EXPECT_EQ(outcome.target_tcp_x_m.size(), 2u);
  EXPECT_EQ(outcome.target_tcp_y_m.size(), 2u);
  EXPECT_EQ(outcome.target_tcp_z_m.size(), 2u);
  EXPECT_EQ(outcome.ik_position_error_m.size(), 2u);
  EXPECT_EQ(outcome.joint_tracking_error_rad.size(), 2u);
  EXPECT_EQ(outcome.actual_tcp_position_error_m.size(), 2u);
  EXPECT_DOUBLE_EQ(outcome.target_tcp_x_m.front(), 0.5);
  EXPECT_DOUBLE_EQ(outcome.ik_position_error_m.front(), 0.001);
  EXPECT_TRUE(std::isnan(outcome.target_tcp_x_m.back()));
  EXPECT_TRUE(std::isnan(outcome.ik_position_error_m.back()));
  EXPECT_TRUE(std::isnan(outcome.joint_tracking_error_rad.back()));

  telemetry.clear();
  EXPECT_TRUE(telemetry.phases.empty());
}

}  // namespace
}  // namespace task_executor
