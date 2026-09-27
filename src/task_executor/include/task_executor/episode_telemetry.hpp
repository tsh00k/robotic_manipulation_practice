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

#pragma once

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <manipulation_interfaces/msg/episode_outcome.hpp>

namespace task_executor
{

struct PhaseTelemetry
{
  std::string phase_name;
  double duration_s = 0.0;
  double target_tcp_x_m = std::numeric_limits<double>::quiet_NaN();
  double target_tcp_y_m = std::numeric_limits<double>::quiet_NaN();
  double target_tcp_z_m = std::numeric_limits<double>::quiet_NaN();
  double ik_position_error_m = std::numeric_limits<double>::quiet_NaN();
  double joint_tracking_error_rad = std::numeric_limits<double>::quiet_NaN();
  double actual_tcp_position_error_m = std::numeric_limits<double>::quiet_NaN();
};

struct EpisodeTelemetry
{
  std::vector<PhaseTelemetry> phases;

  void clear() {phases.clear();}
  void append(PhaseTelemetry sample) {phases.push_back(std::move(sample));}
  void appendTo(manipulation_interfaces::msg::EpisodeOutcome & outcome) const;
};

}  // namespace task_executor
