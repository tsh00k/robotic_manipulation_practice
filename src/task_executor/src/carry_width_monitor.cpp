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

#include "task_executor/carry_width_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>

namespace task_executor
{

CarryWidthMonitor::CarryWidthMonitor(CarryWidthParams params)
: params_(std::move(params)),
  min_width_m_(std::numeric_limits<double>::quiet_NaN()),
  max_width_m_(std::numeric_limits<double>::quiet_NaN())
{
}

void CarryWidthMonitor::update(Phase phase, bool attached, double width_m, double sim_time_s)
{
  const bool carrying = attached &&
    (phase == Phase::kLift || phase == Phase::kPreplace || phase == Phase::kPlace);
  if (!carrying || !std::isfinite(width_m)) {
    low_since_s_.reset();
    return;
  }
  min_width_m_ = std::isnan(min_width_m_) ? width_m : std::min(min_width_m_, width_m);
  max_width_m_ = std::isnan(max_width_m_) ? width_m : std::max(max_width_m_, width_m);
  if (width_m >= params_.min_width_m) {
    low_since_s_.reset();
    return;
  }
  if (!low_since_s_) {
    low_since_s_ = sim_time_s;
    lowest_in_run_m_ = width_m;
  }
  lowest_in_run_m_ = std::min(lowest_in_run_m_, width_m);
  const double duration = sim_time_s - *low_since_s_;
  // 1 ns of slack: the stamps are whole nanoseconds turned into seconds, and 1.2 - 1.0 can come
  // out just under 0.2.
  if (!alert_ && duration >= params_.min_duration_s - 1e-9) {
    char text[128];
    std::snprintf(
      text, sizeof(text), "CARRY_WIDTH_LOW: %.1f mm < %.1f mm for %.2f s in %s",
      lowest_in_run_m_ * 1000.0, params_.min_width_m * 1000.0, duration, phaseName(phase));
    alert_ = text;
    alert_time_s_ = sim_time_s;
  }
}

}  // namespace task_executor
