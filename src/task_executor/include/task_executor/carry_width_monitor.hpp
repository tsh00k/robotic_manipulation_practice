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

#include <optional>
#include <string>

#include "task_executor/phase.hpp"

namespace task_executor
{

struct CarryWidthParams
{
  // Below this the fingers have closed past the box (40 mm; 38.2..38.8 mm seen while carrying).
  double min_width_m = 0.034;
  // For this long in simulated time, so that one noisy sample is not an alert.
  double min_duration_s = 0.2;
};

// Watches the gripper opening while the box is carried (Week 4.1 Stage 12). The bridge keeps
// its attachment latched unless the gripper OPENS past 50 mm; a box that slips out lets the
// fingers close instead, which nothing else notices. Pure rule, no ROS. One monitor per
// attempt: the caller makes a new one at every reset.
class CarryWidthMonitor
{
public:
  explicit CarryWidthMonitor(CarryWidthParams params = {});

  // One bridge sample. Counts only while attached and in LIFT, PREPLACE or PLACE.
  void update(Phase phase, bool attached, double width_m, double sim_time_s);

  // The first alert of this attempt, e.g. "CARRY_WIDTH_LOW: 2.1 mm < 34.0 mm for 0.20 s in
  // PREPLACE", and the simulated time at which it was raised.
  const std::optional<std::string> & alert() const {return alert_;}
  double alertTime() const {return alert_time_s_;}

  // Smallest and largest opening seen while carrying; NaN before the first such sample.
  double minWidth() const {return min_width_m_;}
  double maxWidth() const {return max_width_m_;}

private:
  CarryWidthParams params_;
  std::optional<double> low_since_s_;
  double lowest_in_run_m_ = 0.0;
  std::optional<std::string> alert_;
  double alert_time_s_ = 0.0;
  double min_width_m_;
  double max_width_m_;
};

}  // namespace task_executor
