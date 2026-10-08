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

#include <cstddef>
#include <vector>

namespace mujoco_bridge
{

// Executes a timed joint trajectory (Week 5 Stage 3, ADR 020): the reference position at a time
// since the trajectory started, for the position servos' ctrl. The executor samples its TOTG
// trajectory every 1 ms (the real robot's control cycle); between two samples this interpolates
// linearly, so at every physics step (2 ms) the reference is one of those samples and keeps their
// velocity and acceleration. A cubic Hermite through positions and velocities was tried first and
// overshot the acceleration limit by up to 24 % where TOTG switches from accelerating to braking
// (week5 Stage 3). Before the first sample it returns the first; after the last, the last. No
// MuJoCo and no ROS: the node owns the clock and writes the result into ctrl.
class TrajectoryInterpolator
{
public:
  // times_s strictly increasing, starting at 0; positions[k] all the same size and finite.
  // Throws std::invalid_argument otherwise. Replaces any earlier trajectory.
  void set(std::vector<double> times_s, std::vector<std::vector<double>> positions);
  void clear();
  bool active() const {return !times_s_.empty();}
  double duration() const {return times_s_.empty() ? 0.0 : times_s_.back();}
  // The reference at time t since the start, into q (resized). False when no trajectory is set.
  bool sample(double t, std::vector<double> & q) const;

private:
  std::vector<double> times_s_;
  std::vector<std::vector<double>> positions_;
};

}  // namespace mujoco_bridge
