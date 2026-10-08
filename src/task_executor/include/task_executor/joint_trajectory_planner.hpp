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

#include <array>
#include <vector>

namespace task_executor
{

// Per-joint limits for the arm's reference trajectory (joint1..joint7).
struct JointLimits
{
  std::array<double, 7> max_velocity{};
  std::array<double, 7> max_acceleration{};
};

// Franka Emika Robot (Panda) limits from the FCI documentation ("Control Interface
// Specification and Robot Limits"): velocity 95 % of the given 2.175 / 2.61 rad/s, acceleration
// a quarter of the given 15, 7.5, 10, 12.5, 15, 20, 20 rad/s^2 -- the values MoveIt's own Panda
// configuration uses. The 5 % keeps the recorded actions clear of the dataset contract's
// per-frame limit (velocity limit x 0.1 s): at 100 % a cruising joint changed by exactly the
// limit, so a converter's strict comparison would reject frames on rounding (week5 Stage 6).
// With a quarter, TOTG's piecewise-constant acceleration may switch from -max to +max within
// one 1 ms control cycle without exceeding the FCI jerk limits (7500 .. 10000 rad/s^3), so no
// separate jerk limiting is needed (Week 5 Stage 3, ADR 020).
inline constexpr JointLimits kPandaReferenceLimits{
  {2.06625, 2.06625, 2.06625, 2.06625, 2.4795, 2.4795, 2.4795},
  {3.75, 1.875, 2.5, 3.125, 3.75, 5.0, 5.0}};

// A timed joint trajectory, sampled: the reference positions and velocities at times_s (from 0,
// increasing). The last sample is the goal, at rest. A single sample means no motion.
struct JointTrajectoryPlan
{
  std::vector<double> times_s;
  std::vector<std::array<double, 7>> positions;
  std::vector<std::array<double, 7>> velocities;
  double duration_s() const {return times_s.empty() ? 0.0 : times_s.back();}
};

// The time-optimal trajectory from start to goal along the straight line in joint space,
// starting and ending at rest, within the limits (MoveIt's TOTG). Sampled every
// sample_period_s plus the end; the default 1 ms is the real robot's control cycle, and the
// bridge interpolates linearly between samples, so the reference keeps TOTG's velocity and
// acceleration only if the samples are that dense (week5 Stage 3). Throws
// std::invalid_argument for non-finite input or limits that are not positive,
// std::runtime_error if TOTG fails.
JointTrajectoryPlan planJointLine(
  const std::array<double, 7> & start, const std::array<double, 7> & goal,
  const JointLimits & limits = kPandaReferenceLimits, double sample_period_s = 0.001);

// The same through a dense sequence of joint waypoints (a TCP straight line solved point by
// point, DiffIkWaypointSource::cartesianLine). TOTG must round the small corners between
// consecutive waypoints or it would stop at each of them: max_deviation_rad bounds how far the
// rounded path may leave a waypoint. The plan ends exactly at the last waypoint, at rest.
JointTrajectoryPlan planJointPath(
  const std::vector<std::array<double, 7>> & waypoints,
  const JointLimits & limits = kPandaReferenceLimits, double sample_period_s = 0.001,
  double max_deviation_rad = 0.0);

}  // namespace task_executor
