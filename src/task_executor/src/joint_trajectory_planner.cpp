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

#include "task_executor/joint_trajectory_planner.hpp"

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <cmath>
#include <list>
#include <stdexcept>

namespace task_executor
{
namespace
{
// TOTG integrates the path velocity with this step; it is not the sample period.
constexpr double kIntegrationStepS = 0.001;
// Joint changes below this are no motion (a single sample).
constexpr double kNoMotionRad = 1e-9;

Eigen::VectorXd toEigen(const std::array<double, 7> & values)
{
  Eigen::VectorXd vector(7);
  for (int i = 0; i < 7; ++i) {
    vector(i) = values[static_cast<std::size_t>(i)];
  }
  return vector;
}

std::array<double, 7> toArray(const Eigen::VectorXd & vector)
{
  std::array<double, 7> values{};
  for (int i = 0; i < 7; ++i) {
    values[static_cast<std::size_t>(i)] = vector(i);
  }
  return values;
}
}  // namespace

JointTrajectoryPlan planJointLine(
  const std::array<double, 7> & start, const std::array<double, 7> & goal,
  const JointLimits & limits, double sample_period_s)
{
  // Two waypoints, no blending: the path is exactly the line.
  return planJointPath({start, goal}, limits, sample_period_s, 0.0);
}

JointTrajectoryPlan planJointPath(
  const std::vector<std::array<double, 7>> & waypoints, const JointLimits & limits,
  double sample_period_s, double max_deviation_rad)
{
  for (std::size_t i = 0; i < 7; ++i) {
    if (!(limits.max_velocity[i] > 0.0) || !(limits.max_acceleration[i] > 0.0) ||
      !std::isfinite(limits.max_velocity[i]) || !std::isfinite(limits.max_acceleration[i]))
    {
      throw std::invalid_argument("planJointPath: limits must be finite and positive");
    }
  }
  if (waypoints.empty() || !(sample_period_s > 0.0) || !std::isfinite(sample_period_s) ||
    !(max_deviation_rad >= 0.0) || !std::isfinite(max_deviation_rad))
  {
    throw std::invalid_argument(
            "planJointPath: needs waypoints, a positive sample period, a deviation >= 0");
  }
  // Drop waypoints that repeat the previous one: TOTG needs distinct consecutive points.
  std::list<Eigen::VectorXd> path;
  for (const auto & waypoint : waypoints) {
    for (const double q : waypoint) {
      if (!std::isfinite(q)) {
        throw std::invalid_argument("planJointPath: positions must be finite");
      }
    }
    const Eigen::VectorXd point = toEigen(waypoint);
    if (path.empty() || (point - path.back()).cwiseAbs().maxCoeff() >= kNoMotionRad) {
      path.push_back(point);
    }
  }
  const std::array<double, 7> & goal = waypoints.back();

  JointTrajectoryPlan plan;
  if (path.size() < 2) {
    plan.times_s = {0.0};
    plan.positions = {goal};
    plan.velocities = {std::array<double, 7>{}};
    return plan;
  }

  const trajectory_processing::Trajectory trajectory(
    trajectory_processing::Path(path, max_deviation_rad), toEigen(limits.max_velocity),
    toEigen(limits.max_acceleration), kIntegrationStepS);
  if (!trajectory.isValid()) {
    throw std::runtime_error("planJointPath: TOTG did not find a trajectory");
  }
  const double duration = trajectory.getDuration();
  // Positions and velocities only: TOTG's getAcceleration() reports values above the limit at
  // some instants while the positions themselves respect it (week5 Stage 3, 3.1).
  for (int k = 0; k * sample_period_s < duration; ++k) {
    const double t = k * sample_period_s;
    plan.times_s.push_back(t);
    plan.positions.push_back(toArray(trajectory.getPosition(t)));
    plan.velocities.push_back(toArray(trajectory.getVelocity(t)));
  }
  // The end exactly: the goal, at rest. A sample within a microsecond of it is replaced.
  if (plan.times_s.size() > 1 && duration - plan.times_s.back() < 1e-6) {
    plan.times_s.pop_back();
    plan.positions.pop_back();
    plan.velocities.pop_back();
  }
  plan.times_s.push_back(duration);
  plan.positions.push_back(goal);
  plan.velocities.push_back(std::array<double, 7>{});
  return plan;
}

}  // namespace task_executor
