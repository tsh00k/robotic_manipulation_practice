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

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "mujoco_bridge/trajectory_interpolator.hpp"
#include "task_executor/joint_trajectory_planner.hpp"
#include "task_executor/waypoint_source.hpp"

namespace task_executor
{
namespace
{

// FCI limits for the Franka Emika Robot; the reference uses a quarter of the acceleration.
constexpr std::array<double, 7> kFciAcceleration{15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};
constexpr std::array<double, 7> kFciJerk{7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0};

// HOME to the PREGRASP of HELD-A layout 14, and a short descent from that layout.
constexpr std::array<double, 7> kPregrasp{0.011, 0.210, -0.217, -1.639, 0.047, 1.844, -1.495};
constexpr std::array<double, 7> kGrasp{0.000, 0.323, -0.200, -1.875, 0.078, 2.190, -1.515};

// The rest-to-rest time-optimal time along a straight line: a trapezoid (or triangle) in the
// path parameter, limited by the most constrained joint (week5, route section F).
double closedFormDuration(
  const std::array<double, 7> & a, const std::array<double, 7> & b, const JointLimits & limits)
{
  double sd = INFINITY, sdd = INFINITY;
  for (std::size_t i = 0; i < 7; ++i) {
    const double dq = std::abs(b[i] - a[i]);
    if (dq > 0.0) {
      sd = std::min(sd, limits.max_velocity[i] / dq);
      sdd = std::min(sdd, limits.max_acceleration[i] / dq);
    }
  }
  return sd * sd / sdd < 1.0 ? 2.0 * sd / sdd + (1.0 - sd * sd / sdd) / sd : 2.0 * std::sqrt(
    1.0 / sdd);
}

// The bridge's reference at time t (linear between the samples).
mujoco_bridge::TrajectoryInterpolator bridgeView(const JointTrajectoryPlan & plan)
{
  std::vector<std::vector<double>> positions;
  for (const auto & q : plan.positions) {
    positions.emplace_back(q.begin(), q.end());
  }
  mujoco_bridge::TrajectoryInterpolator interpolator;
  interpolator.set(plan.times_s, positions);
  return interpolator;
}

// T1 and T2 of week5 Stage 3: the duration is the closed-form trapezoid, the path is the line,
// and the reference the bridge executes stays within the velocity, acceleration and FCI jerk
// limits when differentiated every 1 ms (the real robot's control cycle).
void checkSegment(const std::array<double, 7> & a, const std::array<double, 7> & b)
{
  const JointTrajectoryPlan plan = planJointLine(a, b);
  ASSERT_GT(plan.times_s.size(), 2u);
  EXPECT_NEAR(plan.duration_s(), closedFormDuration(a, b, kPandaReferenceLimits), 1e-3);
  EXPECT_EQ(plan.positions.front(), a);
  EXPECT_EQ(plan.positions.back(), b);
  EXPECT_EQ(plan.velocities.back(), (std::array<double, 7>{}));

  const auto interpolator = bridgeView(plan);
  const double h = 0.001;
  const int steps = static_cast<int>(std::ceil(plan.duration_s() / h)) + 3;
  std::vector<std::vector<double>> q(steps + 1);
  for (int k = 0; k <= steps; ++k) {
    ASSERT_TRUE(interpolator.sample((k - 1) * h, q[k]));
  }
  double worst_off_line = 0.0, worst_velocity = 0.0, worst_acceleration = 0.0, worst_jerk = 0.0;
  for (int k = 0; k <= steps; ++k) {
    // Off the straight line from a to b: the component orthogonal to b - a.
    double dot = 0.0, norm2 = 0.0;
    for (std::size_t i = 0; i < 7; ++i) {
      dot += (q[k][i] - a[i]) * (b[i] - a[i]);
      norm2 += (b[i] - a[i]) * (b[i] - a[i]);
    }
    for (std::size_t i = 0; i < 7; ++i) {
      const double on_line = a[i] + dot / norm2 * (b[i] - a[i]);
      worst_off_line = std::max(worst_off_line, std::abs(q[k][i] - on_line));
    }
  }
  for (int k = 1; k + 1 <= steps; ++k) {
    for (std::size_t i = 0; i < 7; ++i) {
      const double velocity = (q[k + 1][i] - q[k][i]) / h;
      const double acceleration = (q[k + 1][i] - 2.0 * q[k][i] + q[k - 1][i]) / (h * h);
      worst_velocity = std::max(
        worst_velocity, std::abs(
          velocity) / kPandaReferenceLimits.max_velocity[i]);
      worst_acceleration = std::max(
        worst_acceleration, std::abs(acceleration) / kPandaReferenceLimits.max_acceleration[i]);
      if (k + 2 <= steps) {
        const double next = (q[k + 2][i] - 2.0 * q[k + 1][i] + q[k][i]) / (h * h);
        worst_jerk = std::max(worst_jerk, std::abs(next - acceleration) / h / kFciJerk[i]);
      }
    }
  }
  EXPECT_LT(worst_off_line, 1e-9);
  EXPECT_LE(worst_velocity, 1.0 + 1e-6);
  EXPECT_LE(worst_acceleration, 1.0 + 1e-6);
  EXPECT_LE(worst_jerk, 1.0 + 1e-6);
  ::testing::Test::RecordProperty("duration_s", std::to_string(plan.duration_s()));
}

TEST(JointTrajectoryPlanner, LongSegmentFromHome) {
  checkSegment(kFrankaReadyPose, kPregrasp);
}
TEST(JointTrajectoryPlanner, ShortDescent) {
  checkSegment(kPregrasp, kGrasp);
}
TEST(JointTrajectoryPlanner, BackHome) {
  checkSegment(kGrasp, kFrankaReadyPose);
}

// A dense curved path (an arc in joints 2 and 4, 31 waypoints) as a TCP line produces: the
// rounded corners keep the limits, the plan ends at the last waypoint, and it never strays more
// than the allowed deviation from the polyline.
TEST(JointTrajectoryPlanner, DensePathKeepsTheLimits)
{
  std::vector<std::array<double, 7>> waypoints;
  for (int k = 0; k <= 30; ++k) {
    const double angle = 0.5 * k / 30.0;
    std::array<double, 7> q = kPregrasp;
    q[1] += 0.3 * std::sin(angle);
    q[3] += 0.3 * (1.0 - std::cos(angle));
    waypoints.push_back(q);
  }
  const JointTrajectoryPlan plan =
    planJointPath(waypoints, kPandaReferenceLimits, 0.001, 1e-3);
  ASSERT_GT(plan.times_s.size(), 2u);
  EXPECT_EQ(plan.positions.back(), waypoints.back());
  double worst_velocity = 0.0, worst_acceleration = 0.0;
  for (std::size_t k = 1; k + 1 < plan.positions.size(); ++k) {
    const double h = plan.times_s[k] - plan.times_s[k - 1];
    const double h2 = plan.times_s[k + 1] - plan.times_s[k];
    for (std::size_t i = 0; i < 7; ++i) {
      const double v1 = (plan.positions[k][i] - plan.positions[k - 1][i]) / h;
      const double v2 = (plan.positions[k + 1][i] - plan.positions[k][i]) / h2;
      worst_velocity =
        std::max(worst_velocity, std::abs(v2) / kPandaReferenceLimits.max_velocity[i]);
      worst_acceleration = std::max(
        worst_acceleration,
        std::abs(v2 - v1) / (0.5 * (h + h2)) / kPandaReferenceLimits.max_acceleration[i]);
    }
  }
  EXPECT_LE(worst_velocity, 1.0 + 1e-6);
  // TOTG integrates the rounded corners numerically; on this path the acceleration
  // overshoots its limit by 0.4 % (week5 Stage 3), within 1 %.
  EXPECT_LE(worst_acceleration, 1.01);
}

TEST(JointTrajectoryPlanner, NoMotionIsASingleSample)
{
  const JointTrajectoryPlan plan = planJointLine(kPregrasp, kPregrasp);
  ASSERT_EQ(plan.times_s.size(), 1u);
  EXPECT_EQ(plan.positions.front(), kPregrasp);
  EXPECT_DOUBLE_EQ(plan.duration_s(), 0.0);
}

TEST(JointTrajectoryPlanner, ReferenceLimitsAre95PercentVelocityAndAQuarterAcceleration)
{
  const std::array<double, 7> fci_velocity{2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
  for (std::size_t i = 0; i < 7; ++i) {
    EXPECT_NEAR(kPandaReferenceLimits.max_velocity[i], 0.95 * fci_velocity[i], 1e-12);
    EXPECT_DOUBLE_EQ(kPandaReferenceLimits.max_acceleration[i], kFciAcceleration[i] / 4.0);
  }
}

TEST(JointTrajectoryPlanner, RejectsBadInput)
{
  std::array<double, 7> bad = kPregrasp;
  bad[2] = std::nan("");
  EXPECT_THROW(planJointLine(bad, kGrasp), std::invalid_argument);
  JointLimits limits = kPandaReferenceLimits;
  limits.max_acceleration[1] = 0.0;
  EXPECT_THROW(planJointLine(kPregrasp, kGrasp, limits), std::invalid_argument);
  EXPECT_THROW(planJointLine(kPregrasp, kGrasp, kPandaReferenceLimits, 0.0), std::invalid_argument);
}

}  // namespace
}  // namespace task_executor
