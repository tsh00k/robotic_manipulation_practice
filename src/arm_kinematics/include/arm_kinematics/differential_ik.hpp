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

#include <Eigen/Geometry>

#include <cstddef>
#include <limits>

#include "arm_kinematics/model.hpp"

namespace arm_kinematics
{

using TaskVector = Eigen::Matrix<double, 6, 1>;

// All task-space quantities use the world frame and the ordering
// [x, y, z, rx, ry, rz]. Translation is measured in metres and rotation is an
// axis-angle vector in radians. The two weights explicitly nondimensionalize
// this mixed-unit error before DLS is applied:
//   e_w = diag(w_translation I3, w_rotation I3) e, J_w = W J.
// Damping and singular values therefore live in this weighted convention and
// must not be compared with values produced using different weights.
struct DifferentialIkParameters
{
  double translation_weight = 1.0;
  double rotation_weight = 0.2;
  double damping_threshold = 0.08;
  double maximum_damping = 0.05;
  double joint_centering_gain = 0.02;
  double maximum_translation_error = 0.05;
  double maximum_rotation_error = 0.2;
  double maximum_joint_step = 0.12;
  double joint_limit_margin = 1e-4;
};

struct DifferentialIkStep
{
  JointVector delta_q = JointVector::Zero();
  double minimum_singular_value = 0.0;
  double condition_number = std::numeric_limits<double>::infinity();
  double damping = 0.0;
  double weighted_error_norm = 0.0;
};

enum class IkStatus
{
  kConverged,
  kMaxIterations,
  kStalled,
};

struct IkParameters
{
  DifferentialIkParameters step;
  std::size_t max_iterations = 250;
  double position_tolerance = 1e-4;
  double orientation_tolerance = 1e-3;
  double minimum_progress = 1e-10;
  std::size_t stalled_iterations = 12;
};

struct IkResult
{
  IkStatus status = IkStatus::kMaxIterations;
  JointVector q = JointVector::Zero();
  std::size_t iterations = 0;
  double position_error = std::numeric_limits<double>::infinity();
  double orientation_error = std::numeric_limits<double>::infinity();
  double minimum_singular_value = 0.0;
  double condition_number = std::numeric_limits<double>::infinity();
  double damping = 0.0;
};

// Returns target minus current pose error, expressed in world coordinates.
TaskVector poseError(
  const Eigen::Isometry3d & current, const Eigen::Isometry3d & target);

// Computes one bounded, joint-limit-aware weighted DLS step. Joint centering is
// projected through I - J# J_w, where J# is the damped weighted inverse. The
// returned delta is clipped per joint, then projected into the model limits.
DifferentialIkStep differentialIkStep(
  const ArmModel & model, const JointVector & q, const Jacobian & jacobian,
  const TaskVector & error,
  const DifferentialIkParameters & parameters = DifferentialIkParameters());

// Uses differentialIkStep iteratively as an offline pose IK solver. Failure is
// reported with the last finite state and residual; unreachable goals never
// return a nominally successful or NaN solution.
IkResult solveIk(
  const ArmModel & model, const JointVector & seed,
  const Eigen::Isometry3d & target,
  const IkParameters & parameters = IkParameters());

}  // namespace arm_kinematics
