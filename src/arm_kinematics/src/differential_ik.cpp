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

#include "arm_kinematics/differential_ik.hpp"

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "arm_kinematics/forward_kinematics.hpp"

namespace arm_kinematics
{
namespace
{

using TaskMatrix = Eigen::Matrix<double, 6, 6>;

void validate(const DifferentialIkParameters & parameters)
{
  if (!(parameters.translation_weight > 0.0) || !(parameters.rotation_weight > 0.0) ||
    !(parameters.damping_threshold > 0.0) || !(parameters.maximum_damping > 0.0) ||
    parameters.joint_centering_gain < 0.0 ||
    !(parameters.maximum_translation_error > 0.0) ||
    !(parameters.maximum_rotation_error > 0.0) || !(parameters.maximum_joint_step > 0.0) ||
    parameters.joint_limit_margin < 0.0)
  {
    throw std::invalid_argument("Differential IK parameters are outside their valid range");
  }
}

Eigen::DiagonalMatrix<double, 6> taskWeights(const DifferentialIkParameters & parameters)
{
  TaskVector diagonal;
  diagonal << parameters.translation_weight, parameters.translation_weight,
    parameters.translation_weight, parameters.rotation_weight, parameters.rotation_weight,
    parameters.rotation_weight;
  return diagonal.asDiagonal();
}

void clampNorm(Eigen::Vector3d & value, double maximum_norm)
{
  const double norm = value.norm();
  if (norm > maximum_norm) {
    value *= maximum_norm / norm;
  }
}

JointVector centeringDirection(const ArmModel & model, const JointVector & q)
{
  JointVector direction = JointVector::Zero();
  for (std::size_t i = 0; i < kArmDof; ++i) {
    const Eigen::Index index = static_cast<Eigen::Index>(i);
    const double lower = model.joints[i].limits.lower;
    const double upper = model.joints[i].limits.upper;
    if (std::isfinite(lower) && std::isfinite(upper) && upper > lower) {
      const double midpoint = 0.5 * (lower + upper);
      const double half_range = 0.5 * (upper - lower);
      direction(index) = (midpoint - q(index)) / half_range;
    }
  }
  return direction;
}

double adaptiveDamping(double minimum_singular_value, const DifferentialIkParameters & parameters)
{
  if (minimum_singular_value >= parameters.damping_threshold) {
    return 0.0;
  }
  const double ratio = minimum_singular_value / parameters.damping_threshold;
  return parameters.maximum_damping * (1.0 - ratio * ratio);
}

}  // namespace

TaskVector poseError(const Eigen::Isometry3d & current, const Eigen::Isometry3d & target)
{
  if (!current.matrix().allFinite() || !target.matrix().allFinite()) {
    throw std::invalid_argument("Poses must be finite");
  }

  TaskVector error;
  error.head<3>() = target.translation() - current.translation();
  const Eigen::AngleAxisd rotation_error(target.linear() * current.linear().transpose());
  error.tail<3>() = rotation_error.axis() * rotation_error.angle();
  return error;
}

DifferentialIkStep differentialIkStep(
  const ArmModel & model, const JointVector & q, const Jacobian & geometric_jacobian,
  const TaskVector & error, const DifferentialIkParameters & parameters)
{
  validate(parameters);
  if (!q.allFinite() || !geometric_jacobian.allFinite() || !error.allFinite()) {
    throw std::invalid_argument("Differential IK inputs must be finite");
  }
  if (!withinJointLimits(model, q)) {
    throw std::invalid_argument("Differential IK state is outside the joint limits");
  }

  TaskVector bounded_error = error;
  Eigen::Vector3d translation = bounded_error.head<3>();
  Eigen::Vector3d rotation = bounded_error.tail<3>();
  clampNorm(translation, parameters.maximum_translation_error);
  clampNorm(rotation, parameters.maximum_rotation_error);
  bounded_error.head<3>() = translation;
  bounded_error.tail<3>() = rotation;

  const Eigen::DiagonalMatrix<double, 6> weights = taskWeights(parameters);
  const Jacobian weighted_jacobian = weights * geometric_jacobian;
  const TaskVector weighted_error = weights * bounded_error;
  const Eigen::JacobiSVD<Jacobian> svd(weighted_jacobian);
  const auto singular_values = svd.singularValues();

  DifferentialIkStep result;
  result.minimum_singular_value = singular_values.minCoeff();
  const double maximum_singular_value = singular_values.maxCoeff();
  if (result.minimum_singular_value > 1e-12) {
    result.condition_number = maximum_singular_value / result.minimum_singular_value;
  }
  result.damping = adaptiveDamping(result.minimum_singular_value, parameters);
  result.weighted_error_norm = weighted_error.norm();

  TaskMatrix normal = weighted_jacobian * weighted_jacobian.transpose();
  normal.diagonal().array() += result.damping * result.damping;
  const Eigen::LDLT<TaskMatrix> factorization(normal);
  if (factorization.info() != Eigen::Success) {
    throw std::runtime_error("DLS normal matrix factorization failed");
  }

  const Eigen::Matrix<double, kArmDof, 6> damped_inverse =
    weighted_jacobian.transpose() * factorization.solve(TaskMatrix::Identity());
  result.delta_q = damped_inverse * weighted_error;

  const Eigen::Matrix<double, kArmDof, kArmDof> nullspace =
    Eigen::Matrix<double, kArmDof, kArmDof>::Identity() -
    damped_inverse * weighted_jacobian;
  result.delta_q += parameters.joint_centering_gain *
    nullspace * centeringDirection(model, q);

  for (std::size_t i = 0; i < kArmDof; ++i) {
    const Eigen::Index index = static_cast<Eigen::Index>(i);
    result.delta_q(index) = std::clamp(
      result.delta_q(index), -parameters.maximum_joint_step, parameters.maximum_joint_step);
    const double lower = model.joints[i].limits.lower + parameters.joint_limit_margin;
    const double upper = model.joints[i].limits.upper - parameters.joint_limit_margin;
    if (lower > upper) {
      throw std::invalid_argument("Joint-limit margin leaves no feasible interval");
    }
    if (std::isfinite(lower) && std::isfinite(upper)) {
      const double projected = std::clamp(q(index) + result.delta_q(index), lower, upper);
      result.delta_q(index) = projected - q(index);
    }
  }

  if (!result.delta_q.allFinite()) {
    throw std::runtime_error("Differential IK produced a non-finite step");
  }
  return result;
}

IkResult solveIk(
  const ArmModel & model, const JointVector & seed, const Eigen::Isometry3d & target,
  const IkParameters & parameters)
{
  validate(parameters.step);
  if (!seed.allFinite() || !target.matrix().allFinite() || parameters.max_iterations == 0U ||
    !(parameters.position_tolerance > 0.0) || !(parameters.orientation_tolerance > 0.0) ||
    parameters.minimum_progress < 0.0 || parameters.stalled_iterations == 0U)
  {
    throw std::invalid_argument("IK parameters and inputs must be finite and positive");
  }
  if (!withinJointLimits(model, seed)) {
    throw std::invalid_argument("IK seed is outside the joint limits");
  }

  IkResult result;
  result.q = seed;
  double previous_residual = std::numeric_limits<double>::infinity();
  std::size_t stalled_count = 0;

  for (std::size_t iteration = 0; iteration <= parameters.max_iterations; ++iteration) {
    const ForwardKinematics current = fk(model, result.q);
    const TaskVector error = poseError(current.hand_tcp, target);
    result.position_error = error.head<3>().norm();
    result.orientation_error = error.tail<3>().norm();
    result.iterations = iteration;
    if (result.position_error <= parameters.position_tolerance &&
      result.orientation_error <= parameters.orientation_tolerance)
    {
      result.status = IkStatus::kConverged;
      return result;
    }
    if (iteration == parameters.max_iterations) {
      result.status = IkStatus::kMaxIterations;
      return result;
    }

    const double residual = result.position_error + result.orientation_error;
    if (previous_residual - residual <= parameters.minimum_progress) {
      ++stalled_count;
    } else {
      stalled_count = 0;
    }
    previous_residual = residual;
    if (stalled_count >= parameters.stalled_iterations) {
      result.status = IkStatus::kStalled;
      return result;
    }

    const DifferentialIkStep step = differentialIkStep(
      model, result.q, jacobian(model, result.q), error, parameters.step);
    result.minimum_singular_value = step.minimum_singular_value;
    result.condition_number = step.condition_number;
    result.damping = step.damping;
    result.q += step.delta_q;
  }

  return result;
}

}  // namespace arm_kinematics
