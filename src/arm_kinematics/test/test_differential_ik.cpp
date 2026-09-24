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

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>

#include "arm_kinematics/differential_ik.hpp"
#include "arm_kinematics/forward_kinematics.hpp"

namespace arm_kinematics
{
namespace
{

ArmModel loadModel()
{
  return loadFrankaFerModel(KINEMATICS_YAML_PATH, JOINT_LIMITS_YAML_PATH);
}

JointVector readyConfiguration()
{
  JointVector q;
  q << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
  return q;
}

DifferentialIkParameters unconstrainedStepParameters()
{
  DifferentialIkParameters parameters;
  parameters.translation_weight = 1.0;
  parameters.rotation_weight = 1.0;
  parameters.joint_centering_gain = 0.0;
  parameters.maximum_translation_error = 10.0;
  parameters.maximum_rotation_error = 10.0;
  parameters.maximum_joint_step = 10.0;
  parameters.joint_limit_margin = 0.0;
  return parameters;
}

TEST(PoseError, UsesWorldFrameTranslationAndAxisAngleRotation)
{
  const Eigen::Isometry3d current = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(0.1, -0.2, 0.3);
  target.linear() =
    Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const TaskVector error = poseError(current, target);
  EXPECT_TRUE(error.head<3>().isApprox(target.translation(), 1e-12));
  EXPECT_TRUE(error.tail<3>().isApprox(Eigen::Vector3d(0.0, 0.25, 0.0), 1e-12));
}

TEST(DifferentialIkStep, SolvesAWellConditionedLinearTask)
{
  const ArmModel model = loadModel();
  const JointVector q = readyConfiguration();
  Jacobian task_jacobian = Jacobian::Zero();
  task_jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  TaskVector error;
  error << 0.01, -0.02, 0.03, -0.04, 0.05, -0.06;

  const DifferentialIkStep step = differentialIkStep(
    model, q, task_jacobian, error, unconstrainedStepParameters());

  EXPECT_NEAR(step.damping, 0.0, 1e-15);
  EXPECT_NEAR(step.minimum_singular_value, 1.0, 1e-15);
  EXPECT_NEAR(step.condition_number, 1.0, 1e-15);
  EXPECT_TRUE((task_jacobian * step.delta_q).isApprox(error, 1e-12));
  EXPECT_NEAR(step.delta_q(6), 0.0, 1e-12);
}

TEST(DifferentialIkStep, DampingKeepsASingularTaskBounded)
{
  const ArmModel model = loadModel();
  const JointVector q = readyConfiguration();
  Jacobian singular_jacobian = Jacobian::Zero();
  singular_jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  singular_jacobian.row(5) *= 1e-9;
  TaskVector error = TaskVector::Zero();
  error(5) = 0.1;
  DifferentialIkParameters parameters = unconstrainedStepParameters();
  parameters.maximum_damping = 0.05;
  parameters.damping_threshold = 0.08;

  const DifferentialIkStep step =
    differentialIkStep(model, q, singular_jacobian, error, parameters);

  EXPECT_GT(step.damping, 0.049);
  EXPECT_LT(step.minimum_singular_value, 1e-8);
  EXPECT_TRUE(step.delta_q.allFinite());
  EXPECT_LT(step.delta_q.norm(), 1e-6);
}

TEST(DifferentialIkStep, BoundsErrorsStepsAndJointLimits)
{
  const ArmModel model = loadModel();
  JointVector q = readyConfiguration();
  q(0) = model.joints[0].limits.upper - 0.005;
  Jacobian task_jacobian = Jacobian::Zero();
  task_jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  TaskVector error = TaskVector::Constant(100.0);
  DifferentialIkParameters parameters = unconstrainedStepParameters();
  parameters.maximum_translation_error = 0.03;
  parameters.maximum_rotation_error = 0.06;
  parameters.maximum_joint_step = 0.02;
  parameters.joint_limit_margin = 1e-3;

  const DifferentialIkStep step =
    differentialIkStep(model, q, task_jacobian, error, parameters);

  EXPECT_LE(step.delta_q.cwiseAbs().maxCoeff(), 0.02 + 1e-12);
  EXPECT_LE(
    q(0) + step.delta_q(0), model.joints[0].limits.upper - parameters.joint_limit_margin);
  EXPECT_TRUE(withinJointLimits(model, q + step.delta_q));
}

TEST(DifferentialIkStep, JointCenteringActsThroughTheRedundantDirection)
{
  const ArmModel model = loadModel();
  JointVector q = readyConfiguration();
  q(6) = model.joints[6].limits.upper - 0.2;
  Jacobian task_jacobian = Jacobian::Zero();
  task_jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  DifferentialIkParameters parameters = unconstrainedStepParameters();
  parameters.joint_centering_gain = 0.1;

  const DifferentialIkStep step = differentialIkStep(
    model, q, task_jacobian, TaskVector::Zero(), parameters);

  EXPECT_NEAR(step.delta_q.head<6>().norm(), 0.0, 1e-12);
  EXPECT_LT(step.delta_q(6), 0.0);
}

TEST(SolveIk, ConvergesToSeveralReachablePoses)
{
  const ArmModel model = loadModel();
  const JointVector seed = readyConfiguration();
  const std::array<JointVector, 4> targets = {
    (JointVector() << 0.2, -0.65, 0.15, -2.1, 0.1, 1.65, 0.55).finished(),
    (JointVector() << -0.35, -0.9, 0.25, -1.9, -0.25, 1.35, 1.0).finished(),
    (JointVector() << 0.45, -0.45, -0.35, -2.45, 0.3, 1.9, 0.2).finished(),
    (JointVector() << -0.15, -1.1, -0.2, -2.0, 0.45, 1.15, 0.65).finished()};

  for (const JointVector & target_q : targets) {
    SCOPED_TRACE(target_q.transpose());
    const Eigen::Isometry3d target = fk(model, target_q).hand_tcp;
    const IkResult result = solveIk(model, seed, target);
    EXPECT_EQ(result.status, IkStatus::kConverged);
    EXPECT_LT(result.iterations, 250U);
    EXPECT_LT(result.position_error, 1e-4);
    EXPECT_LT(result.orientation_error, 1e-3);
    EXPECT_TRUE(withinJointLimits(model, result.q));
    EXPECT_TRUE(result.q.allFinite());
    std::cout << "stage_m_convergence,iterations=" << result.iterations
              << ",position_error=" << result.position_error
              << ",orientation_error=" << result.orientation_error << '\n';
  }
}

TEST(SolveIk, ReportsFiniteFailureForAnUnreachablePose)
{
  const ArmModel model = loadModel();
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(5.0, 5.0, 5.0);
  IkParameters parameters;
  parameters.max_iterations = 80;

  const IkResult result = solveIk(model, readyConfiguration(), target, parameters);

  EXPECT_NE(result.status, IkStatus::kConverged);
  EXPECT_LE(result.iterations, parameters.max_iterations);
  EXPECT_TRUE(result.q.allFinite());
  EXPECT_TRUE(std::isfinite(result.position_error));
  EXPECT_TRUE(std::isfinite(result.orientation_error));
  EXPECT_GT(result.position_error, 1.0);
  EXPECT_TRUE(withinJointLimits(model, result.q));
  std::cout << "stage_m_unreachable,status=" << static_cast<int>(result.status)
            << ",iterations=" << result.iterations
            << ",position_error=" << result.position_error
            << ",orientation_error=" << result.orientation_error << '\n';
}

TEST(SolveIk, RejectsInvalidInputs)
{
  const ArmModel model = loadModel();
  JointVector invalid_seed = readyConfiguration();
  invalid_seed(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    solveIk(model, invalid_seed, Eigen::Isometry3d::Identity()), std::invalid_argument);

  JointVector outside_limits = readyConfiguration();
  outside_limits(0) = model.joints[0].limits.upper + 0.1;
  EXPECT_THROW(
    solveIk(model, outside_limits, Eigen::Isometry3d::Identity()), std::invalid_argument);
}

}  // namespace
}  // namespace arm_kinematics
