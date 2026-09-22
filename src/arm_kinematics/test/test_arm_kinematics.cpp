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

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "arm_kinematics/forward_kinematics.hpp"

namespace arm_kinematics
{
namespace
{

ArmModel loadModel()
{
  return loadFrankaFerModel(KINEMATICS_YAML_PATH, JOINT_LIMITS_YAML_PATH);
}

JointVector sampleConfiguration()
{
  JointVector q;
  q << 0.25, -0.55, 0.4, -1.8, 0.7, 1.2, -0.35;
  return q;
}

Jacobian finiteDifferenceJacobian(const ArmModel & model, const JointVector & q, double h)
{
  Jacobian result = Jacobian::Zero();
  const ForwardKinematics center = fk(model, q);

  for (std::size_t i = 0; i < kArmDof; ++i) {
    JointVector plus = q;
    JointVector minus = q;
    plus(static_cast<Eigen::Index>(i)) += h;
    minus(static_cast<Eigen::Index>(i)) -= h;
    const ForwardKinematics plus_fk = fk(model, plus);
    const ForwardKinematics minus_fk = fk(model, minus);

    result.block<3, 1>(0, static_cast<Eigen::Index>(i)) =
      (plus_fk.hand_tcp.translation() - minus_fk.hand_tcp.translation()) / (2.0 * h);

    const Eigen::Matrix3d rotation_dot =
      (plus_fk.hand_tcp.linear() - minus_fk.hand_tcp.linear()) / (2.0 * h);
    const Eigen::Matrix3d omega_hat = rotation_dot * center.hand_tcp.linear().transpose();
    result.block<3, 1>(3, static_cast<Eigen::Index>(i)) <<
      omega_hat(2, 1), omega_hat(0, 2), omega_hat(1, 0);
  }

  return result;
}

TEST(ModelLoader, ReadsOfficialFerTables)
{
  const ArmModel model = loadModel();
  EXPECT_NEAR(model.joints[0].origin.translation().z(), 0.333, 1e-12);
  EXPECT_NEAR(model.joints[4].origin.translation().y(), 0.384, 1e-12);
  EXPECT_NEAR(model.joints[3].limits.lower, -3.0718, 1e-12);
  EXPECT_NEAR(model.joints[5].limits.upper, 3.7525, 1e-12);
  EXPECT_NEAR(model.link7_to_link8.translation().z(), 0.107, 1e-12);
  EXPECT_NEAR(model.hand_to_tcp.translation().z(), 0.1034, 1e-12);
}

TEST(ModelLoader, RejectsMissingAndNonFiniteKinematicValues)
{
  EXPECT_THROW(
    loadFrankaFerModel(std::string(TEST_FIXTURE_DIR) + "/missing_field.yaml"),
    std::runtime_error);
  EXPECT_THROW(
    loadFrankaFerModel(std::string(TEST_FIXTURE_DIR) + "/non_finite.yaml"),
    std::runtime_error);
  EXPECT_THROW(
    loadFrankaFerModel(
      KINEMATICS_YAML_PATH, std::string(TEST_FIXTURE_DIR) + "/reversed_limits.yaml"),
    std::runtime_error);
}

TEST(ForwardKinematics, ProducesAConsistentTransformChain)
{
  const ArmModel model = loadModel();
  const JointVector q = sampleConfiguration();
  const ForwardKinematics result = fk(model, q);

  for (const Eigen::Isometry3d & transform : result.link) {
    EXPECT_TRUE(transform.matrix().allFinite());
    EXPECT_TRUE(
      (transform.linear().transpose() * transform.linear()).isApprox(
        Eigen::Matrix3d::Identity(), 1e-12));
    EXPECT_NEAR(transform.linear().determinant(), 1.0, 1e-12);
  }

  EXPECT_TRUE(
    result.link[8].matrix().isApprox(
      (result.link[7] * model.link7_to_link8).matrix(), 1e-12));
  EXPECT_TRUE(
    result.hand.matrix().isApprox(
      (result.link[8] * model.link8_to_hand).matrix(), 1e-12));
  EXPECT_TRUE(
    result.hand_tcp.matrix().isApprox(
      (result.hand * model.hand_to_tcp).matrix(), 1e-12));

  const Eigen::Isometry3d identity = result.hand_tcp * result.hand_tcp.inverse();
  EXPECT_TRUE(identity.isApprox(Eigen::Isometry3d::Identity(), 1e-12));
}

TEST(Jacobian, MatchesCentralFiniteDifference)
{
  const ArmModel model = loadModel();
  const JointVector q = sampleConfiguration();
  const Jacobian analytic = jacobian(model, q);

  const double coarse_error =
    (analytic - finiteDifferenceJacobian(model, q, 1e-2)).norm();
  const double medium_error =
    (analytic - finiteDifferenceJacobian(model, q, 5e-3)).norm();
  const double fine_error =
    (analytic - finiteDifferenceJacobian(model, q, 2.5e-3)).norm();

  EXPECT_LT(fine_error, medium_error * 0.5);
  EXPECT_LT(medium_error, coarse_error * 0.5);
  EXPECT_LT(fine_error, 1e-5);
}

TEST(Jacobian, UsesHandTcpRatherThanHandAsReferencePoint)
{
  const ArmModel model = loadModel();
  const JointVector q = sampleConfiguration();
  const ForwardKinematics transforms = fk(model, q);
  const Jacobian tcp_jacobian = jacobian(model, q);

  ASSERT_GT(model.hand_to_tcp.translation().norm(), 0.0);
  EXPECT_GT((transforms.hand_tcp.translation() - transforms.hand.translation()).norm(), 0.1);
  EXPECT_GT((tcp_jacobian.topRows<3>()).norm(), 0.0);
}

TEST(Model, EnforcesTheOfficialJointLimits)
{
  const ArmModel model = loadModel();
  JointVector q;
  for (std::size_t i = 0; i < kArmDof; ++i) {
    q(static_cast<Eigen::Index>(i)) = model.joints[i].limits.lower;
  }
  EXPECT_TRUE(withinJointLimits(model, q));

  q(3) = model.joints[3].limits.upper + 1e-4;
  EXPECT_FALSE(withinJointLimits(model, q));
  EXPECT_TRUE(withinJointLimits(model, q, 1e-3));
}

TEST(Model, RejectsNonFiniteInputsButAllowsFiniteOutOfRangeKinematics)
{
  const ArmModel model = loadModel();
  JointVector q = sampleConfiguration();
  q(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(fk(model, q), std::invalid_argument);
  EXPECT_THROW(jacobian(model, q), std::invalid_argument);
  EXPECT_FALSE(withinJointLimits(model, q));

  q = sampleConfiguration();
  q(0) = model.joints[0].limits.upper + 0.1;
  EXPECT_FALSE(withinJointLimits(model, q));
  EXPECT_NO_THROW(fk(model, q));
  EXPECT_NO_THROW(jacobian(model, q));
  EXPECT_TRUE(fk(model, q).hand_tcp.matrix().allFinite());
  EXPECT_TRUE(jacobian(model, q).allFinite());
}

TEST(Model, NearSingularConfigurationRemainsFinite)
{
  const ArmModel model = loadModel();
  JointVector q;
  q << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
  EXPECT_TRUE(fk(model, q).hand_tcp.matrix().allFinite());
  EXPECT_TRUE(jacobian(model, q).allFinite());
}

}  // namespace
}  // namespace arm_kinematics
