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

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "arm_kinematics/forward_kinematics.hpp"
#include "mujoco_bridge/mujoco_dl.hpp"

namespace mujoco_bridge
{
namespace
{

constexpr double kHandToTcpZ = 0.1034;
constexpr double kPositionToleranceM = 1e-6;
constexpr double kOrientationToleranceRad = 1e-6;
constexpr double kJacobianTolerance = 1e-8;

struct Configuration
{
  const char * name;
  const char * purpose;
  arm_kinematics::JointVector q;
};

std::string readFile(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("Cannot open " + path);
  }
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

double orientationError(
  const Eigen::Isometry3d & lhs, const Eigen::Isometry3d & rhs)
{
  return Eigen::AngleAxisd(lhs.linear().transpose() * rhs.linear()).angle();
}

std::vector<Configuration> configurations()
{
  using arm_kinematics::JointVector;
  std::vector<Configuration> result;

  JointVector zero = JointVector::Zero();
  result.push_back({"zero", "algebraic zero, including joint4 outside its physical range", zero});

  JointVector home;
  home << 0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853;
  result.push_back({"mjcf_home", "MuJoCo vendor keyframe", home});

  JointVector ready;
  ready << 0.0, -M_PI / 4.0, 0.0, -3.0 * M_PI / 4.0, 0.0, M_PI / 2.0,
    M_PI / 4.0;
  result.push_back({"srdf_ready", "official SRDF ready state", ready});

  JointVector extended;
  extended << 0.0, 0.0, 0.0, -0.1, 0.0, M_PI / 2.0, M_PI / 4.0;
  result.push_back({"near_singular", "near-extended arm from the SRDF", extended});

  JointVector near_limits;
  near_limits << 2.88, 1.74, -2.88, -3.05, 2.88, 3.73, -2.88;
  result.push_back({"near_limits", "all seven joints close to alternating limits", near_limits});

  JointVector random_a;
  random_a << 0.31, -1.02, 1.47, -2.41, -0.82, 2.28, 1.19;
  result.push_back({"random_a", "fixed interior sample A", random_a});

  JointVector random_b;
  random_b << -1.73, 0.64, -0.91, -1.13, 1.82, 0.37, -2.11;
  result.push_back({"random_b", "fixed interior sample B", random_b});

  return result;
}

class ModelConsistencyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    arm_model_ = arm_kinematics::loadFrankaFerModel(
      KINEMATICS_YAML_PATH, JOINT_LIMITS_YAML_PATH);

    const auto concrete_urdf_model = std::make_shared<urdf::Model>();
    ASSERT_TRUE(concrete_urdf_model->initString(readFile(FER_URDF_PATH)));
    urdf_model_ = concrete_urdf_model;
    srdf_model_ = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf_model_->initString(*urdf_model_, readFile(FER_SRDF_PATH)));
    moveit_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model_, srdf_model_);
    moveit_state_ = std::make_unique<moveit::core::RobotState>(moveit_model_);
    moveit_state_->setToDefaultValues();
    arm_group_ = moveit_model_->getJointModelGroup("fer_arm");
    tcp_link_ = moveit_model_->getLinkModel("fer_hand_tcp");
    ASSERT_NE(arm_group_, nullptr);
    ASSERT_NE(tcp_link_, nullptr);
    ASSERT_EQ(arm_group_->getVariableCount(), arm_kinematics::kArmDof);

    char error[1024] = {};
    mujoco_model_ = api_.loadXML(PANDA_MJCF_PATH, nullptr, error, sizeof(error));
    ASSERT_NE(mujoco_model_, nullptr) << error;
    mujoco_data_ = api_.makeData(mujoco_model_);
    ASSERT_NE(mujoco_data_, nullptr);

    for (std::size_t i = 0; i < arm_kinematics::kArmDof; ++i) {
      const std::string joint_name = "joint" + std::to_string(i + 1);
      const int joint_id = api_.name2id(mujoco_model_, mjOBJ_JOINT, joint_name.c_str());
      ASSERT_GE(joint_id, 0) << joint_name;
      qpos_addresses_[i] = mujoco_model_->jnt_qposadr[joint_id];
      dof_addresses_[i] = mujoco_model_->jnt_dofadr[joint_id];
    }
  }

  void TearDown() override
  {
    if (mujoco_data_) {
      api_.deleteData(mujoco_data_);
    }
    if (mujoco_model_) {
      api_.deleteModel(mujoco_model_);
    }
  }

  void setConfiguration(const arm_kinematics::JointVector & q)
  {
    std::vector<double> moveit_q(q.data(), q.data() + q.size());
    moveit_state_->setJointGroupPositions(arm_group_, moveit_q);
    moveit_state_->update();

    for (std::size_t i = 0; i < arm_kinematics::kArmDof; ++i) {
      mujoco_data_->qpos[qpos_addresses_[i]] = q(static_cast<Eigen::Index>(i));
    }
    api_.forward(mujoco_model_, mujoco_data_);
  }

  Eigen::Isometry3d mujocoBodyTransform(const std::string & name) const
  {
    const int body_id = api_.name2id(mujoco_model_, mjOBJ_BODY, name.c_str());
    if (body_id < 0) {
      throw std::runtime_error("Missing MuJoCo body " + name);
    }
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Map<const Eigen::Vector3d>(mujoco_data_->xpos + 3 * body_id);
    const mjtNum * matrix = mujoco_data_->xmat + 9 * body_id;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        transform.linear()(row, column) = matrix[3 * row + column];
      }
    }
    return transform;
  }

  Eigen::Isometry3d mujocoTcpTransform() const
  {
    Eigen::Isometry3d tcp = mujocoBodyTransform("hand");
    tcp.translate(Eigen::Vector3d(0.0, 0.0, kHandToTcpZ));
    return tcp;
  }

  arm_kinematics::Jacobian mujocoTcpJacobian() const
  {
    const int hand_id = api_.name2id(mujoco_model_, mjOBJ_BODY, "hand");
    const Eigen::Vector3d point = mujocoTcpTransform().translation();
    std::vector<mjtNum> jacp(3 * mujoco_model_->nv, 0.0);
    std::vector<mjtNum> jacr(3 * mujoco_model_->nv, 0.0);
    api_.jac(
      mujoco_model_, mujoco_data_, jacp.data(), jacr.data(), point.data(), hand_id);

    arm_kinematics::Jacobian result = arm_kinematics::Jacobian::Zero();
    for (std::size_t column = 0; column < arm_kinematics::kArmDof; ++column) {
      const int dof = dof_addresses_[column];
      for (int row = 0; row < 3; ++row) {
        result(row, static_cast<Eigen::Index>(column)) =
          jacp[row * mujoco_model_->nv + dof];
        result(row + 3, static_cast<Eigen::Index>(column)) =
          jacr[row * mujoco_model_->nv + dof];
      }
    }
    return result;
  }

  arm_kinematics::Jacobian moveitTcpJacobian()
  {
    Eigen::MatrixXd dynamic_jacobian;
    if (!moveit_state_->getJacobian(
        arm_group_, tcp_link_, Eigen::Vector3d::Zero(), dynamic_jacobian))
    {
      throw std::runtime_error("MoveIt failed to compute the hand_tcp Jacobian");
    }
    if (dynamic_jacobian.rows() != 6 ||
      dynamic_jacobian.cols() != static_cast<Eigen::Index>(arm_kinematics::kArmDof))
    {
      throw std::runtime_error("MoveIt returned an unexpected Jacobian shape");
    }
    return dynamic_jacobian;
  }

  void expectTransformsNear(
    const std::string & configuration, const std::string & pair, const std::string & link,
    const Eigen::Isometry3d & lhs, const Eigen::Isometry3d & rhs,
    double & maximum_position_error, double & maximum_orientation_error)
  {
    const double position_error = (lhs.translation() - rhs.translation()).norm();
    const double orientation_error = orientationError(lhs, rhs);
    maximum_position_error = std::max(maximum_position_error, position_error);
    maximum_orientation_error = std::max(maximum_orientation_error, orientation_error);
    std::cout << "stage_l_fk," << configuration << ',' << pair << ',' << link << ','
              << std::scientific << std::setprecision(9) << position_error << ','
              << orientation_error << '\n';
    SCOPED_TRACE(configuration + ":" + pair + ":" + link);
    EXPECT_LT(position_error, kPositionToleranceM);
    EXPECT_LT(orientation_error, kOrientationToleranceRad);
  }

  MujocoApi & api_ = loadMujocoApi();
  arm_kinematics::ArmModel arm_model_;
  urdf::ModelInterfaceSharedPtr urdf_model_;
  std::shared_ptr<srdf::Model> srdf_model_;
  moveit::core::RobotModelPtr moveit_model_;
  std::unique_ptr<moveit::core::RobotState> moveit_state_;
  const moveit::core::JointModelGroup * arm_group_ = nullptr;
  const moveit::core::LinkModel * tcp_link_ = nullptr;
  mjModel * mujoco_model_ = nullptr;
  mjData * mujoco_data_ = nullptr;
  std::array<int, arm_kinematics::kArmDof> qpos_addresses_{};
  std::array<int, arm_kinematics::kArmDof> dof_addresses_{};
};

TEST_F(ModelConsistencyTest, FixedConfigurationsMatchAcrossAllThreeModels)
{
  double maximum_position_error = 0.0;
  double maximum_orientation_error = 0.0;
  double maximum_jacobian_error = 0.0;

  std::cout << "record,configuration,pair,link,position_error_m,orientation_error_rad\n";
  for (const Configuration & configuration : configurations()) {
    SCOPED_TRACE(
      std::string(configuration.name) + " (" + configuration.purpose + ")");
    setConfiguration(configuration.q);
    const arm_kinematics::ForwardKinematics own_fk =
      arm_kinematics::fk(arm_model_, configuration.q);

    for (std::size_t i = 0; i < arm_kinematics::kLinkCount; ++i) {
      const std::string moveit_link = "fer_link" + std::to_string(i);
      expectTransformsNear(
        configuration.name, "own_vs_moveit", moveit_link, own_fk.link[i],
        moveit_state_->getGlobalLinkTransform(moveit_link), maximum_position_error,
        maximum_orientation_error);
      if (i <= 7) {
        const std::string mujoco_link = "link" + std::to_string(i);
        const Eigen::Isometry3d mujoco_transform = mujocoBodyTransform(mujoco_link);
        expectTransformsNear(
          configuration.name, "own_vs_mujoco", mujoco_link, own_fk.link[i],
          mujoco_transform, maximum_position_error,
          maximum_orientation_error);
        expectTransformsNear(
          configuration.name, "moveit_vs_mujoco", mujoco_link,
          moveit_state_->getGlobalLinkTransform(moveit_link), mujoco_transform,
          maximum_position_error,
          maximum_orientation_error);
      }
    }

    expectTransformsNear(
      configuration.name, "own_vs_moveit", "hand", own_fk.hand,
      moveit_state_->getGlobalLinkTransform("fer_hand"), maximum_position_error,
      maximum_orientation_error);
    expectTransformsNear(
      configuration.name, "own_vs_mujoco", "hand", own_fk.hand,
      mujocoBodyTransform("hand"), maximum_position_error, maximum_orientation_error);
    expectTransformsNear(
      configuration.name, "moveit_vs_mujoco", "hand",
      moveit_state_->getGlobalLinkTransform("fer_hand"), mujocoBodyTransform("hand"),
      maximum_position_error, maximum_orientation_error);
    expectTransformsNear(
      configuration.name, "own_vs_moveit", "hand_tcp", own_fk.hand_tcp,
      moveit_state_->getGlobalLinkTransform("fer_hand_tcp"), maximum_position_error,
      maximum_orientation_error);
    expectTransformsNear(
      configuration.name, "own_vs_mujoco", "hand_tcp", own_fk.hand_tcp,
      mujocoTcpTransform(), maximum_position_error, maximum_orientation_error);
    expectTransformsNear(
      configuration.name, "moveit_vs_mujoco", "hand_tcp",
      moveit_state_->getGlobalLinkTransform("fer_hand_tcp"), mujocoTcpTransform(),
      maximum_position_error, maximum_orientation_error);

    const arm_kinematics::Jacobian own_jacobian =
      arm_kinematics::jacobian(arm_model_, configuration.q);
    const arm_kinematics::Jacobian moveit_jacobian = moveitTcpJacobian();
    const arm_kinematics::Jacobian mujoco_jacobian = mujocoTcpJacobian();
    const double own_moveit_error = (own_jacobian - moveit_jacobian).cwiseAbs().maxCoeff();
    const double own_mujoco_error = (own_jacobian - mujoco_jacobian).cwiseAbs().maxCoeff();
    const double moveit_mujoco_error =
      (moveit_jacobian - mujoco_jacobian).cwiseAbs().maxCoeff();
    maximum_jacobian_error =
      std::max(
      {maximum_jacobian_error, own_moveit_error, own_mujoco_error,
        moveit_mujoco_error});
    std::cout << "stage_l_jacobian," << configuration.name << ",own_vs_moveit,hand_tcp,"
              << own_moveit_error << '\n'
              << "stage_l_jacobian," << configuration.name << ",own_vs_mujoco,hand_tcp,"
              << own_mujoco_error << '\n'
              << "stage_l_jacobian," << configuration.name
              << ",moveit_vs_mujoco,hand_tcp," << moveit_mujoco_error << '\n';
    EXPECT_LT(own_moveit_error, kJacobianTolerance);
    EXPECT_LT(own_mujoco_error, kJacobianTolerance);
    EXPECT_LT(moveit_mujoco_error, kJacobianTolerance);
  }

  std::cout << std::scientific << std::setprecision(9)
            << "Stage L maximum position error [m]: " << maximum_position_error << '\n'
            << "Stage L maximum orientation error [rad]: " << maximum_orientation_error << '\n'
            << "Stage L maximum Jacobian element error: " << maximum_jacobian_error << '\n';
}

TEST_F(ModelConsistencyTest, MujocoPointAndBodyJacobiansAgreeAtTheBodyOrigin)
{
  const arm_kinematics::JointVector q = configurations().at(1).q;
  setConfiguration(q);
  const int hand_id = api_.name2id(mujoco_model_, mjOBJ_BODY, "hand");
  const Eigen::Vector3d hand_origin = mujocoBodyTransform("hand").translation();
  std::vector<mjtNum> point_jacp(3 * mujoco_model_->nv, 0.0);
  std::vector<mjtNum> point_jacr(3 * mujoco_model_->nv, 0.0);
  std::vector<mjtNum> body_jacp(3 * mujoco_model_->nv, 0.0);
  std::vector<mjtNum> body_jacr(3 * mujoco_model_->nv, 0.0);

  api_.jac(
    mujoco_model_, mujoco_data_, point_jacp.data(), point_jacr.data(),
    hand_origin.data(), hand_id);
  api_.jacBody(
    mujoco_model_, mujoco_data_, body_jacp.data(), body_jacr.data(), hand_id);

  EXPECT_TRUE(
    Eigen::Map<const Eigen::VectorXd>(point_jacp.data(), point_jacp.size()).isApprox(
      Eigen::Map<const Eigen::VectorXd>(body_jacp.data(), body_jacp.size()), 1e-12));
  EXPECT_TRUE(
    Eigen::Map<const Eigen::VectorXd>(point_jacr.data(), point_jacr.size()).isApprox(
      Eigen::Map<const Eigen::VectorXd>(body_jacr.data(), body_jacr.size()), 1e-12));
}

}  // namespace
}  // namespace mujoco_bridge
