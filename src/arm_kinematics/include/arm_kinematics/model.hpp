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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <string>

namespace arm_kinematics
{

constexpr std::size_t kArmDof = 7;
constexpr std::size_t kLinkCount = 9;  // link0 through link8.

using JointVector = Eigen::Matrix<double, kArmDof, 1>;
using Jacobian = Eigen::Matrix<double, 6, kArmDof>;

struct JointLimit
{
  double lower = 0.0;
  double upper = 0.0;
};

// The origin is the fixed parent-link -> joint-frame transform at q=0. The axis
// is expressed in that joint frame, matching the URDF convention.
struct JointModel
{
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  JointLimit limits;
};

struct ArmModel
{
  std::array<JointModel, kArmDof> joints;

  // The URDF chain has a fixed joint7 -> link8 transform before the hand.
  Eigen::Isometry3d link7_to_link8 = Eigen::Isometry3d::Identity();

  // The vendor description uses a -45 degree fixed rotation from link8 to hand.
  Eigen::Isometry3d link8_to_hand = Eigen::Isometry3d::Identity();

  // hand_tcp is a fixed, identity-rotation offset from hand.
  Eigen::Isometry3d hand_to_tcp = Eigen::Isometry3d::Identity();
};

struct ForwardKinematics
{
  // link[0] is link0, ..., link[8] is link8. All transforms are world-frame
  // absolute transforms. In this model world and link0 are coincident.
  std::array<Eigen::Isometry3d, kLinkCount> link;
  Eigen::Isometry3d hand = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d hand_tcp = Eigen::Isometry3d::Identity();
};

// Loads the official franka_description parameter tables. The mathematical core
// remains independent of ROS; the caller supplies the two file paths explicitly.
// The fixed hand and TCP transforms come from the same URDF/xacro description.
ArmModel loadFrankaFerModel(
  const std::string & kinematics_yaml_path,
  const std::string & joint_limits_yaml_path = "");

bool withinJointLimits(const ArmModel & model, const JointVector & q, double tolerance = 0.0);

}  // namespace arm_kinematics
