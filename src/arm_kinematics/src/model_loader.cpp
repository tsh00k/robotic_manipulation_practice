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

#include "arm_kinematics/model.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace arm_kinematics
{

namespace
{

Eigen::Isometry3d transformFromRpy(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(x, y, z);
  transform.linear() =
    (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  return transform;
}

double scalar(const YAML::Node & node, const char * name, const std::string & context)
{
  if (!node[name]) {
    throw std::runtime_error("Missing " + std::string(name) + " in " + context);
  }
  const double value = node[name].as<double>();
  if (!std::isfinite(value)) {
    throw std::runtime_error("Non-finite " + std::string(name) + " in " + context);
  }
  return value;
}

Eigen::Isometry3d readKinematicTransform(
  const YAML::Node & root, const std::string & joint_name)
{
  const YAML::Node kinematic = root[joint_name]["kinematic"];
  if (!kinematic) {
    throw std::runtime_error("Missing kinematic entry for " + joint_name);
  }
  return transformFromRpy(
    scalar(kinematic, "x", joint_name),
    scalar(kinematic, "y", joint_name),
    scalar(kinematic, "z", joint_name),
    scalar(kinematic, "roll", joint_name),
    scalar(kinematic, "pitch", joint_name),
    scalar(kinematic, "yaw", joint_name));
}

}  // namespace

ArmModel loadFrankaFerModel(
  const std::string & kinematics_yaml_path,
  const std::string & joint_limits_yaml_path)
{
  const YAML::Node kinematics = YAML::LoadFile(kinematics_yaml_path);
  ArmModel model;

  for (std::size_t i = 0; i < kArmDof; ++i) {
    const std::string joint_name = "joint" + std::to_string(i + 1);
    model.joints[i].origin = readKinematicTransform(kinematics, joint_name);
    model.joints[i].axis = Eigen::Vector3d::UnitZ();
    model.joints[i].limits = {
      -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  }

  model.link7_to_link8 = readKinematicTransform(kinematics, "joint8");
  model.link8_to_hand = transformFromRpy(0.0, 0.0, 0.0, 0.0, 0.0, -M_PI / 4.0);
  model.hand_to_tcp = transformFromRpy(0.0, 0.0, 0.1034, 0.0, 0.0, 0.0);

  if (!joint_limits_yaml_path.empty()) {
    const YAML::Node limits = YAML::LoadFile(joint_limits_yaml_path);
    for (std::size_t i = 0; i < kArmDof; ++i) {
      const std::string joint_name = "joint" + std::to_string(i + 1);
      const YAML::Node limit = limits[joint_name]["limit"];
      if (!limit) {
        throw std::runtime_error("Missing limit entry for " + joint_name);
      }
      model.joints[i].limits.lower = scalar(limit, "lower", joint_name);
      model.joints[i].limits.upper = scalar(limit, "upper", joint_name);
      if (model.joints[i].limits.lower > model.joints[i].limits.upper) {
        throw std::runtime_error("Lower limit exceeds upper limit for " + joint_name);
      }
    }
  }

  return model;
}

bool withinJointLimits(const ArmModel & model, const JointVector & q, double tolerance)
{
  if (!q.allFinite() || tolerance < 0.0) {
    return false;
  }
  for (std::size_t i = 0; i < kArmDof; ++i) {
    if (q(static_cast<Eigen::Index>(i)) < model.joints[i].limits.lower - tolerance ||
      q(static_cast<Eigen::Index>(i)) > model.joints[i].limits.upper + tolerance)
    {
      return false;
    }
  }
  return true;
}

}  // namespace arm_kinematics
