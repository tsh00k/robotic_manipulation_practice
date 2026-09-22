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

#include "arm_kinematics/forward_kinematics.hpp"

#include <Eigen/Geometry>

#include <stdexcept>

namespace arm_kinematics
{

namespace
{

Eigen::Isometry3d jointRotation(const JointModel & joint, double angle)
{
  Eigen::Isometry3d rotation = Eigen::Isometry3d::Identity();
  rotation.linear() = Eigen::AngleAxisd(angle, joint.axis.normalized()).toRotationMatrix();
  return rotation;
}

}  // namespace

ForwardKinematics fk(const ArmModel & model, const JointVector & q)
{
  if (!q.allFinite()) {
    throw std::invalid_argument("Joint positions must all be finite");
  }

  ForwardKinematics result;
  Eigen::Isometry3d current = Eigen::Isometry3d::Identity();
  result.link[0] = current;

  for (std::size_t i = 0; i < kArmDof; ++i) {
    current = current * model.joints[i].origin *
      jointRotation(model.joints[i], q(static_cast<Eigen::Index>(i)));
    result.link[i + 1] = current;
  }

  result.link[8] = current * model.link7_to_link8;
  result.hand = result.link[8] * model.link8_to_hand;
  result.hand_tcp = result.hand * model.hand_to_tcp;
  return result;
}

Jacobian jacobian(const ArmModel & model, const JointVector & q)
{
  const ForwardKinematics transforms = fk(model, q);
  Jacobian result = Jacobian::Zero();
  const Eigen::Vector3d tcp_position = transforms.hand_tcp.translation();
  Eigen::Isometry3d parent = Eigen::Isometry3d::Identity();

  for (std::size_t i = 0; i < kArmDof; ++i) {
    const Eigen::Isometry3d joint_frame = parent * model.joints[i].origin;
    const Eigen::Vector3d joint_position = joint_frame.translation();
    const Eigen::Vector3d axis_world =
      joint_frame.linear() * model.joints[i].axis.normalized();

    result.block<3, 1>(0, static_cast<Eigen::Index>(i)) =
      axis_world.cross(tcp_position - joint_position);
    result.block<3, 1>(3, static_cast<Eigen::Index>(i)) = axis_world;

    parent = joint_frame *
      jointRotation(model.joints[i], q(static_cast<Eigen::Index>(i)));
  }

  return result;
}

}  // namespace arm_kinematics
