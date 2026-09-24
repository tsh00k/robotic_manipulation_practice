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

#include <Eigen/Geometry>

#include <cstddef>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "arm_kinematics/differential_ik.hpp"
#include "arm_kinematics/forward_kinematics.hpp"

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cerr << "usage: singularity_sweep KINEMATICS_YAML JOINT_LIMITS_YAML OUTPUT_CSV\n";
    return 2;
  }

  try {
    const arm_kinematics::ArmModel model =
      arm_kinematics::loadFrankaFerModel(argv[1], argv[2]);
    arm_kinematics::JointVector start_q;
    start_q << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
    arm_kinematics::JointVector near_singular_q;
    near_singular_q << 0.0, 0.0, 0.0, -0.07, 0.0, 0.07, 0.0;

    std::ofstream output(argv[3]);
    if (!output) {
      throw std::runtime_error("Could not open output CSV");
    }
    output << "sample,fraction,tcp_x,tcp_y,tcp_z,minimum_singular_value,"
      "condition_number,damping,maximum_joint_step,linearized_tracking_error\n";

    constexpr std::size_t kSamples = 81;
    arm_kinematics::DifferentialIkParameters parameters;
    parameters.joint_centering_gain = 0.0;
    const arm_kinematics::TaskVector cartesian_command =
      (arm_kinematics::TaskVector() << 0.001, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    for (std::size_t sample = 0; sample < kSamples; ++sample) {
      const double fraction = static_cast<double>(sample) / (kSamples - 1U);
      const arm_kinematics::JointVector q =
        (1.0 - fraction) * start_q + fraction * near_singular_q;
      const Eigen::Isometry3d tcp = arm_kinematics::fk(model, q).hand_tcp;
      const arm_kinematics::Jacobian geometric_jacobian = arm_kinematics::jacobian(model, q);
      const arm_kinematics::DifferentialIkStep step = arm_kinematics::differentialIkStep(
        model, q, geometric_jacobian, cartesian_command, parameters);
      const double tracking_error =
        (geometric_jacobian * step.delta_q - cartesian_command).norm();
      output << sample << ',' << fraction << ',' << tcp.translation().x() << ','
             << tcp.translation().y() << ',' << tcp.translation().z() << ','
             << step.minimum_singular_value << ',' << step.condition_number << ','
             << step.damping << ',' << step.delta_q.cwiseAbs().maxCoeff() << ','
             << tracking_error << '\n';
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }

  return 0;
}
