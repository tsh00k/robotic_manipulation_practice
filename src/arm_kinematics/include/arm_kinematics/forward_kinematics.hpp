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

#include "arm_kinematics/model.hpp"

namespace arm_kinematics
{

// Computes world-frame transforms for every arm link, hand, and hand_tcp.
// Rejects non-finite q; finite values outside joint limits remain mathematically
// valid and can be checked separately with withinJointLimits().
ForwardKinematics fk(const ArmModel & model, const JointVector & q);

// Computes the geometric Jacobian of hand_tcp in world coordinates. Rows are
// [vx, vy, vz, wx, wy, wz], so J * qdot is the TCP spatial twist. Input
// validation follows fk().
Jacobian jacobian(const ArmModel & model, const JointVector & q);

}  // namespace arm_kinematics
