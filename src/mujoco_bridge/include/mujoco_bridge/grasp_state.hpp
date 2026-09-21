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

#include "mujoco_bridge/mujoco_dl.hpp"

namespace mujoco_bridge
{

// Sum of the two finger joints' qpos. Each finger's own qpos is its slide
// displacement from fully closed (0) toward fully open (0.04), so the sum is the
// total finger-to-finger opening -- the same convention as the real Franka gripper's
// "width", and the reason ~/gripper_command divides its `position` field by two
// before writing ctrl (see mujoco_bridge_node.cpp onGripperCommand).
double gripperWidth(const mjData * d, int finger1_qpos_adr, int finger2_qpos_adr);

// True if mjData::contact[0..ncon) has an active contact between body_a and body_b,
// checked in both geom orderings since MuJoCo does not guarantee which side of a
// pair ends up as geom[0] vs geom[1]. Needs mjModel (for geom_bodyid) and mjData
// (for the live contact list produced by the last mj_forward/mj_step), so unlike
// grasp_criteria.hpp's classifyGrasp this cannot be unit tested without loading a
// model -- Layer 2 in week2.md Stage F's four-layer scheme, not Layer 1.
bool bodiesInContact(const mjModel * m, const mjData * d, int body_a, int body_b);

}  // namespace mujoco_bridge
