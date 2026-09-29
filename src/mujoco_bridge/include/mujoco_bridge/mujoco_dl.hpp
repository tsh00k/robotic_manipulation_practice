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

// libmujoco.so statically links its own (patched, newer) tinyxml2 and re-exports all
// ~223 tinyxml2 symbols with default visibility. libfastrtps.so -- which rclcpp
// dlopens as its DDS backend when the first Node is constructed -- does NOT bundle
// tinyxml2; it imports 16 tinyxml2 symbols and declares DT_NEEDED libtinyxml2.so.9,
// expecting the system copy.
//
// All 16 of those symbols are also exported by libmujoco. If mujoco is a normal
// link-time dependency of the executable it lands in the global symbol scope first,
// so the loader resolves fastrtps's tinyxml2 imports to mujoco's copy instead of
// libtinyxml2.so.9 -- classic symbol interposition. The two copies are not
// ABI-compatible (e.g. mujoco has XMLDocument::Identify(char*, XMLNode**, bool) vs
// the system's 2-arg form, and an extra XMLPrinter ctor param), so fastrtps then
// operates on objects whose layout it disagrees with and dies writing through a
// garbage vtable slot.
//
// dlopen(RTLD_LOCAL | RTLD_DEEPBIND) keeps mujoco's symbols out of the global scope
// entirely, so fastrtps binds to the system tinyxml2 as intended. The cost is that
// we cannot link mujoco at compile time (no target_link_libraries(mujoco::mujoco)):
// every mj_* entry point must be dlsym'd into a function pointer below, and calling
// code goes through this struct rather than the raw mj_* names.
// Full write-up: Job_guides/my_study/week1.md section 6.2.

#include <mujoco/mujoco.h>

namespace mujoco_bridge
{

// Fields are decltype'd off the real mj_*/mju_* declarations rather than hand-typed,
// so a signature that no longer matches mujoco.h is a compile error here instead of
// undefined behavior at the dlsym call site in mujoco_dl.cpp.
struct MujocoApi
{
  decltype(&mj_loadXML) loadXML;
  decltype(&mj_makeData) makeData;
  decltype(&mj_step) step;
  decltype(&mj_deleteData) deleteData;
  decltype(&mj_deleteModel) deleteModel;
  decltype(&mj_name2id) name2id;
  decltype(&mj_id2name) id2name;
  decltype(&mj_resetDataKeyframe) resetDataKeyframe;
  // Recomputes every derived quantity (xpos/xquat, jacobians, contacts, qfrc_*) from
  // the current qpos/qvel *without* advancing time. Needed after a reset: the reset
  // only rewrites the state vector, so anything we publish from mjData before the
  // next mj_step would still be the pre-reset derived values.
  decltype(&mj_forward) forward;
  // Kinematic Jacobians. mj_jac evaluates an arbitrary world-space point fixed
  // to a body; mj_jacBody evaluates the body-frame origin.
  decltype(&mj_jac) jac;
  decltype(&mj_jacBody) jacBody;
  // Quaternion helpers, used to turn absolute body poses (xpos/xquat, relative to
  // world) into the parent-relative transforms TF wants.
  decltype(&mju_negQuat) negQuat;
  decltype(&mju_mulQuat) mulQuat;
  decltype(&mju_rotVecQuat) rotVecQuat;
  // Visualization/render entry points, used only by the optional DebugViewer
  // (debug_viewer.hpp). Exported by the same libmujoco.so as the mj_*/mju_* symbols
  // above -- MuJoCo has no separate render library -- so they go through this same
  // dlopen-isolated struct rather than being linked directly.
  decltype(&mjv_defaultCamera) defaultCamera;
  decltype(&mjv_defaultOption) defaultOption;
  decltype(&mjv_defaultScene) defaultScene;
  decltype(&mjr_defaultContext) defaultContext;
  decltype(&mjv_makeScene) makeScene;
  decltype(&mjr_makeContext) makeContext;
  decltype(&mjv_updateScene) updateScene;
  decltype(&mjr_render) render;
  decltype(&mjr_setBuffer) setBuffer;
  decltype(&mjr_readPixels) readPixels;
  decltype(&mjv_freeScene) freeScene;
  decltype(&mjr_freeContext) freeContext;
  decltype(&mjv_moveCamera) moveCamera;
};

// Loads libmujoco.so in isolated (RTLD_LOCAL | RTLD_DEEPBIND) mode and resolves all
// entry points above. Throws std::runtime_error if the library or any symbol is missing.
MujocoApi & loadMujocoApi();

}  // namespace mujoco_bridge
