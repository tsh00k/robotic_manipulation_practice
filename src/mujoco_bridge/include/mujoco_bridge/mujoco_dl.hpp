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

struct MujocoApi
{
  mjModel * (*loadXML)(const char * filename, const mjVFS * vfs, char * error, int error_sz);
  mjData * (*makeData)(const mjModel * m);
  void (*step)(const mjModel * m, mjData * d);
  void (*deleteData)(mjData * d);
  void (*deleteModel)(mjModel * m);
  int (*name2id)(const mjModel * m, int type, const char * name);
  const char * (*id2name)(const mjModel * m, int type, int id);
  void (*resetDataKeyframe)(const mjModel * m, mjData * d, int key);
  // Quaternion helpers, used to turn absolute body poses (xpos/xquat, relative to
  // world) into the parent-relative transforms TF wants. Array parameters decay to
  // pointers, so these signatures are ABI-identical to the mju_* declarations.
  void (*negQuat)(mjtNum * res, const mjtNum * quat);
  void (*mulQuat)(mjtNum * res, const mjtNum * quat1, const mjtNum * quat2);
  void (*rotVecQuat)(mjtNum * res, const mjtNum * vec, const mjtNum * quat);
};

// Loads libmujoco.so in isolated (RTLD_LOCAL | RTLD_DEEPBIND) mode and resolves all
// entry points above. Throws std::runtime_error if the library or any symbol is missing.
MujocoApi & loadMujocoApi();

}  // namespace mujoco_bridge
