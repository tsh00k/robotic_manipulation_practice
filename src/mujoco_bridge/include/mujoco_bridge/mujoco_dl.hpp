#pragma once

// MuJoCo's shared library bundles its own copy of tinyxml2 with default (exported)
// symbol visibility. So does libfastrtps.so, which rclcpp pulls in as a DDS backend
// and loads *after* our executable's normal DT_NEEDED libraries are already resident.
// Because both export the same mangled tinyxml2 symbols, standard dynamic linking lets
// mujoco's copy "win" the process-wide symbol lookup for fastrtps's internal XML
// profile parsing too -- fastrtps then runs with mujoco's (ABI-incompatible) tinyxml2
// object layout and crashes (call through a null vtable entry).
//
// Loading libmujoco.so ourselves via dlopen(RTLD_LOCAL | RTLD_DEEPBIND) keeps its
// symbols out of the global scope entirely, so this never happens. That means we
// cannot link against mujoco at compile time (no target_link_libraries(mujoco::mujoco));
// instead every mj_* entry point we use is resolved with dlsym into a function pointer
// here, and calling code goes through this struct instead of the raw mj_* names.

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
  void (*resetDataKeyframe)(const mjModel * m, mjData * d, int key);
};

// Loads libmujoco.so in isolated (RTLD_LOCAL | RTLD_DEEPBIND) mode and resolves all
// entry points above. Throws std::runtime_error if the library or any symbol is missing.
MujocoApi & loadMujocoApi();

}  // namespace mujoco_bridge
