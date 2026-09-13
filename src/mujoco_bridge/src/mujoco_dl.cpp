#include "mujoco_bridge/mujoco_dl.hpp"

#include <dlfcn.h>

#include <stdexcept>
#include <string>

namespace mujoco_bridge
{

namespace
{
constexpr const char * kMujocoLibPath = "/opt/mujoco-3.3.7/lib/libmujoco.so.3.3.7";

template<typename FuncPtr>
void resolve(void * handle, const char * symbol, FuncPtr & out)
{
  out = reinterpret_cast<FuncPtr>(dlsym(handle, symbol));
  if (!out) {
    throw std::runtime_error(std::string("dlsym failed for ") + symbol + ": " + dlerror());
  }
}
}  // namespace

MujocoApi & loadMujocoApi()
{
  static MujocoApi api = []() {
    void * handle = dlopen(kMujocoLibPath, RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND); // RTLD_LOCAL | RTLD_DEEPBIND keeps mujoco's symbols out of the global scope, avoiding conflicts with fastrtps's tinyxml2
    if (!handle) {
      throw std::runtime_error(std::string("dlopen failed for ") + kMujocoLibPath + ": " + dlerror());
    }

    MujocoApi api;
    resolve(handle, "mj_loadXML", api.loadXML);
    resolve(handle, "mj_makeData", api.makeData);
    resolve(handle, "mj_step", api.step);
    resolve(handle, "mj_deleteData", api.deleteData);
    resolve(handle, "mj_deleteModel", api.deleteModel);
    resolve(handle, "mj_name2id", api.name2id);
    resolve(handle, "mj_id2name", api.id2name);
    resolve(handle, "mj_resetDataKeyframe", api.resetDataKeyframe);
    resolve(handle, "mju_negQuat", api.negQuat);
    resolve(handle, "mju_mulQuat", api.mulQuat);
    resolve(handle, "mju_rotVecQuat", api.rotVecQuat);
    return api;
  }();
  return api;
}

}  // namespace mujoco_bridge
