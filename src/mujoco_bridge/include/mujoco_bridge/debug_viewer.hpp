#pragma once

#include <mujoco/mujoco.h>

#include "mujoco_bridge/mujoco_dl.hpp"

// Forward-declared rather than #include <GLFW/glfw3.h>: GLFW types are only ever
// used as opaque pointers/parameters at this boundary, so no caller of this header
// (in particular mujoco_bridge_node.cpp) needs to see GLFW at all.
struct GLFWwindow;

namespace mujoco_bridge
{

// Debug-only visualization, wired directly to whatever mjData the caller is
// currently stepping -- no ROS topic, no decimated /joint_states round-trip, no
// second mjData copy. Intended to be constructed once (behind the
// enable_debug_viewer parameter) and driven from the same thread that owns that
// mjData (mujoco_bridge_node's onTimer(), which already runs single-threaded under
// rclcpp::spin()), so there is no locking here.
//
// render() must be called at a decimated rate (see mujoco_bridge_node's
// debug_viewer_rate_hz), not every physics step: glfwSwapInterval(0) is used
// specifically so a slow display can never stall the physics timer, which means
// nothing else rate-limits render() for you.
class DebugViewer
{
public:
  // model must outlive this object. Throws std::runtime_error if GLFW or the
  // MuJoCo render context fail to initialize (e.g. no DISPLAY) -- same fail-fast
  // style as mujoco_bridge_node's mj_loadXML/mj_makeData failures.
  DebugViewer(const MujocoApi & api, mjModel * model);
  ~DebugViewer();

  DebugViewer(const DebugViewer &) = delete;
  DebugViewer & operator=(const DebugViewer &) = delete;

  // Cheap; safe (and expected) to call every physics step regardless of whether
  // render() is called this step.
  void pollEvents();
  bool shouldClose() const;

  // mjv_updateScene + mjr_render + buffer swap against the given mjData. Call at
  // debug_viewer_rate_hz, not every physics step -- see class comment.
  void render(mjData * data);

private:
  static void mouseButtonCallback(GLFWwindow * window, int button, int action, int mods);
  static void cursorPosCallback(GLFWwindow * window, double xpos, double ypos);
  static void scrollCallback(GLFWwindow * window, double xoffset, double yoffset);

  const MujocoApi & api_;
  mjModel * model_;
  GLFWwindow * window_ = nullptr;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;

  bool button_left_ = false;
  bool button_middle_ = false;
  bool button_right_ = false;
  double last_x_ = 0.0;
  double last_y_ = 0.0;
};

}  // namespace mujoco_bridge
