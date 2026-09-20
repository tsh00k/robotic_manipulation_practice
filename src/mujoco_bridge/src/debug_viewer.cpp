#include "mujoco_bridge/debug_viewer.hpp"

#include <GLFW/glfw3.h>

#include <stdexcept>

namespace mujoco_bridge
{

namespace
{
DebugViewer * self(GLFWwindow * window)
{
  return static_cast<DebugViewer *>(glfwGetWindowUserPointer(window));
}
}  // namespace

DebugViewer::DebugViewer(const MujocoApi & api, mjModel * model)
: api_(api), model_(model)
{
  if (!glfwInit()) {
    throw std::runtime_error("glfwInit failed for debug viewer (no DISPLAY?)");
  }
  window_ = glfwCreateWindow(1200, 900, "mujoco_bridge debug viewer", nullptr, nullptr);
  if (!window_) {
    glfwTerminate();
    throw std::runtime_error("glfwCreateWindow failed for debug viewer");
  }
  glfwMakeContextCurrent(window_);
  // No vsync: the swap below must never block the physics thread that drives
  // onTimer(). debug_viewer_rate_hz decimation in mujoco_bridge_node, not this
  // swap call, is what limits render frequency -- see class comment.
  glfwSwapInterval(0);

  glfwSetWindowUserPointer(window_, this);
  glfwSetMouseButtonCallback(window_, &DebugViewer::mouseButtonCallback);
  glfwSetCursorPosCallback(window_, &DebugViewer::cursorPosCallback);
  glfwSetScrollCallback(window_, &DebugViewer::scrollCallback);

  api_.defaultCamera(&cam_);
  api_.defaultOption(&opt_);
  api_.defaultScene(&scn_);
  api_.defaultContext(&con_);
  api_.makeScene(model_, &scn_, 2000);
  api_.makeContext(model_, &con_, mjFONTSCALE_150);
}

DebugViewer::~DebugViewer()
{
  api_.freeScene(&scn_);
  api_.freeContext(&con_);
  // glfwTerminate() after this destructor's window crashes on Linux with
  // proprietary NVIDIA drivers -- ported from the upstream MuJoCo sample
  // (CPP/Chapter2-view&step/basic.cc in the mujoco_learning tutorial repo), which
  // carries the same guard for the same reason.
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
}

void DebugViewer::pollEvents()
{
  glfwPollEvents();
}

bool DebugViewer::shouldClose() const
{
  return glfwWindowShouldClose(window_);
}

void DebugViewer::render(mjData * data)
{
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

  api_.updateScene(model_, data, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
  api_.render(viewport, &scn_, &con_);

  glfwSwapBuffers(window_);
}

void DebugViewer::mouseButtonCallback(GLFWwindow * window, int, int, int)
{
  DebugViewer * viewer = self(window);
  viewer->button_left_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  viewer->button_middle_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  viewer->button_right_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  glfwGetCursorPos(window, &viewer->last_x_, &viewer->last_y_);
}

void DebugViewer::cursorPosCallback(GLFWwindow * window, double xpos, double ypos)
{
  DebugViewer * viewer = self(window);
  if (!viewer->button_left_ && !viewer->button_middle_ && !viewer->button_right_) {
    return;
  }

  const double dx = xpos - viewer->last_x_;
  const double dy = ypos - viewer->last_y_;
  viewer->last_x_ = xpos;
  viewer->last_y_ = ypos;

  int width, height;
  glfwGetWindowSize(window, &width, &height);

  const bool mod_shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

  mjtMouse action;
  if (viewer->button_right_) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (viewer->button_left_) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  viewer->api_.moveCamera(viewer->model_, action, dx / height, dy / height, &viewer->scn_,
    &viewer->cam_);
}

void DebugViewer::scrollCallback(GLFWwindow * window, double, double yoffset)
{
  DebugViewer * viewer = self(window);
  viewer->api_.moveCamera(
    viewer->model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &viewer->scn_, &viewer->cam_);
}

}  // namespace mujoco_bridge
