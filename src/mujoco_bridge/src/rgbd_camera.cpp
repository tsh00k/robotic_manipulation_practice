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

#include "mujoco_bridge/rgbd_camera.hpp"

#include <GLFW/glfw3.h>

#include <cmath>
#include <stdexcept>

#include "mujoco_bridge/camera_geometry.hpp"

namespace mujoco_bridge
{

RgbdCamera::RgbdCamera(
  const MujocoApi & api, mjModel * model, int camera_id, int width, int height)
: api_(api), model_(model), width_(width), height_(height),
  rgb_(width * height * 3), depth_(width * height),
  raw_rgb_(width * height * 3), raw_depth_(width * height)
{
  // MuJoCo's renderer is an OpenGL renderer. GLFW is used here only to create
  // an OpenGL context and an offscreen-capable framebuffer; it is not drawing
  // the scene or publishing ROS messages. The debug viewer has its own GLFW
  // path, while this camera deliberately uses a hidden context.
  if (!glfwInit()) {
    throw std::runtime_error("glfwInit failed for RGB-D camera (DISPLAY unavailable?)");
  }
  // A hidden GLFW window gives MuJoCo a current OpenGL context without opening
  // a user-visible GUI window. The requested width/height also determine the
  // size of the pixel buffers that readPixels() will fill below.
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  window_ = glfwCreateWindow(width, height, "rgbd offscreen context", nullptr, nullptr);
  glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
  if (!window_) {
    throw std::runtime_error("glfwCreateWindow failed for RGB-D camera");
  }
  // MuJoCo rendering calls operate on the current OpenGL context. This must be
  // repeated in capture() because another renderer may have made a different
  // context current since the previous frame.
  glfwMakeContextCurrent(window_);

  // These are MuJoCo-side render descriptions, not ROS camera messages:
  //   camera_ selects the fixed camera from the MJCF by ID;
  //   option_ controls which geom categories are rendered;
  //   scene_ is MuJoCo's CPU-side render scene;
  //   context_ owns the OpenGL/offscreen framebuffer state.
  api_.defaultCamera(&camera_);
  camera_.type = mjCAMERA_FIXED;
  camera_.fixedcamid = camera_id;
  api_.defaultOption(&option_);
  api_.defaultScene(&scene_);
  api_.defaultContext(&context_);
  // 2000 is the maximum number of geoms allocated in MuJoCo's render scene,
  // not the camera resolution. It leaves room for the Panda, table, object and
  // marker to be submitted to the renderer; a larger scene would need this
  // capacity increased independently of width_/height_.
  api_.makeScene(model_, &scene_, 2000);
  // Font scale affects MuJoCo's render context resources, not RGB-D pixel size.
  api_.makeContext(model_, &context_, mjFONTSCALE_100);
  if (context_.offWidth < width || context_.offHeight < height) {
    // readPixels() cannot read a viewport larger than the framebuffer MuJoCo
    // allocated. Failing at construction is safer than silently returning a
    // cropped or out-of-bounds image later.
    throw std::runtime_error("MuJoCo offscreen framebuffer is smaller than RGB-D resolution");
  }
  // MuJoCo returns a normalized depth-buffer value. capture() converts it to
  // metric z-depth using the model's near/far clipping planes before exposing
  // the public depth() buffer in metres.
  context_.readDepthMap = mjDEPTH_ZERONEAR;
}

RgbdCamera::~RgbdCamera()
{
  if (window_) {
    // Scene/context destruction touches OpenGL resources, so the same context
    // must be current while MuJoCo frees them. Destroy the GLFW window last.
    glfwMakeContextCurrent(window_);
    api_.freeScene(&scene_);
    api_.freeContext(&context_);
    glfwDestroyWindow(window_);
  }
}

void RgbdCamera::capture(mjData * data)
{
  glfwMakeContextCurrent(window_);
  api_.setBuffer(mjFB_OFFSCREEN, &context_);
  const mjrRect viewport{0, 0, width_, height_};
  api_.updateScene(model_, data, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
  api_.render(viewport, &scene_, &context_);
  api_.readPixels(raw_rgb_.data(), raw_depth_.data(), viewport, &context_);

  const double near_m = model_->vis.map.znear * model_->stat.extent;
  const double far_m = model_->vis.map.zfar * model_->stat.extent;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const int src = (height_ - 1 - y) * width_ + x;
      const int dst = y * width_ + x;
      for (int channel = 0; channel < 3; ++channel) {
        rgb_[3 * dst + channel] = raw_rgb_[3 * src + channel];
      }
      depth_[dst] = static_cast<float>(metricDepth(raw_depth_[src], near_m, far_m));
    }
  }
}

}  // namespace mujoco_bridge
