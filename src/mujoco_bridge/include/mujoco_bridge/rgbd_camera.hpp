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

#include <mujoco/mujoco.h>

#include <cstdint>
#include <vector>

#include "mujoco_bridge/mujoco_dl.hpp"

// Forward declaration keeps GLFW out of the public header. The implementation
// owns the hidden window and includes <GLFW/glfw3.h>; users of RgbdCamera only
// need the opaque handle type.
struct GLFWwindow;

namespace mujoco_bridge
{

class RgbdCamera
{
public:
  RgbdCamera(const MujocoApi & api, mjModel * model, int camera_id, int width, int height);
  ~RgbdCamera();

  RgbdCamera(const RgbdCamera &) = delete;
  RgbdCamera & operator=(const RgbdCamera &) = delete;

  void capture(mjData * data);
  const std::vector<uint8_t> & rgb() const {return rgb_;}
  const std::vector<float> & depth() const {return depth_;}

private:
  const MujocoApi & api_;
  mjModel * model_;
  // Hidden GLFW window whose OpenGL context backs MuJoCo's offscreen renderer.
  GLFWwindow * window_ = nullptr;
  int width_;
  int height_;
  mjvCamera camera_{};   // fixed MJCF camera selection
  mjvOption option_{};   // render-category/options state
  mjvScene scene_{};     // MuJoCo render scene rebuilt from mjData
  mjrContext context_{};  // OpenGL/offscreen framebuffer resources
  std::vector<uint8_t> rgb_;       // ROS row order, RGB interleaved, 3 bytes/pixel
  std::vector<float> depth_;       // ROS row order, metres, optical z-depth
  std::vector<uint8_t> raw_rgb_;   // renderer row order, scratch readback buffer
  std::vector<float> raw_depth_;   // normalized renderer depth, scratch readback buffer
};

}  // namespace mujoco_bridge
