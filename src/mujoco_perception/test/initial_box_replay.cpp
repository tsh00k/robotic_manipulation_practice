// Copyright 2026 anby
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Replays recorded depth frames through the NEW initial box detector, offline.
//
// The counterpart of baseline_replay.cpp (which runs the existing detector). It feeds each
// group of frames into a DepthWindow, takes the per-pixel mean and runs detectInitialBox() on
// it, exactly as the estimator node does, so that the C++ implementation can be compared with
// the Python prototype (initial_pose_eval.py) on identical inputs. Measurement tool, built only
// with BUILD_TESTING and not installed.
//
// Input (little endian, written by initial_box_compare.py):
//   int32   detections
//   int32   frames_per_detection       the DepthWindow capacity
//   detections x frames_per_detection x ( 240 x 320 float32 )   depth in metres, NaN = invalid
//
// Output, per detection on stdout:
//   detection <i> measured=<0|1> reason=<name> x=<m> y=<m> z=<m> yaw_deg=<deg> valid_px=<n>
//     candidate <k> pixels=<n> along_mm=<mm> across_mm=<mm> yaw_deg=<deg> match=<0|1>
//   bin measured=<0|1> reason=<name> x=<m> y=<m> z=<m> yaw_deg=<deg> valid_px=<n>
//     bincandidate <k> pixels=<n> along_mm=<mm> across_mm=<mm> yaw_deg=<deg> match=<0|1>
//
// Camera model: workcell_camera.hpp. Default InitialBoxConfig and InitialBinConfig, as the node
// uses them.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <vector>

#include "mujoco_perception/depth_window.hpp"
#include "mujoco_perception/initial_bin_detector.hpp"
#include "mujoco_perception/initial_box_detector.hpp"
#include "workcell_camera.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <frames.bin>\n", argv[0]);
    return 2;
  }
  std::ifstream input(argv[1], std::ios::binary);
  int32_t detections = 0;
  int32_t frames_per_detection = 0;
  if (!input.read(reinterpret_cast<char *>(&detections), sizeof(detections)) ||
    !input.read(reinterpret_cast<char *>(&frames_per_detection), sizeof(frames_per_detection)) ||
    detections < 0 || frames_per_detection < 1)
  {
    std::fprintf(stderr, "cannot read the header of %s\n", argv[1]);
    return 2;
  }

  const auto info = mujoco_perception_test::workcellCameraInfo();
  const auto world_from_optical = mujoco_perception_test::workcellWorldFromOptical();
  const mujoco_perception::InitialBoxConfig config;
  const mujoco_perception::InitialBinConfig bin_config;
  const std::size_t pixels = static_cast<std::size_t>(mujoco_perception_test::kImageWidth) *
    mujoco_perception_test::kImageHeight;

  for (int detection = 0; detection < detections; ++detection) {
    mujoco_perception::DepthWindow window(static_cast<std::size_t>(frames_per_detection));
    for (int frame = 0; frame < frames_per_detection; ++frame) {
      std::vector<float> depth(pixels);
      if (!input.read(reinterpret_cast<char *>(depth.data()), pixels * sizeof(float))) {
        std::fprintf(stderr, "file ended inside detection %d\n", detection);
        return 2;
      }
      window.push(depth);
    }
    const auto result =
      mujoco_perception::detectInitialBox(window.mean(), info, world_from_optical, config);
    std::printf(
      "detection %d measured=%d reason=%s x=%.6f y=%.6f z=%.6f yaw_deg=%.4f valid_px=%zu\n",
      detection, result.measured() ? 1 : 0,
      mujoco_perception::initialBoxRejectionName(result.rejection), result.position.x(),
      result.position.y(), result.position.z(), result.yaw_rad * 180.0 / M_PI,
      result.valid_depth_pixels);
    for (std::size_t k = 0; k < result.candidates.size(); ++k) {
      const auto & c = result.candidates[k];
      std::printf(
        "  candidate %zu pixels=%zu along_mm=%.3f across_mm=%.3f yaw_deg=%.4f match=%d\n", k,
        c.pixels, c.side_along_m * 1000.0, c.side_across_m * 1000.0,
        c.yaw_rad * 180.0 / M_PI, c.matches ? 1 : 0);
    }
    const auto bin = mujoco_perception::detectInitialBin(
      window.mean(), info, world_from_optical, bin_config);
    std::printf(
      "  bin measured=%d reason=%s x=%.6f y=%.6f z=%.6f yaw_deg=%.4f valid_px=%zu\n",
      bin.measured() ? 1 : 0, mujoco_perception::initialBinRejectionName(bin.rejection),
      bin.position.x(), bin.position.y(), bin.position.z(), bin.yaw_rad * 180.0 / M_PI,
      bin.valid_depth_pixels);
    for (std::size_t k = 0; k < bin.candidates.size(); ++k) {
      const auto & c = bin.candidates[k];
      std::printf(
        "    bincandidate %zu pixels=%zu along_mm=%.3f across_mm=%.3f yaw_deg=%.4f match=%d\n", k,
        c.pixels, c.side_along_m * 1000.0, c.side_across_m * 1000.0,
        c.yaw_rad * 180.0 / M_PI, c.matches ? 1 : 0);
    }
  }
  return 0;
}
