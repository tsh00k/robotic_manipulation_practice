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

#include <Eigen/Core>

#include <cstddef>

namespace mujoco_perception
{

// Why a detection from one depth image failed. Shared by the initial box and the initial bin
// detectors, which differ only in the numbers they look for (Week 4.1 Stages 5 and 6).
enum class DetectionRejection
{
  kNone,                       // the object was measured
  kInvalidInput,               // the depth image does not match the camera info size
  kNoValidDepth,               // the image has no usable depth at all
  kNoBandPixels,               // nothing in the height band forms a block
  kNoMatchingRectangle,        // blocks exist but none has the object's two sides
  kSeveralMatchingRectangles,  // more than one block looks like the object
  kNoFloorPixels,              // bin only: no depth in the middle of the bin to read its floor
};

// One block of the height band, whether or not it was taken for the object. Reported so that a
// miss can be explained from the message alone (which blocks, how big, how far off).
struct BlockCandidate
{
  std::size_t pixels = 0;
  // Rectangle sides, in metres: along the reported yaw direction and across it.
  double side_along_m = 0.0;
  double side_across_m = 0.0;
  Eigen::Vector2d center_xy = Eigen::Vector2d::Zero();
  double yaw_rad = 0.0;
  bool matches = false;
};

}  // namespace mujoco_perception
