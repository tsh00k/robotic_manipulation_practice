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

#include <cstddef>
#include <deque>
#include <vector>

namespace mujoco_perception
{

// A sliding window over the last `capacity` depth images of a STATIC scene, and their per-pixel
// mean. Averaging N frames lowers independent depth noise by sqrt(N); it does nothing about an
// error that is the same in every frame (a sensor bias, a fixed pattern), and it blurs
// anything that moves during the window, so the caller decides when the scene may be assumed
// static (Week 4.1 Stage 5, docs/adr/018).
//
// A pixel is "valid" in a frame when its depth is finite and positive. The mean of a pixel is
// taken over the frames in which it is valid; if it is valid in fewer than
// `min_valid_fraction` of the frames currently in the window it is NaN, so that a pixel that
// flickers in and out (for instance at the edge of a masked robot link) does not turn into a
// measurement made from one lucky frame.
//
// Pure data structure: no ROS and no OpenCV, so it is tested on its own.
class DepthWindow
{
public:
  explicit DepthWindow(std::size_t capacity, double min_valid_fraction = 0.5);

  // Append a frame, dropping the oldest when the window is full. A frame whose size differs
  // from the frames already held starts a new window (the camera changed): the old frames are
  // not comparable.
  void push(const std::vector<float> & depth);

  void clear();

  std::size_t size() const {return frames_.size();}
  std::size_t capacity() const {return capacity_;}
  bool full() const {return frames_.size() == capacity_;}

  // Per-pixel mean over the valid frames. Empty if the window is empty.
  std::vector<float> mean() const;

private:
  std::size_t capacity_;
  double min_valid_fraction_;
  std::deque<std::vector<float>> frames_;
};

}  // namespace mujoco_perception
