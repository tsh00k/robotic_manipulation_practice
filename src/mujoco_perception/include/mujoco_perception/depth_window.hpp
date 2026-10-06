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
// A pixel whose valid frames span more than `max_spread_m` (largest minus smallest depth) is
// also NaN. Such a pixel saw two different surfaces within the window, typically a box edge
// that moved by a fraction of a pixel (the box settling after a reset, a little arm motion), so
// that the pixel is the box in some frames and the table behind it in others. Its mean lies
// between the two surfaces and belongs to neither; back-projected it becomes a point floating
// beside the box, which enlarged a 40 mm face to 42.5 mm in the Stage 5 online check. The
// default, 20 mm, is ten standard deviations of the 2 mm noise at which the Stage 3 averaging
// met its acceptance rule, so that noise alone does not reach it (ten samples of 4 mm noise
// span about 12 mm on average and almost never 20), and far below the 51 mm jump at that box
// edge.
//
// Pure data structure: no ROS and no OpenCV, so it is tested on its own.
class DepthWindow
{
public:
  // max_spread_m > 0; infinity switches the spread rule off.
  explicit DepthWindow(
    std::size_t capacity, double min_valid_fraction = 0.5, double max_spread_m = 0.02);

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
  double max_spread_m_;
  std::deque<std::vector<float>> frames_;
};

}  // namespace mujoco_perception
