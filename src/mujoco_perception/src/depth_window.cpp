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

#include "mujoco_perception/depth_window.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mujoco_perception
{

DepthWindow::DepthWindow(std::size_t capacity, double min_valid_fraction, double max_spread_m)
: capacity_(capacity), min_valid_fraction_(min_valid_fraction), max_spread_m_(max_spread_m)
{
  if (capacity == 0) {
    throw std::invalid_argument("DepthWindow capacity must be at least 1");
  }
  if (!(min_valid_fraction >= 0.0 && min_valid_fraction <= 1.0)) {
    throw std::invalid_argument("DepthWindow min_valid_fraction must be in [0, 1]");
  }
  if (!(max_spread_m > 0.0)) {  // also rejects NaN
    throw std::invalid_argument("DepthWindow max_spread_m must be positive");
  }
}

void DepthWindow::push(const std::vector<float> & depth)
{
  if (!frames_.empty() && frames_.front().size() != depth.size()) {
    frames_.clear();
  }
  frames_.push_back(depth);
  while (frames_.size() > capacity_) {
    frames_.pop_front();
  }
}

void DepthWindow::clear()
{
  frames_.clear();
}

std::vector<float> DepthWindow::mean() const
{
  if (frames_.empty()) {
    return {};
  }
  const std::size_t pixels = frames_.front().size();
  const double required = min_valid_fraction_ * static_cast<double>(frames_.size());
  std::vector<float> result(pixels, std::numeric_limits<float>::quiet_NaN());
  for (std::size_t i = 0; i < pixels; ++i) {
    double sum = 0.0;
    float smallest = std::numeric_limits<float>::infinity();
    float largest = -std::numeric_limits<float>::infinity();
    std::size_t valid = 0;
    for (const auto & frame : frames_) {
      const float value = frame[i];
      if (std::isfinite(value) && value > 0.0F) {
        sum += value;
        smallest = std::min(smallest, value);
        largest = std::max(largest, value);
        ++valid;
      }
    }
    if (valid > 0 && static_cast<double>(valid) >= required &&
      static_cast<double>(largest) - static_cast<double>(smallest) <= max_spread_m_)
    {
      result[i] = static_cast<float>(sum / static_cast<double>(valid));
    }
  }
  return result;
}

}  // namespace mujoco_perception
