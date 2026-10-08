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

#include "mujoco_bridge/trajectory_interpolator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace mujoco_bridge
{

void TrajectoryInterpolator::set(
  std::vector<double> times_s, std::vector<std::vector<double>> positions)
{
  if (times_s.empty() || positions.size() != times_s.size() || times_s.front() != 0.0) {
    throw std::invalid_argument("trajectory: times and positions must match, from 0");
  }
  const std::size_t joints = positions.front().size();
  for (std::size_t k = 0; k < times_s.size(); ++k) {
    if (!std::isfinite(times_s[k]) || (k > 0 && !(times_s[k] > times_s[k - 1])) ||
      positions[k].size() != joints)
    {
      throw std::invalid_argument("trajectory: times must increase and every sample be complete");
    }
    for (const double q : positions[k]) {
      if (!std::isfinite(q)) {
        throw std::invalid_argument("trajectory: positions must be finite");
      }
    }
  }
  times_s_ = std::move(times_s);
  positions_ = std::move(positions);
}

void TrajectoryInterpolator::clear()
{
  times_s_.clear();
  positions_.clear();
}

bool TrajectoryInterpolator::sample(double t, std::vector<double> & q) const
{
  if (times_s_.empty()) {
    return false;
  }
  if (t <= 0.0) {
    q = positions_.front();
    return true;
  }
  if (t >= times_s_.back()) {
    q = positions_.back();
    return true;
  }
  // The first sample later than t; the segment is [k - 1, k].
  const std::size_t k = static_cast<std::size_t>(
    std::upper_bound(times_s_.begin(), times_s_.end(), t) - times_s_.begin());
  const double u = (t - times_s_[k - 1]) / (times_s_[k] - times_s_[k - 1]);
  const auto & p0 = positions_[k - 1];
  const auto & p1 = positions_[k];
  q.resize(p0.size());
  for (std::size_t j = 0; j < p0.size(); ++j) {
    q[j] = p0[j] + u * (p1[j] - p0[j]);
  }
  return true;
}

}  // namespace mujoco_bridge
