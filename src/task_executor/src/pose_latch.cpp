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

#include "task_executor/pose_latch.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

namespace task_executor
{

namespace
{

// The smallest arc of a circle with the given period that contains every angle.
double circularSpread(const std::deque<LatchSample> & run, double period)
{
  std::vector<double> wrapped;
  wrapped.reserve(run.size());
  for (const auto & sample : run) {
    wrapped.push_back(std::fmod(std::fmod(sample.yaw_rad, period) + period, period));
  }
  std::sort(wrapped.begin(), wrapped.end());
  // The largest empty gap between neighbours (including the wrap-around) is outside the arc.
  double largest_gap = wrapped.front() + period - wrapped.back();
  for (std::size_t i = 1; i < wrapped.size(); ++i) {
    largest_gap = std::max(largest_gap, wrapped[i] - wrapped[i - 1]);
  }
  return period - largest_gap;
}

}  // namespace

PoseLatch::PoseLatch(LatchParams params)
: params_(std::move(params))
{
}

void PoseLatch::reset(uint64_t bridge_session, uint64_t generation)
{
  bound_ = true;
  session_ = bridge_session;
  generation_ = generation;
  last_sequence_ = 0;
  run_.clear();
  pose_.reset();
  status_ = "WAITING:NO_SAMPLE";
}

void PoseLatch::offer(const LatchSample & sample)
{
  if (!bound_ || pose_ || sample.bridge_session != session_ ||
    sample.generation != generation_ || sample.sequence <= last_sequence_)
  {
    return;
  }
  last_sequence_ = sample.sequence;
  if (!sample.measured || !std::isfinite(sample.x) || !std::isfinite(sample.y) ||
    !std::isfinite(sample.z) || !std::isfinite(sample.yaw_rad))
  {
    run_.clear();
    status_ = "WAITING:" + (sample.reason.empty() ? std::string("NOT_MEASURED") : sample.reason);
    return;
  }
  run_.push_back(sample);
  while (run_.size() > params_.frames) {
    run_.pop_front();
  }
  char text[96];
  if (run_.size() < params_.frames) {
    std::snprintf(text, sizeof(text), "WAITING:COLLECTING %zu/%zu", run_.size(), params_.frames);
    status_ = text;
    return;
  }

  const auto [min_x, max_x] = std::minmax_element(
    run_.begin(), run_.end(), [](const LatchSample & a, const LatchSample & b) {return a.x < b.x;});
  const auto [min_y, max_y] = std::minmax_element(
    run_.begin(), run_.end(), [](const LatchSample & a, const LatchSample & b) {return a.y < b.y;});
  const double position_spread = std::max(max_x->x - min_x->x, max_y->y - min_y->y);
  const double yaw_spread = circularSpread(run_, params_.yaw_period_rad);
  if (position_spread > params_.max_position_spread_m || yaw_spread > params_.max_yaw_spread_rad) {
    std::snprintf(
      text, sizeof(text), "WAITING:INCONSISTENT %.1f mm %.1f deg", position_spread * 1000.0,
      yaw_spread * 180.0 / M_PI);
    status_ = text;
    return;
  }
  const LatchSample & newest = run_.back();
  pose_ = LatchedPose{
    newest.x, newest.y, newest.z, newest.yaw_rad, newest.sequence, newest.stamp_s,
    position_spread, yaw_spread};
  status_ = "LATCHED";
}

}  // namespace task_executor
