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
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

namespace task_executor
{

// One estimate of an object's pose from the perception node, reduced to what the latch needs.
struct LatchSample
{
  uint64_t bridge_session = 0;
  uint64_t generation = 0;
  uint64_t sequence = 0;   // the bridge sample the estimate belongs to; must increase
  double stamp_s = 0.0;
  bool measured = false;
  std::string reason;      // why it is not measured, for the status text
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw_rad = 0.0;
};

struct LatchParams
{
  // Consecutive measured estimates that must agree.
  std::size_t frames = 5;
  double max_position_spread_m = 0.003;
  double max_yaw_spread_rad = 0.0523598775598;  // 3 degrees
  // The object's yaw repeats after this angle: pi / 2 for the square box, pi for the bin.
  double yaw_period_rad = 1.5707963267949;
};

struct LatchedPose
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw_rad = 0.0;
  uint64_t sequence = 0;   // of the estimate that was latched (the newest of the agreeing ones)
  double stamp_s = 0.0;
  double position_spread_m = 0.0;  // largest x or y range over the agreeing estimates
  double yaw_spread_rad = 0.0;     // smallest arc containing their yaws
};

// Latches an object's initial pose once enough consecutive estimates agree (Week 4.1 Stage 7).
// Pure rule, no ROS. After reset(session, generation) it only looks at samples of that exact
// lifecycle; a sample of another session or generation, or one whose sequence does not
// increase, is ignored and neither counts nor interrupts. A sample that is not measured
// interrupts the run. Once latched, the pose does not change until the next reset().
class PoseLatch
{
public:
  explicit PoseLatch(LatchParams params);

  // A new episode or retry: forget everything and bind to this lifecycle.
  void reset(uint64_t bridge_session, uint64_t generation);

  void offer(const LatchSample & sample);

  bool latched() const {return pose_.has_value();}
  const std::optional<LatchedPose> & pose() const {return pose_;}

  // "LATCHED", or "WAITING:<why>" where why is the reason of the last unmeasured sample,
  // "COLLECTING <n>/<frames>", or "INCONSISTENT <mm> mm <deg> deg".
  const std::string & status() const {return status_;}

private:
  LatchParams params_;
  bool bound_ = false;
  uint64_t session_ = 0;
  uint64_t generation_ = 0;
  uint64_t last_sequence_ = 0;
  std::deque<LatchSample> run_;
  std::optional<LatchedPose> pose_;
  std::string status_ = "WAITING:NOT_BOUND";
};

}  // namespace task_executor
