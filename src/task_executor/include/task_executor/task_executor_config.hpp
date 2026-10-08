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

#include <array>
#include <cstddef>

#include <rclcpp/node.hpp>

#include "task_executor/cartesian_waypoint_source.hpp"
#include "task_executor/joint_trajectory_planner.hpp"
#include "task_executor/waypoint_source.hpp"
#include "task_executor/fsm.hpp"

namespace task_executor
{

enum class WaypointMode
{
  kDiffIk,
  kKeyframe
};

enum class ObservationSource
{
  kOracle,
  kVision
};

struct PlacementVerification
{
  double box_target_x_m = 0.5;
  double box_target_y_m = 0.3;
  double radius_m = 0.08;
};

struct PlacementTask
{
  // The fixed place target, used without a bin (place.into_bin false): the legacy scene's
  // marker on the table.
  double tcp_target_x_m = 0.5;
  double tcp_target_y_m = 0.3;
  double table_top_z_m = 0.22;
  double hover_height_m = 0.15;
  double tcp_above_box_center_m = 0.05;
  double tool_yaw_rad = 0.0;
  bool align_tool_to_box_yaw = true;

  PickPlaceGeometry geometry() const;
  PlaceTarget fixedPlace() const {return {tcp_target_x_m, tcp_target_y_m, table_top_z_m};}
};

// Waiting for the initial box (and bin) pose to be latched before an episode starts
// (Week 4.1 Stage 7). Vision source only.
struct InitialPoseLatchConfig
{
  std::size_t frames = 5;
  double max_position_spread_m = 0.003;
  double max_yaw_spread_rad = 0.0523598775598;  // 3 degrees
  // Wall time from the reset to the latch; longer than the estimator needs (about 2.3 s).
  double timeout_s = 10.0;
};

struct TaskExecutorConfig
{
  TaskExecutorConfig()
  {
    fsm.place_x_m = verification.box_target_x_m;
    fsm.place_y_m = verification.box_target_y_m;
    fsm.place_region_radius_m = verification.radius_m;
  }

  FsmParams fsm;
  PlacementTask task;
  PlacementVerification verification;
  WaypointMode waypoint_mode = WaypointMode::kDiffIk;
  ObservationSource observation_source = ObservationSource::kOracle;
  bool allow_target_mismatch = false;
  InitialPoseLatchConfig latch;
  // Put the box into the bin (Week 4.1 Stage 8): the place target is the bin's inner floor,
  // from the latched vision bin or, with the oracle source, from the bridge's ground-truth
  // bin pose; the episode waits for it and never falls back to the fixed target. False: the
  // fixed target on the table (the legacy scene has no bin). The demo launch sets it with
  // scene_enabled. Replaces Stage 7's latch.require_bin.
  bool place_into_bin = false;
  // HOME, and where VERIFY returns to (Week 5 Stage 2): a joint target in the diff_ik mode,
  // the same configuration as the bridge's reset keyframe.
  std::array<double, 7> home_joint_positions = kFrankaReadyPose;
  // Limits of every phase's timed arm trajectory (Week 5 Stage 3, ADR 020).
  JointLimits trajectory_limits = kPandaReferenceLimits;
};

const char * waypointModeName(WaypointMode mode);
const char * observationSourceName(ObservationSource source);
TaskExecutorConfig loadTaskExecutorConfig(rclcpp::Node & node);
void validateTaskExecutorConfig(const TaskExecutorConfig & config);

}  // namespace task_executor
