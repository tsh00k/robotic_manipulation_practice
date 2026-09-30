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

#include <rclcpp/node.hpp>

#include "task_executor/cartesian_waypoint_source.hpp"
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
  double tcp_target_x_m = 0.5;
  double tcp_target_y_m = 0.3;
  double hover_height_m = 0.15;
  double tcp_above_box_center_m = 0.05;
  double tool_yaw_rad = 0.0;

  PickPlaceGeometry geometry() const;
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
  double vision_min_confidence = 0.5;
  double vision_max_residual_m = 0.005;
  double vision_min_inlier_ratio = 0.7;
  bool allow_target_mismatch = false;
};

const char * waypointModeName(WaypointMode mode);
const char * observationSourceName(ObservationSource source);
TaskExecutorConfig loadTaskExecutorConfig(rclcpp::Node & node);
void validateTaskExecutorConfig(const TaskExecutorConfig & config);

}  // namespace task_executor
