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

struct PlacementVerification
{
  double x_m = 0.5;
  double y_m = 0.3;
  double radius_m = 0.08;
};

struct TaskExecutorConfig
{
  FsmParams fsm;
  PickPlaceGeometry geometry;
  PlacementVerification verification;
  WaypointMode waypoint_mode = WaypointMode::kDiffIk;
};

const char * waypointModeName(WaypointMode mode);
TaskExecutorConfig loadTaskExecutorConfig(rclcpp::Node & node);
void validateTaskExecutorConfig(const TaskExecutorConfig & config);

}  // namespace task_executor
