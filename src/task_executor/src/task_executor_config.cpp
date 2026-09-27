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

#include "task_executor/task_executor_config.hpp"

#include <cmath>
#include <stdexcept>

namespace task_executor
{

const char * waypointModeName(WaypointMode mode)
{
  return mode == WaypointMode::kDiffIk ? "diff_ik" : "keyframe";
}

TaskExecutorConfig loadTaskExecutorConfig(rclcpp::Node & node)
{
  TaskExecutorConfig config{};
  config.fsm.grasp_criteria.box_width_m = node.declare_parameter("grasp.box_width_m", 0.04);
  config.fsm.grasp_criteria.width_epsilon_m = node.declare_parameter(
    "grasp.width_epsilon_m", 0.01);
  config.fsm.grasp_criteria.lift_height_threshold_m = node.declare_parameter(
    "grasp.lift_height_threshold_m", 0.26);
  config.fsm.grasp_criteria.region_radius_m = node.declare_parameter(
    "grasp.region_radius_m", 0.05);
  config.fsm.position_epsilon_rad = node.declare_parameter("fsm.position_epsilon_rad", 0.05);
  config.fsm.grasp_position_epsilon_rad = node.declare_parameter(
    "fsm.grasp_position_epsilon_rad", 0.3);
  config.fsm.velocity_epsilon_rad_s = node.declare_parameter(
    "fsm.velocity_epsilon_rad_s", 0.05);
  config.fsm.min_settle_s = node.declare_parameter("fsm.min_settle_s", 0.5);
  config.fsm.lift_settle_grace_s = node.declare_parameter("fsm.lift_settle_grace_s", 2.0);
  config.fsm.close_settle_s = node.declare_parameter("fsm.close_settle_s", 2.0);
  config.fsm.phase_timeout_s = node.declare_parameter("fsm.phase_timeout_s", 6.0);
  config.fsm.max_retries = node.declare_parameter("fsm.max_retries", 3);

  const std::string mode = node.declare_parameter("waypoint_source", "diff_ik");
  config.geometry.place_x_m = node.declare_parameter("target.place_x_m", 0.5);
  config.geometry.place_y_m = node.declare_parameter("target.place_y_m", 0.3);
  config.geometry.hover_height_m = node.declare_parameter("target.hover_height_m", 0.15);
  config.geometry.place_tcp_above_box_center_m = node.declare_parameter(
    "target.place_tcp_above_box_center_m", 0.05);
  config.geometry.tool_yaw_rad = node.declare_parameter("target.tool_yaw_rad", 0.0);
  const bool legacy = mode == "keyframe";
  config.verification.x_m = node.declare_parameter("verify.place_x_m", legacy ? 0.43 : 0.5);
  config.verification.y_m = node.declare_parameter("verify.place_y_m", legacy ? 0.31 : 0.3);
  config.verification.radius_m = node.declare_parameter("verify.place_region_radius_m", 0.08);
  config.fsm.place_x_m = config.verification.x_m;
  config.fsm.place_y_m = config.verification.y_m;
  config.fsm.place_region_radius_m = config.verification.radius_m;
  if (mode == "diff_ik") {
    config.waypoint_mode = WaypointMode::kDiffIk;
  } else if (legacy) {
    config.waypoint_mode = WaypointMode::kKeyframe;
  } else {
    throw std::invalid_argument("waypoint_source must be diff_ik or keyframe");
  }
  validateTaskExecutorConfig(config);
  return config;
}

void validateTaskExecutorConfig(const TaskExecutorConfig & config)
{
  const auto finite = [](double value) {return std::isfinite(value);};
  if (!finite(config.geometry.place_x_m) || !finite(config.geometry.place_y_m) ||
    !finite(config.geometry.hover_height_m) ||
    !finite(config.geometry.place_tcp_above_box_center_m) ||
    !finite(config.geometry.tool_yaw_rad) || !finite(config.verification.x_m) ||
    !finite(config.verification.y_m) || !finite(config.verification.radius_m))
  {
    throw std::invalid_argument("Task executor geometry contains a non-finite value");
  }
  if (config.geometry.hover_height_m <= 0.0 ||
    config.geometry.place_tcp_above_box_center_m <= 0.0 ||
    config.verification.radius_m <= 0.0)
  {
    throw std::invalid_argument("Task executor geometry contains a non-positive distance");
  }
  if (config.fsm.max_retries < 0 || config.fsm.phase_timeout_s <= 0.0) {
    throw std::invalid_argument("Task executor FSM configuration is invalid");
  }
}

}  // namespace task_executor
