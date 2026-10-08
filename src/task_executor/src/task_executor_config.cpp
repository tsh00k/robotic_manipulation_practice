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

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace task_executor
{

const char * waypointModeName(WaypointMode mode)
{
  return mode == WaypointMode::kDiffIk ? "diff_ik" : "keyframe";
}

const char * observationSourceName(ObservationSource source)
{
  return source == ObservationSource::kOracle ? "oracle" : "vision";
}

PickPlaceGeometry PlacementTask::geometry() const
{
  PickPlaceGeometry result;
  result.hover_height_m = hover_height_m;
  result.place_tcp_above_box_center_m = tcp_above_box_center_m;
  result.tool_yaw_rad = tool_yaw_rad;
  result.align_tool_to_box_yaw = align_tool_to_box_yaw;
  return result;
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
  config.fsm.close_settle_s = node.declare_parameter(
    "fsm.close_settle_s", config.fsm.close_settle_s);
  config.fsm.close_after_attach_s = node.declare_parameter(
    "fsm.close_after_attach_s", config.fsm.close_after_attach_s);
  config.fsm.phase_timeout_s = node.declare_parameter("fsm.phase_timeout_s", 6.0);
  config.fsm.max_retries = node.declare_parameter("fsm.max_retries", 3);

  const std::string mode = node.declare_parameter("waypoint_source", "diff_ik");
  const std::string observation_source = node.declare_parameter("observation_source", "oracle");
  if (observation_source == "oracle") {
    config.observation_source = ObservationSource::kOracle;
  } else if (observation_source == "vision") {
    config.observation_source = ObservationSource::kVision;
  } else {
    throw std::invalid_argument("observation_source must be oracle or vision");
  }
  config.latch.frames = static_cast<std::size_t>(node.declare_parameter(
      "latch.frames", static_cast<int>(config.latch.frames)));
  config.latch.max_position_spread_m = node.declare_parameter(
    "latch.max_position_spread_m", config.latch.max_position_spread_m);
  config.latch.max_yaw_spread_rad = node.declare_parameter(
    "latch.max_yaw_spread_deg", config.latch.max_yaw_spread_rad * 180.0 / M_PI) * M_PI / 180.0;
  config.latch.timeout_s = node.declare_parameter("latch.timeout_s", config.latch.timeout_s);
  config.place_into_bin = node.declare_parameter("place.into_bin", config.place_into_bin);
  const std::vector<double> home = node.declare_parameter(
    "home.joint_positions",
    std::vector<double>(config.home_joint_positions.begin(), config.home_joint_positions.end()));
  if (home.size() != config.home_joint_positions.size()) {
    throw std::invalid_argument("home.joint_positions must have 7 values");
  }
  std::copy(home.begin(), home.end(), config.home_joint_positions.begin());
  const auto declareJointArray = [&node](const std::string & name, std::array<double, 7> & values) {
      const std::vector<double> given = node.declare_parameter(
        name, std::vector<double>(values.begin(), values.end()));
      if (given.size() != values.size()) {
        throw std::invalid_argument(name + " must have 7 values");
      }
      std::copy(given.begin(), given.end(), values.begin());
    };
  declareJointArray("trajectory.max_velocity", config.trajectory_limits.max_velocity);
  declareJointArray("trajectory.max_acceleration", config.trajectory_limits.max_acceleration);
  config.task.tcp_target_x_m = node.declare_parameter("target.place_x_m", 0.5);
  config.task.tcp_target_y_m = node.declare_parameter("target.place_y_m", 0.3);
  config.task.hover_height_m = node.declare_parameter("target.hover_height_m", 0.15);
  config.task.tcp_above_box_center_m = node.declare_parameter(
    "target.place_tcp_above_box_center_m", 0.05);
  config.task.tool_yaw_rad = node.declare_parameter("target.tool_yaw_rad", 0.0);
  config.task.align_tool_to_box_yaw = node.declare_parameter(
    "target.align_tool_to_box_yaw", config.task.align_tool_to_box_yaw);
  const bool legacy = mode == "keyframe";
  config.verification.box_target_x_m = node.declare_parameter(
    "verify.place_x_m", legacy ? 0.43 : 0.5);
  config.verification.box_target_y_m = node.declare_parameter(
    "verify.place_y_m", legacy ? 0.31 : 0.3);
  config.verification.radius_m = node.declare_parameter("verify.place_region_radius_m", 0.08);
  config.allow_target_mismatch = node.declare_parameter("verify.allow_target_mismatch", false);
  config.fsm.place_x_m = config.verification.box_target_x_m;
  config.fsm.place_y_m = config.verification.box_target_y_m;
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
  if (!finite(config.task.tcp_target_x_m) || !finite(config.task.tcp_target_y_m) ||
    !finite(config.task.hover_height_m) ||
    !finite(config.task.tcp_above_box_center_m) ||
    !finite(config.task.tool_yaw_rad) || !finite(config.verification.box_target_x_m) ||
    !finite(config.verification.box_target_y_m) || !finite(config.verification.radius_m))
  {
    throw std::invalid_argument("Task executor geometry contains a non-finite value");
  }
  if (config.task.hover_height_m <= 0.0 ||
    config.task.tcp_above_box_center_m <= 0.0 ||
    config.verification.radius_m <= 0.0)
  {
    throw std::invalid_argument("Task executor geometry contains a non-positive distance");
  }
  for (std::size_t i = 0; i < 7; ++i) {
    if (!finite(config.trajectory_limits.max_velocity[i]) ||
      !finite(config.trajectory_limits.max_acceleration[i]) ||
      config.trajectory_limits.max_velocity[i] <= 0.0 ||
      config.trajectory_limits.max_acceleration[i] <= 0.0)
    {
      throw std::invalid_argument("Arm trajectory limits must be finite and positive");
    }
  }
  if (config.fsm.max_retries < 0 || config.fsm.phase_timeout_s <= 0.0) {
    throw std::invalid_argument("Task executor FSM configuration is invalid");
  }
  if (config.latch.frames < 1 || !finite(config.latch.max_position_spread_m) ||
    !finite(config.latch.max_yaw_spread_rad) || !finite(config.latch.timeout_s) ||
    config.latch.max_position_spread_m <= 0.0 || config.latch.max_yaw_spread_rad <= 0.0 ||
    config.latch.timeout_s <= 0.0)
  {
    throw std::invalid_argument("Initial pose latch configuration is invalid");
  }
  if (config.fsm.place_x_m != config.verification.box_target_x_m ||
    config.fsm.place_y_m != config.verification.box_target_y_m ||
    config.fsm.place_region_radius_m != config.verification.radius_m)
  {
    throw std::invalid_argument("FSM placement criterion differs from verification target");
  }
  if (config.waypoint_mode == WaypointMode::kDiffIk && !config.allow_target_mismatch &&
    (config.task.tcp_target_x_m != config.verification.box_target_x_m ||
    config.task.tcp_target_y_m != config.verification.box_target_y_m))
  {
    throw std::invalid_argument(
            "IK task and verification XY differ; set verify.allow_target_mismatch:=true "
            "for an intentional mismatch");
  }
}

}  // namespace task_executor
