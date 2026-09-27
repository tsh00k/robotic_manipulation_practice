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

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "task_executor/task_executor_config.hpp"

namespace task_executor
{
namespace
{

TEST(TaskExecutorConfig, DefaultsAreValid)
{
  const TaskExecutorConfig config{};
  EXPECT_STREQ(waypointModeName(config.waypoint_mode), "diff_ik");
  EXPECT_NO_THROW(validateTaskExecutorConfig(config));
}

TEST(TaskExecutorConfig, RejectsInvalidGeometry)
{
  TaskExecutorConfig config;
  config.geometry.tool_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validateTaskExecutorConfig(config), std::invalid_argument);

  config = TaskExecutorConfig{};
  config.verification.radius_m = 0.0;
  EXPECT_THROW(validateTaskExecutorConfig(config), std::invalid_argument);
}

TEST(TaskExecutorConfig, RejectsInvalidFsm)
{
  TaskExecutorConfig config;
  config.fsm.max_retries = -1;
  EXPECT_THROW(validateTaskExecutorConfig(config), std::invalid_argument);
}

class LoadedTaskExecutorConfig : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(LoadedTaskExecutorConfig, PreservesModeDefaultsAndOverrides)
{
  auto default_node = std::make_shared<rclcpp::Node>("config_default_test");
  const auto defaults = loadTaskExecutorConfig(*default_node);
  EXPECT_EQ(defaults.waypoint_mode, WaypointMode::kDiffIk);
  EXPECT_DOUBLE_EQ(defaults.verification.x_m, 0.5);
  EXPECT_DOUBLE_EQ(defaults.verification.y_m, 0.3);
  EXPECT_DOUBLE_EQ(defaults.fsm.place_x_m, defaults.verification.x_m);
  EXPECT_DOUBLE_EQ(defaults.fsm.place_y_m, defaults.verification.y_m);
  EXPECT_DOUBLE_EQ(defaults.fsm.place_region_radius_m, defaults.verification.radius_m);
  EXPECT_DOUBLE_EQ(defaults.fsm.close_settle_s, 2.0);

  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("waypoint_source", "keyframe"),
        rclcpp::Parameter("target.place_x_m", 0.6),
        rclcpp::Parameter("verify.place_region_radius_m", 0.03)});
  auto legacy_node = std::make_shared<rclcpp::Node>("config_legacy_test", options);
  const auto legacy = loadTaskExecutorConfig(*legacy_node);
  EXPECT_EQ(legacy.waypoint_mode, WaypointMode::kKeyframe);
  EXPECT_DOUBLE_EQ(legacy.geometry.place_x_m, 0.6);
  EXPECT_DOUBLE_EQ(legacy.verification.x_m, 0.43);
  EXPECT_DOUBLE_EQ(legacy.verification.y_m, 0.31);
  EXPECT_DOUBLE_EQ(legacy.verification.radius_m, 0.03);
  EXPECT_DOUBLE_EQ(legacy.fsm.place_x_m, legacy.verification.x_m);
  EXPECT_DOUBLE_EQ(legacy.fsm.place_y_m, legacy.verification.y_m);
  EXPECT_DOUBLE_EQ(legacy.fsm.place_region_radius_m, legacy.verification.radius_m);
}

}  // namespace
}  // namespace task_executor
