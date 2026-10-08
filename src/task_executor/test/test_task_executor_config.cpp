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
  EXPECT_STREQ(observationSourceName(config.observation_source), "oracle");
  EXPECT_NO_THROW(validateTaskExecutorConfig(config));
}

TEST(TaskExecutorConfig, RejectsInvalidGeometry)
{
  TaskExecutorConfig config;
  config.task.tool_yaw_rad = std::numeric_limits<double>::quiet_NaN();
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

TEST(TaskExecutorConfig, RejectsALatchThatCouldNeverBeBuilt)
{
  TaskExecutorConfig config;
  config.latch.frames = 0;  // the latch would read from an empty run
  EXPECT_THROW(validateTaskExecutorConfig(config), std::invalid_argument);
}

TEST(TaskExecutorConfig, RejectsUnmappedVerificationCriterion)
{
  TaskExecutorConfig config;
  config.verification.box_target_x_m = 0.6;
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
  EXPECT_DOUBLE_EQ(defaults.task.tcp_target_x_m, 0.5);
  EXPECT_DOUBLE_EQ(defaults.task.tcp_target_y_m, 0.3);
  EXPECT_DOUBLE_EQ(defaults.verification.box_target_x_m, 0.5);
  EXPECT_DOUBLE_EQ(defaults.verification.box_target_y_m, 0.3);
  EXPECT_DOUBLE_EQ(defaults.fsm.place_x_m, defaults.verification.box_target_x_m);
  EXPECT_DOUBLE_EQ(defaults.fsm.place_y_m, defaults.verification.box_target_y_m);
  EXPECT_DOUBLE_EQ(defaults.fsm.place_region_radius_m, defaults.verification.radius_m);
  EXPECT_DOUBLE_EQ(defaults.fsm.close_settle_s, 2.0);
  EXPECT_EQ(defaults.observation_source, ObservationSource::kOracle);

  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("waypoint_source", "keyframe"),
        rclcpp::Parameter("observation_source", "vision"),
        rclcpp::Parameter("target.place_x_m", 0.6),
        rclcpp::Parameter("verify.place_region_radius_m", 0.03)});
  auto legacy_node = std::make_shared<rclcpp::Node>("config_legacy_test", options);
  const auto legacy = loadTaskExecutorConfig(*legacy_node);
  EXPECT_EQ(legacy.waypoint_mode, WaypointMode::kKeyframe);
  EXPECT_EQ(legacy.observation_source, ObservationSource::kVision);
  EXPECT_DOUBLE_EQ(legacy.task.tcp_target_x_m, 0.6);
  EXPECT_DOUBLE_EQ(legacy.verification.box_target_x_m, 0.43);
  EXPECT_DOUBLE_EQ(legacy.verification.box_target_y_m, 0.31);
  EXPECT_DOUBLE_EQ(legacy.verification.radius_m, 0.03);
  EXPECT_DOUBLE_EQ(legacy.fsm.place_x_m, legacy.verification.box_target_x_m);
  EXPECT_DOUBLE_EQ(legacy.fsm.place_y_m, legacy.verification.box_target_y_m);
  EXPECT_DOUBLE_EQ(legacy.fsm.place_region_radius_m, legacy.verification.radius_m);
}

TEST_F(LoadedTaskExecutorConfig, PlacesIntoTheBinOnlyWhenAsked)
{
  auto default_node = std::make_shared<rclcpp::Node>("config_place_default_test");
  EXPECT_FALSE(loadTaskExecutorConfig(*default_node).place_into_bin);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("place.into_bin", true)});
  auto node = std::make_shared<rclcpp::Node>("config_place_bin_test", options);
  EXPECT_TRUE(loadTaskExecutorConfig(*node).place_into_bin);
}

TEST_F(LoadedTaskExecutorConfig, RejectsUnapprovedIkMismatch)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("target.place_x_m", 0.6)});
  auto node = std::make_shared<rclcpp::Node>("config_mismatch_test", options);
  EXPECT_THROW(loadTaskExecutorConfig(*node), std::invalid_argument);
}

TEST_F(LoadedTaskExecutorConfig, AcceptsIntentionalIkMismatch)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("target.place_x_m", 0.6),
      rclcpp::Parameter("verify.allow_target_mismatch", true)});
  auto node = std::make_shared<rclcpp::Node>("config_experiment_test", options);
  const auto config = loadTaskExecutorConfig(*node);
  EXPECT_TRUE(config.allow_target_mismatch);
  EXPECT_DOUBLE_EQ(config.task.fixedPlace().x, 0.6);
  EXPECT_DOUBLE_EQ(config.verification.box_target_x_m, 0.5);
  EXPECT_DOUBLE_EQ(config.fsm.place_x_m, 0.5);
}

TEST_F(LoadedTaskExecutorConfig, PreservesExplicitMatchingOverrides)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("target.place_x_m", 0.6),
      rclcpp::Parameter("target.place_y_m", 0.2),
      rclcpp::Parameter("verify.place_x_m", 0.6),
      rclcpp::Parameter("verify.place_y_m", 0.2)});
  auto node = std::make_shared<rclcpp::Node>("config_matching_test", options);
  const auto config = loadTaskExecutorConfig(*node);
  EXPECT_DOUBLE_EQ(config.task.fixedPlace().x, 0.6);
  EXPECT_DOUBLE_EQ(config.task.fixedPlace().y, 0.2);
  EXPECT_DOUBLE_EQ(config.fsm.place_x_m, 0.6);
  EXPECT_DOUBLE_EQ(config.fsm.place_y_m, 0.2);
}

}  // namespace
}  // namespace task_executor
