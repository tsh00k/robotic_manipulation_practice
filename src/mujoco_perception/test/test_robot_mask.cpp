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
#include <geometric_shapes/mesh_operations.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

#include <sensor_msgs/msg/camera_info.hpp>

#include "mujoco_perception/robot_mask.hpp"

namespace mujoco_perception
{
namespace
{

sensor_msgs::msg::CameraInfo cameraInfo()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 64;
  info.height = 64;
  info.k[0] = 100.0;
  info.k[4] = 100.0;
  info.k[2] = 32.0;
  info.k[5] = 32.0;
  return info;
}

RobotMesh frontSquare()
{
  RobotMesh mesh;
  mesh.frame = "hand";
  mesh.geometry.reset(shapes::createMeshFromShape(shapes::Box(0.06, 0.06, 0.002)));
  return mesh;
}

Eigen::Isometry3d handPose()
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().z() = 1.0;
  return pose;
}

TEST(RobotMask, RemovesOnlyDepthMatchingTheRobotSurface)
{
  const auto info = cameraInfo();
  std::vector<float> depth(info.width * info.height, 1.0F);
  depth[32 * info.width + 32] = 0.8F;
  depth[32 * info.width + 31] = 1.2F;
  const std::map<std::string, Eigen::Isometry3d> links{
    {"hand", handPose()}};
  RobotMaskConfig config;
  RobotMaskFilter filter({frontSquare()});
  const auto result = filter.filter(
    depth, info, Eigen::Isometry3d::Identity(), links, config);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.projected_pixels, 0U);
  EXPECT_NEAR(result.predicted_depth[32 * info.width + 32], 0.999F, 0.003F);
  EXPECT_NEAR(result.predicted_depth[32 * info.width + 31], 0.999F, 0.003F);
  EXPECT_EQ(result.mask[32 * info.width + 32], 0U);
  EXPECT_FLOAT_EQ(result.filtered_depth[32 * info.width + 32], 0.8F);
  EXPECT_EQ(result.mask[32 * info.width + 31], 0U);
  EXPECT_FLOAT_EQ(result.filtered_depth[32 * info.width + 31], 1.2F);
  EXPECT_EQ(result.mask[31 * info.width + 32], 255U);
  EXPECT_TRUE(std::isnan(result.filtered_depth[31 * info.width + 32]));
  EXPECT_EQ(result.mismatch_pixels, 1U);
  EXPECT_EQ(result.comparison_pixels, result.masked_pixels + 2U);
}

TEST(RobotMask, RejectsMissingTransformButPreservesDepthBehindRobot)
{
  const auto info = cameraInfo();
  const std::vector<float> depth(info.width * info.height, 1.2F);
  const std::map<std::string, Eigen::Isometry3d> links{
    {"hand", handPose()}};
  RobotMaskConfig config;

  RobotMaskFilter filter({frontSquare()});
  EXPECT_FALSE(filter.filter(depth, info, Eigen::Isometry3d::Identity(), {}, config).valid);
  const auto background = filter.filter(
    depth, info, Eigen::Isometry3d::Identity(), links, config);
  EXPECT_TRUE(background.valid);
  EXPECT_EQ(background.masked_pixels, 0U);
  EXPECT_GT(background.comparison_pixels, 0U);
  EXPECT_EQ(background.mismatch_pixels, background.comparison_pixels);
  EXPECT_FLOAT_EQ(background.filtered_depth[32 * info.width + 32], 1.2F);
}

TEST(RobotMask, PreservesAnObjectInFrontOfAdjacentMovingLinks)
{
  const auto info = cameraInfo();
  std::vector<float> depth(info.width * info.height, 1.0F);
  depth[32 * info.width + 35] = 0.8F;
  RobotMesh finger = frontSquare();
  finger.frame = "left_finger";
  Eigen::Isometry3d finger_pose = handPose();
  finger_pose.translation().x() = 0.03;
  const std::map<std::string, Eigen::Isometry3d> links{
    {"hand", handPose()}, {"left_finger", finger_pose}};
  RobotMaskFilter filter({frontSquare(), finger});
  const auto result = filter.filter(
    depth, info, Eigen::Isometry3d::Identity(), links, {});

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.mask[32 * info.width + 32], 255U);
  EXPECT_EQ(result.mask[32 * info.width + 35], 0U);
  EXPECT_FLOAT_EQ(result.filtered_depth[32 * info.width + 35], 0.8F);
}

TEST(RobotMask, DoesNotMaskEmptyBackground)
{
  const auto info = cameraInfo();
  std::vector<float> depth(info.width * info.height, std::numeric_limits<float>::quiet_NaN());
  const std::map<std::string, Eigen::Isometry3d> links{
    {"hand", handPose()}};
  RobotMaskFilter filter({frontSquare()});
  const auto result = filter.filter(
    depth, info, Eigen::Isometry3d::Identity(), links, {});
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.masked_pixels, 0U);
  EXPECT_EQ(result.comparison_pixels, 0U);
  EXPECT_EQ(result.mismatch_pixels, 0U);
}

TEST(RobotMask, UpdatesLinkPoseAndRejectsCalibrationChanges)
{
  RobotMaskFilter filter({frontSquare()});
  const auto first_info = cameraInfo();
  const std::vector<float> first_depth(first_info.width * first_info.height, 1.0F);
  const std::map<std::string, Eigen::Isometry3d> first_links{{"hand", handPose()}};
  const auto first = filter.filter(
    first_depth, first_info, Eigen::Isometry3d::Identity(), first_links, {});
  ASSERT_TRUE(first.valid);
  EXPECT_EQ(first.mask[32 * first_info.width + 32], 255U);

  Eigen::Isometry3d moved = handPose();
  moved.translation().x() = 0.2;
  const std::map<std::string, Eigen::Isometry3d> second_links{{"hand", moved}};
  const auto moved_frame = filter.filter(
    first_depth, first_info, Eigen::Isometry3d::Identity(), second_links, {});
  ASSERT_TRUE(moved_frame.valid);
  std::size_t moved_max_column = 0;
  for (std::size_t index = 0; index < moved_frame.mask.size(); ++index) {
    if (moved_frame.mask[index] == 255) {
      moved_max_column = std::max(moved_max_column, index % first_info.width);
    }
  }
  EXPECT_GE(moved_max_column, 49U);
  EXPECT_LE(moved_max_column, 55U);

  auto second_info = first_info;
  second_info.width = 96;
  second_info.height = 96;
  second_info.k[2] = 48.0;
  second_info.k[5] = 48.0;
  const std::vector<float> second_depth(second_info.width * second_info.height, 1.0F);
  EXPECT_THROW(
    filter.filter(
      second_depth, second_info, Eigen::Isometry3d::Identity(), second_links, {}),
    std::runtime_error);
  auto changed_intrinsics = first_info;
  changed_intrinsics.k[2] += 1.0;
  EXPECT_THROW(
    filter.filter(
      first_depth, changed_intrinsics, Eigen::Isometry3d::Identity(), second_links, {}),
    std::runtime_error);
}

}  // namespace
}  // namespace mujoco_perception
