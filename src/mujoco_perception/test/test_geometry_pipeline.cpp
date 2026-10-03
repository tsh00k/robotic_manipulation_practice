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
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <limits>

#include <sensor_msgs/msg/camera_info.hpp>

#include "mujoco_perception/geometry_pipeline.hpp"

namespace mujoco_perception
{
namespace
{

pcl::PointCloud<pcl::PointXYZ> transformCloud(
  const pcl::PointCloud<pcl::PointXYZ> & source,
  const Eigen::Matrix3f & rotation,
  const Eigen::Vector3f & translation)
{
  pcl::PointCloud<pcl::PointXYZ> target;
  target.reserve(source.size());
  for (const auto & point : source) {
    const Eigen::Vector3f transformed = rotation * point.getVector3fMap() + translation;
    target.emplace_back(transformed.x(), transformed.y(), transformed.z());
  }
  return target;
}

TEST(KnownCorrespondences, UsesPclSvdToRecoverRigidTransform)
{
  pcl::PointCloud<pcl::PointXYZ> source;
  source.emplace_back(0.0F, 0.0F, 0.0F);
  source.emplace_back(0.1F, 0.0F, 0.0F);
  source.emplace_back(0.0F, 0.2F, 0.0F);
  source.emplace_back(0.0F, 0.0F, 0.3F);
  const Eigen::Matrix3f rotation = (
    Eigen::AngleAxisf(0.4F, Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(-0.2F, Eigen::Vector3f::UnitY())).toRotationMatrix();
  const Eigen::Vector3f translation(0.4F, -0.2F, 0.7F);
  const auto target = transformCloud(source, rotation, translation);

  const auto result = estimateKnownCorrespondences(source, target);
  ASSERT_TRUE(result.valid);
  const Eigen::Matrix3f recovered_rotation = result.transform.block<3, 3>(0, 0);
  const Eigen::Vector3f recovered_translation = result.transform.block<3, 1>(0, 3);
  EXPECT_TRUE(recovered_rotation.isApprox(rotation, 1e-5F));
  EXPECT_TRUE(recovered_translation.isApprox(translation, 1e-5F));
  EXPECT_NEAR(result.rms_residual_m, 0.0, 1e-6);
}

TEST(KnownCorrespondences, RejectsMismatchedPoints)
{
  pcl::PointCloud<pcl::PointXYZ> source;
  pcl::PointCloud<pcl::PointXYZ> target;
  source.emplace_back(0.0F, 0.0F, 0.0F);
  source.emplace_back(1.0F, 0.0F, 0.0F);
  target.emplace_back(0.0F, 1.0F, 0.0F);
  const auto result = estimateKnownCorrespondences(source, target);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection, RejectionReason::kInvalidInput);
}

TEST(KnownCorrespondences, RejectsCollinearPoints)
{
  pcl::PointCloud<pcl::PointXYZ> source;
  pcl::PointCloud<pcl::PointXYZ> target;
  source.emplace_back(0.0F, 0.0F, 0.0F);
  source.emplace_back(0.1F, 0.0F, 0.0F);
  source.emplace_back(0.2F, 0.0F, 0.0F);
  target.emplace_back(0.0F, 0.1F, 0.0F);
  target.emplace_back(0.0F, 0.2F, 0.0F);
  target.emplace_back(0.0F, 0.3F, 0.0F);
  const auto result = estimateKnownCorrespondences(source, target);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.rejection, RejectionReason::kDegenerateCorrespondence);
}

TEST(Segmentation, UsesPclPlaneAndEuclideanClusterStages)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 10;
  info.height = 8;
  info.k[0] = 100.0;
  info.k[4] = 100.0;
  info.k[2] = 4.5;
  info.k[5] = 3.5;
  info.p[0] = 100.0;
  info.p[2] = 4.5;
  info.p[5] = 100.0;
  info.p[6] = 3.5;
  info.p[10] = 1.0;

  std::vector<float> depth(static_cast<std::size_t>(info.width * info.height), 0.22F);
  for (int v = 2; v <= 4; ++v) {
    for (int u = 4; u <= 6; ++u) {
      depth[static_cast<std::size_t>(v * info.width + u)] = 0.30F;
    }
  }
  depth[0] = std::numeric_limits<float>::quiet_NaN();

  SegmentationConfig config;
  config.min_cluster_points = 4;
  config.world_roi.x_min = -1.0;
  config.world_roi.x_max = 1.0;
  config.world_roi.y_min = -1.0;
  config.world_roi.y_max = 1.0;
  config.world_roi.z_min = 0.2;
  config.world_roi.z_max = 0.5;
  const auto result = segmentDepth(
    depth, info, Eigen::Isometry3d::Identity(), config);

  EXPECT_EQ(result.valid_depth_points, 79U);
  EXPECT_EQ(result.plane_points, 70U);
  EXPECT_TRUE(result.target_cluster->empty());
  ASSERT_EQ(result.candidate_clusters.size(), 1U);
  EXPECT_EQ(result.candidate_clusters.front()->size(), 9U);
  EXPECT_EQ(result.rejection, RejectionReason::kNone);
}

TEST(BoxPose, UsesPclObbAndMarksCubeYawAmbiguous)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  for (const float x : {-0.02F, 0.02F}) {
    for (const float y : {-0.02F, 0.02F}) {
      for (const float z : {0.22F, 0.26F}) {
        points.emplace_back(0.5F + x, y, z);
      }
    }
  }
  BoxModel model;
  model.min_inlier_ratio = 0.9;
  const auto result = estimateBoxPose(points, model, 0.22);

  ASSERT_TRUE(result.geometry_valid);
  EXPECT_TRUE(result.orientation_ambiguous);
  EXPECT_NEAR(result.position.x(), 0.5, 1e-4);
  EXPECT_NEAR(result.position.y(), 0.0, 1e-4);
  EXPECT_NEAR(result.position.z(), 0.24, 1e-4);
  EXPECT_NEAR(result.residual_m, 0.0, 1e-5);
}

TEST(BoxPose, AcceptsVisibleTopFaceWithKnownThickness)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  for (const float x : {-0.02F, 0.0F, 0.02F}) {
    for (const float y : {-0.02F, 0.0F, 0.02F}) {
      points.emplace_back(0.5F + x, y, 0.26F);
    }
  }
  const auto result = estimateBoxPose(points, BoxModel{}, 0.22);

  ASSERT_TRUE(result.geometry_valid);
  EXPECT_NEAR(result.position.z(), 0.24, 1e-4);
}

TEST(BoxPose, PartialTopFaceCanPassTaskQualityWithoutFullDimensions)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  for (int x = 0; x < 7; ++x) {
    for (int y = 0; y < 7; ++y) {
      points.emplace_back(0.500F + x * 0.002F, y * 0.002F, 0.260F);
    }
  }
  const auto result = estimateBoxPose(points, BoxModel{}, 0.22);
  ASSERT_TRUE(result.geometry_valid);
  EXPECT_GE(result.confidence, 0.5);
  EXPECT_LE(result.residual_m, 0.005);
  EXPECT_GE(result.inlier_ratio, 0.7);
  // A partial patch cannot reveal the true center: do not claim otherwise.
  EXPECT_NEAR(result.position.x(), 0.506, 1e-4);
  BoxModel unsupported;
  unsupported.anchor_z_to_plane = false;
  EXPECT_FALSE(estimateBoxPose(points, unsupported, 0.22).geometry_valid);
}

TEST(BoxPose, PartialSideUsesKnownWidthToRecoverCenter)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  for (int y = 0; y < 9; ++y) {
    for (int z = 0; z < 9; ++z) {
      points.emplace_back(0.48F, -0.016F + y * 0.004F, 0.23F + z * 0.003F);
    }
  }
  const auto result = estimateBoxPose(points, BoxModel{}, 0.22);
  ASSERT_TRUE(result.geometry_valid);
  EXPECT_NEAR(result.position.x(), 0.50, 1e-4);
  EXPECT_NEAR(result.residual_m, 0.0, 1e-5);
  EXPECT_GE(result.confidence, 0.5);
}

TEST(BoxPose, RejectsTinyPatchAndTableRemnantDespiteKnownBoxPrior)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  for (int x = 0; x < 7; ++x) {
    for (int y = 0; y < 7; ++y) {
      points.emplace_back(0.5F + x * 0.0005F, y * 0.0005F, 0.26F);
    }
  }
  EXPECT_FALSE(estimateBoxPose(points, BoxModel{}, 0.22).geometry_valid);
  points.clear();
  for (int x = 0; x < 7; ++x) {
    for (int y = 0; y < 7; ++y) {
      points.emplace_back(0.5F + x * 0.002F, y * 0.002F, 0.227F);
    }
  }
  const auto result = estimateBoxPose(points, BoxModel{}, 0.22);
  EXPECT_FALSE(result.geometry_valid);
  EXPECT_EQ(result.rejection, RejectionReason::kModelExtentMismatch);
}

TEST(BoxPose, RejectsAClusterWithWrongDimensions)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  points.emplace_back(0.45F, -0.05F, 0.22F);
  points.emplace_back(0.55F, -0.05F, 0.22F);
  points.emplace_back(0.45F, 0.05F, 0.22F);
  points.emplace_back(0.55F, 0.05F, 0.22F);
  const auto result = estimateBoxPose(points, BoxModel{}, 0.22);
  EXPECT_FALSE(result.geometry_valid);
  EXPECT_EQ(result.rejection, RejectionReason::kModelExtentMismatch);
}

TEST(Segmentation, DoesNotRemoveElevatedHorizontalObjectAsSupport)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 8;
  info.height = 8;
  info.k = {100.0, 0.0, 3.5, 0.0, 100.0, 3.5, 0.0, 0.0, 1.0};
  info.p = {100.0, 0.0, 3.5, 0.0, 0.0, 100.0, 3.5, 0.0, 0.0, 0.0, 1.0, 0.0};
  SegmentationConfig config;
  config.world_roi.x_min = -1.0;
  config.world_roi.x_max = 1.0;
  config.world_roi.y_min = -1.0;
  config.world_roi.y_max = 1.0;
  const auto result = segmentDepth(
    std::vector<float>(64, 0.36F), info, Eigen::Isometry3d::Identity(), config);
  EXPECT_EQ(result.plane_points, 0U);
  EXPECT_EQ(result.foreground_points->size(), 64U);
  EXPECT_EQ(result.candidate_clusters.size(), 1U);
}

TEST(BoxPose, ElevatedFullGeometryDisablesSupportAnchorAndTopOnlyCannotMeasure)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  for (const float x : {-0.02F, 0.02F}) {
    for (const float y : {-0.02F, 0.02F}) {
      for (const float z : {0.32F, 0.36F}) {
        points.emplace_back(0.5F + x, y, z);
      }
    }
  }
  BoxModel model;
  model.anchor_z_to_plane = false;
  const auto result = estimateBoxPose(points, model, 0.22);
  ASSERT_TRUE(result.geometry_valid);
  EXPECT_NEAR(result.position.z(), 0.34, 1e-4);
  for (auto & point : points) {
    point.z = 0.36F;
  }
  EXPECT_FALSE(estimateBoxPose(points, model, 0.22).geometry_valid);
}

}  // namespace
}  // namespace mujoco_perception
