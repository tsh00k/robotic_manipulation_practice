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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <string>
#include <vector>

#include <sensor_msgs/msg/camera_info.hpp>

namespace mujoco_perception
{

struct WorldRoi
{
  double x_min = 0.30;
  double x_max = 0.70;
  double y_min = -0.35;
  double y_max = 0.35;
  double z_min = 0.20;
  double z_max = 0.45;
};

struct SegmentationConfig
{
  double depth_min_m = 0.10;
  double depth_max_m = 2.0;
  int u_min = 0;
  int u_max = -1;
  int v_min = 0;
  int v_max = -1;
  double plane_z_m = 0.22;
  double plane_tolerance_m = 0.006;
  double plane_eps_angle_rad = 0.12;
  double cluster_tolerance_m = 0.012;
  std::size_t min_cluster_points = 20;
  WorldRoi world_roi;
};

struct BoxModel
{
  Eigen::Vector3d half_extents{0.02, 0.02, 0.02};
  double extent_tolerance_m = 0.015;
  double inlier_tolerance_m = 0.006;
  double max_residual_m = 0.008;
  double min_inlier_ratio = 0.70;
  bool anchor_z_to_plane = true;
};

enum class RejectionReason
{
  kNone,
  kInvalidInput,
  kNoValidDepth,
  kNoTargetCluster,
  kTooFewPoints,
  kModelExtentMismatch,
  kDegenerateCorrespondence,
  kLowInlierRatio,
  kHighResidual,
};

const char * rejectionReasonName(RejectionReason reason);

struct SegmentationResult
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points{
    new pcl::PointCloud<pcl::PointXYZ>()};
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cluster{
    new pcl::PointCloud<pcl::PointXYZ>()};
  std::size_t valid_depth_points = 0;
  std::size_t plane_points = 0;
  RejectionReason rejection = RejectionReason::kNone;
};

struct RigidTransformResult
{
  bool valid = false;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  double rms_residual_m = 0.0;
  RejectionReason rejection = RejectionReason::kNone;
};

struct PoseEstimate
{
  bool accepted = false;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  double residual_m = 0.0;
  double inlier_ratio = 0.0;
  double confidence = 0.0;
  std::size_t point_count = 0;
  bool orientation_ambiguous = false;
  RejectionReason rejection = RejectionReason::kNone;
};

// Uses image_geometry for camera projection, PCL PassThrough/SACSegmentation for
// ROI and support-plane removal, and PCL EuclideanClusterExtraction for the target.
SegmentationResult segmentDepth(
  const std::vector<float> & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Isometry3d & world_from_optical,
  const SegmentationConfig & config);

// Uses pcl::registration::TransformationEstimationSVD for known corresponding
// points and validates its output before exposing it to the task layer.
RigidTransformResult estimateKnownCorrespondences(
  const pcl::PointCloud<pcl::PointXYZ> & source,
  const pcl::PointCloud<pcl::PointXYZ> & target);

// Uses PCL's MomentOfInertiaEstimation OBB as the box pose baseline. A square box
// has an unobservable 90-degree yaw symmetry and is reported as ambiguous.
PoseEstimate estimateBoxPose(
  const pcl::PointCloud<pcl::PointXYZ> & cluster,
  const BoxModel & model,
  double plane_z_m);

}  // namespace mujoco_perception
