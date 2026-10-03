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

#include "mujoco_perception/geometry_pipeline.hpp"

#include <Eigen/SVD>

#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace mujoco_perception
{
namespace
{

bool finitePoint(const pcl::PointXYZ & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double pointToSurfaceResidual(const Eigen::Vector3d & local, const Eigen::Vector3d & half)
{
  const Eigen::Array3d absolute = local.array().abs();
  const Eigen::Array3d excess = (absolute - half.array()).max(0.0);
  if (excess.maxCoeff() > 0.0) {
    return excess.matrix().norm();
  }
  return (absolute - half.array()).abs().minCoeff();
}

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

std::vector<double> sortedValues(const Eigen::Vector3d & value)
{
  std::vector<double> result{value.x(), value.y(), value.z()};
  std::sort(result.begin(), result.end());
  return result;
}

}  // namespace

const char * rejectionReasonName(RejectionReason reason)
{
  switch (reason) {
    case RejectionReason::kNone: return "NONE";
    case RejectionReason::kInvalidInput: return "INVALID_INPUT";
    case RejectionReason::kNoValidDepth: return "NO_VALID_DEPTH";
    case RejectionReason::kNoTargetCluster: return "NO_TARGET_CLUSTER";
    case RejectionReason::kTooFewPoints: return "TOO_FEW_POINTS";
    case RejectionReason::kModelExtentMismatch: return "MODEL_EXTENT_MISMATCH";
    case RejectionReason::kDegenerateCorrespondence: return "DEGENERATE_CORRESPONDENCE";
    case RejectionReason::kLowInlierRatio: return "LOW_INLIER_RATIO";
    case RejectionReason::kHighResidual: return "HIGH_RESIDUAL";
    case RejectionReason::kMissingRobotTransform: return "MISSING_ROBOT_TRANSFORM";
  }
  return "UNKNOWN";
}

SegmentationResult segmentDepth(
  const std::vector<float> & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Isometry3d & world_from_optical,
  const SegmentationConfig & config)
{
  SegmentationResult result;
  if (camera_info.width == 0 || camera_info.height == 0 || camera_info.k[0] <= 0.0 ||
    camera_info.k[4] <= 0.0 || depth.size() != static_cast<std::size_t>(camera_info.width) *
    static_cast<std::size_t>(camera_info.height) || !world_from_optical.matrix().allFinite())
  {
    result.rejection = RejectionReason::kInvalidInput;
    return result;
  }

  const int u_min = std::max(0, config.u_min);
  const int u_max = std::min(
    static_cast<int>(camera_info.width) - 1,
    config.u_max < 0 ? static_cast<int>(camera_info.width) - 1 : config.u_max);
  const int v_min = std::max(0, config.v_min);
  const int v_max = std::min(
    static_cast<int>(camera_info.height) - 1,
    config.v_max < 0 ? static_cast<int>(camera_info.height) - 1 : config.v_max);
  if (u_min > u_max || v_min > v_max || config.depth_min_m <= 0.0 ||
    config.depth_max_m <= config.depth_min_m)
  {
    result.rejection = RejectionReason::kInvalidInput;
    return result;
  }

  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info);
  pcl::PointCloud<pcl::PointXYZ>::Ptr optical_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  optical_cloud->reserve(static_cast<std::size_t>((u_max - u_min + 1) * (v_max - v_min + 1)));
  for (int v = v_min; v <= v_max; ++v) {
    for (int u = u_min; u <= u_max; ++u) {
      const float raw_depth = depth[static_cast<std::size_t>(v) * camera_info.width + u];
      if (!std::isfinite(raw_depth) || raw_depth < config.depth_min_m ||
        raw_depth > config.depth_max_m)
      {
        continue;
      }
      const cv::Point3d ray = camera_model.projectPixelTo3dRay(cv::Point2d(u, v));
      const pcl::PointXYZ point(
        static_cast<float>(ray.x * raw_depth), static_cast<float>(ray.y * raw_depth), raw_depth);
      if (finitePoint(point)) {
        optical_cloud->push_back(point);
      }
    }
  }
  result.valid_depth_points = optical_cloud->size();
  if (optical_cloud->empty()) {
    result.rejection = RejectionReason::kNoValidDepth;
    return result;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*optical_cloud, *world_cloud, world_from_optical.matrix().cast<float>());

  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>(*world_cloud));
  for (const auto & range : {
      std::pair<const char *, std::pair<float, float>>{"x", {
          static_cast<float>(config.world_roi.x_min), static_cast<float>(config.world_roi.x_max)}},
      std::pair<const char *, std::pair<float, float>>{"y", {
          static_cast<float>(config.world_roi.y_min), static_cast<float>(config.world_roi.y_max)}},
      std::pair<const char *, std::pair<float, float>>{"z", {
          static_cast<float>(config.world_roi.z_min), static_cast<float>(config.world_roi.z_max)}}
    })
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(roi_cloud);
    pass.setFilterFieldName(range.first);
    pass.setFilterLimits(range.second.first, range.second.second);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pass.filter(*filtered);
    roi_cloud = filtered;
  }
  if (roi_cloud->empty()) {
    result.rejection = RejectionReason::kNoTargetCluster;
    return result;
  }

  pcl::SACSegmentation<pcl::PointXYZ> plane;
  plane.setOptimizeCoefficients(true);
  plane.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  plane.setMethodType(pcl::SAC_RANSAC);
  plane.setAxis(Eigen::Vector3f::UnitZ());
  plane.setEpsAngle(static_cast<float>(config.plane_eps_angle_rad));
  plane.setDistanceThreshold(static_cast<float>(config.plane_tolerance_m));
  plane.setInputCloud(roi_cloud);
  pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
  plane.segment(*plane_indices, *plane_coefficients);
  // Only remove the configured support plane, never an elevated box face.
  if (plane_coefficients->values.size() >= 4 &&
    (std::abs(plane_coefficients->values[2]) < 1e-6F ||
    std::abs(
      -plane_coefficients->values[3] / plane_coefficients->values[2] -
      config.plane_z_m) > config.plane_tolerance_m))
  {
    plane_indices->indices.clear();
  }
  result.plane_points = plane_indices->indices.size();

  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground(new pcl::PointCloud<pcl::PointXYZ>());
  if (plane_indices->indices.empty()) {
    // PCL's RANSAC model can reject a very small or perfectly regular fixture.
    // The configured support height is a valid deterministic fallback for this
    // simulation and still uses PCL's PassThrough filter for the operation.
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(roi_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(
      static_cast<float>(config.plane_z_m - config.plane_tolerance_m),
      static_cast<float>(config.plane_z_m + config.plane_tolerance_m));
    pcl::PointCloud<pcl::PointXYZ>::Ptr support(new pcl::PointCloud<pcl::PointXYZ>());
    pass.filter(*support);
    result.plane_points = support->size();
    pass.setNegative(true);
    pass.filter(*foreground);
  } else {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(roi_cloud);
    extract.setIndices(plane_indices);
    extract.setNegative(true);
    extract.filter(*foreground);
  }
  result.foreground_points = foreground;
  if (foreground->empty()) {
    result.rejection = RejectionReason::kNoTargetCluster;
    return result;
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(foreground);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
  clustering.setClusterTolerance(static_cast<float>(config.cluster_tolerance_m));
  clustering.setMinClusterSize(static_cast<int>(config.min_cluster_points));
  clustering.setMaxClusterSize(static_cast<int>(foreground->size()));
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(foreground);
  std::vector<pcl::PointIndices> clusters;
  clustering.extract(clusters);
  for (const auto & indices : clusters) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidate(new pcl::PointCloud<pcl::PointXYZ>());
    for (const int index : indices.indices) {
      candidate->push_back((*foreground)[static_cast<std::size_t>(index)]);
    }
    result.candidate_clusters.push_back(candidate);
  }
  if (clusters.empty()) {
    result.rejection = RejectionReason::kNoTargetCluster;
    return result;
  }
  return result;
}

RigidTransformResult estimateKnownCorrespondences(
  const pcl::PointCloud<pcl::PointXYZ> & source,
  const pcl::PointCloud<pcl::PointXYZ> & target)
{
  RigidTransformResult result;
  if (source.size() != target.size() || source.size() < 3 || source.empty()) {
    result.rejection = RejectionReason::kInvalidInput;
    return result;
  }
  if (!std::all_of(source.begin(), source.end(), finitePoint) ||
    !std::all_of(target.begin(), target.end(), finitePoint))
  {
    result.rejection = RejectionReason::kInvalidInput;
    return result;
  }

  Eigen::Vector3f source_centroid = Eigen::Vector3f::Zero();
  Eigen::Vector3f target_centroid = Eigen::Vector3f::Zero();
  for (std::size_t i = 0; i < source.size(); ++i) {
    source_centroid += source[i].getVector3fMap();
    target_centroid += target[i].getVector3fMap();
  }
  source_centroid /= static_cast<float>(source.size());
  target_centroid /= static_cast<float>(target.size());
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  for (std::size_t i = 0; i < source.size(); ++i) {
    covariance +=
      (source[i].getVector3fMap() - source_centroid) *
      (target[i].getVector3fMap() - target_centroid).transpose();
  }
  const Eigen::JacobiSVD<Eigen::Matrix3f> rank_check(covariance);
  if (rank_check.singularValues()(1) <= rank_check.singularValues()(0) * 1e-6F) {
    result.rejection = RejectionReason::kDegenerateCorrespondence;
    return result;
  }

  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> estimator;
  estimator.estimateRigidTransformation(source, target, result.transform);
  if (!result.transform.allFinite() ||
    std::abs(result.transform.block<3, 3>(0, 0).determinant() - 1.0F) > 1e-3F)
  {
    result.rejection = RejectionReason::kDegenerateCorrespondence;
    return result;
  }
  double squared_error = 0.0;
  for (std::size_t i = 0; i < source.size(); ++i) {
    const Eigen::Vector4f transformed = result.transform * source[i].getVector4fMap();
    const Eigen::Vector3f error = transformed.head<3>() - target[i].getVector3fMap().head<3>();
    squared_error += static_cast<double>(error.squaredNorm());
  }
  result.rms_residual_m = std::sqrt(squared_error / static_cast<double>(source.size()));
  result.valid = std::isfinite(result.rms_residual_m);
  if (!result.valid) {
    result.rejection = RejectionReason::kInvalidInput;
  }
  return result;
}

PoseEstimate estimateBoxPose(
  const pcl::PointCloud<pcl::PointXYZ> & cluster,
  const BoxModel & model,
  double plane_z_m)
{
  PoseEstimate result;
  result.point_count = cluster.size();
  if (cluster.size() < 3 || !model.half_extents.allFinite() ||
    (model.half_extents.array() <= 0.0).any() || model.extent_tolerance_m < 0.0 ||
    model.inlier_tolerance_m < 0.0 || model.max_residual_m <= 0.0 ||
    !std::isfinite(model.min_visible_extent_m) || model.min_visible_extent_m <= 0.0 ||
    !std::isfinite(model.confidence_reference_points) || model.confidence_reference_points <= 0.0 ||
    model.min_inlier_ratio < 0.0 || model.min_inlier_ratio > 1.0 || !std::isfinite(plane_z_m))
  {
    result.rejection = RejectionReason::kInvalidInput;
    return result;
  }
  if (!std::all_of(cluster.begin(), cluster.end(), finitePoint)) {
    result.rejection = RejectionReason::kInvalidInput;
    return result;
  }

  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> obb;
  obb.setInputCloud(cluster.makeShared());
  obb.compute();
  pcl::PointXYZ min_point;
  pcl::PointXYZ max_point;
  pcl::PointXYZ obb_position;
  Eigen::Matrix3f obb_rotation;
  obb.getOBB(min_point, max_point, obb_position, obb_rotation);
  const Eigen::Vector3d obb_extent =
    (max_point.getVector3fMap() - min_point.getVector3fMap()).cast<double>().cwiseAbs();
  const std::vector<double> measured = sortedValues(obb_extent);
  const std::vector<double> expected = sortedValues(2.0 * model.half_extents);
  for (std::size_t i = 0; i < measured.size(); ++i) {
    result.model_conflict = result.model_conflict ||
      measured[i] > expected[i] + model.extent_tolerance_m;
  }
  // Occlusion shrinks observed extents; a supported partial surface is not a
  // smaller object. Keep the upper model bound and require two spatial axes,
  // but supply missing dimensions from the known box instead of rejecting them.
  if (result.model_conflict) {
    result.rejection = RejectionReason::kModelExtentMismatch;
    return result;
  }
  const std::size_t first_dimension = model.anchor_z_to_plane ? 1 : 0;
  for (std::size_t i = first_dimension; i < measured.size(); ++i) {
    const double minimum = model.anchor_z_to_plane ? model.min_visible_extent_m :
      expected[i] - model.extent_tolerance_m;
    if (measured[i] < minimum) {
      result.rejection = RejectionReason::kModelExtentMismatch;
      return result;
    }
  }

  Eigen::Matrix3d rotation = obb_rotation.cast<double>();
  Eigen::Vector3d position = obb_position.getVector3fMap().cast<double>();
  result.orientation_ambiguous =
    std::abs(model.half_extents.x() - model.half_extents.y()) <= 1e-6;
  if (result.orientation_ambiguous) {
    pcl::PointXYZ min_world;
    pcl::PointXYZ max_world;
    pcl::getMinMax3D(cluster, min_world, max_world);
    // A horizontal patch below the center cannot be a visible top face of
    // the supported cube. Do not fit support-plane remnants as box bottoms.
    if (model.anchor_z_to_plane &&
      max_world.z - min_world.z <= model.inlier_tolerance_m &&
      max_world.z < plane_z_m + model.half_extents.z())
    {
      result.rejection = RejectionReason::kModelExtentMismatch;
      return result;
    }
    rotation = Eigen::Matrix3d::Identity();
    position.x() = (min_world.x + max_world.x) * 0.5;
    position.y() = (min_world.y + max_world.y) * 0.5;
    if (model.anchor_z_to_plane) {
      position.z() = plane_z_m + model.half_extents.z();
      // A visible side is a box boundary, not the middle of the observed patch.
      // Test centers consistent with either observed boundary and the known size.
      // Top-only ties retain the patch midpoint: hidden XY is not observable.
      const Eigen::Vector3d midpoint = position;
      double best_error = std::numeric_limits<double>::infinity();
      for (const double x : {midpoint.x(), min_world.x + model.half_extents.x(),
          max_world.x - model.half_extents.x()})
      {
        for (const double y : {midpoint.y(), min_world.y + model.half_extents.y(),
            max_world.y - model.half_extents.y()})
        {
          const Eigen::Vector3d center(x, y, midpoint.z());
          double error = 0.0;
          for (const auto & point : cluster) {
            const double residual = pointToSurfaceResidual(
              point.getVector3fMap().cast<double>() - center, model.half_extents);
            error += residual * residual;
          }
          if (error < best_error - 1e-12) {
            best_error = error;
            position = center;
          }
        }
      }
    }
  }

  std::size_t inliers = 0;
  double squared_residual = 0.0;
  for (const pcl::PointXYZ & point : cluster) {
    const Eigen::Vector3d local = rotation.transpose() *
      (point.getVector3fMap().cast<double>() - position);
    const Eigen::Array3d excess =
      (local.array().abs() - model.half_extents.array() - model.inlier_tolerance_m).max(0.0);
    if (excess.maxCoeff() <= 0.0) {
      ++inliers;
    }
    const double residual = pointToSurfaceResidual(local, model.half_extents);
    squared_residual += residual * residual;
  }
  result.position = position;
  result.orientation = Eigen::Quaterniond(rotation);
  result.inlier_ratio = static_cast<double>(inliers) / static_cast<double>(cluster.size());
  result.residual_m = std::sqrt(squared_residual / static_cast<double>(cluster.size()));
  const double point_score = clamp01(
    static_cast<double>(cluster.size()) / model.confidence_reference_points);
  const double residual_score = clamp01(1.0 - result.residual_m / model.max_residual_m);
  result.confidence = point_score * result.inlier_ratio * residual_score;
  if (result.inlier_ratio < model.min_inlier_ratio) {
    result.rejection = RejectionReason::kLowInlierRatio;
  } else if (result.residual_m > model.max_residual_m) {
    result.rejection = RejectionReason::kHighResidual;
  } else {
    result.geometry_valid = true;
  }
  return result;
}

}  // namespace mujoco_perception
