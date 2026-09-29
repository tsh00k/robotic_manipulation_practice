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

#include <Eigen/Geometry>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <set>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <manipulation_interfaces/msg/bridge_observation.hpp>
#include <manipulation_interfaces/msg/vision_object_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include "mujoco_perception/geometry_pipeline.hpp"

namespace mujoco_perception
{
namespace
{

using Stamp = int64_t;
using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;
using CameraInfoConstPtr = sensor_msgs::msg::CameraInfo::ConstSharedPtr;
using ObservationConstPtr = manipulation_interfaces::msg::BridgeObservation::ConstSharedPtr;

Stamp stampKey(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<Stamp>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

Eigen::Isometry3d eigenTransform(const geometry_msgs::msg::TransformStamped & message)
{
  const auto & t = message.transform.translation;
  const auto & q = message.transform.rotation;
  Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  if (!rotation.coeffs().allFinite() || rotation.norm() <= 1e-12) {
    throw std::runtime_error("TF contains an invalid quaternion");
  }
  rotation.normalize();
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = rotation.toRotationMatrix();
  result.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  return result;
}

}  // namespace

class ObjectPoseEstimatorNode : public rclcpp::Node
{
public:
  ObjectPoseEstimatorNode()
  : Node("object_pose_estimator"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    config_.depth_min_m = declare_parameter("depth_min_m", config_.depth_min_m);
    config_.depth_max_m = declare_parameter("depth_max_m", config_.depth_max_m);
    config_.plane_z_m = declare_parameter("plane_z_m", config_.plane_z_m);
    config_.plane_tolerance_m = declare_parameter(
      "plane_tolerance_m", config_.plane_tolerance_m);
    config_.cluster_tolerance_m = declare_parameter(
      "cluster_tolerance_m", config_.cluster_tolerance_m);
    config_.min_cluster_points = static_cast<std::size_t>(declare_parameter(
        "min_cluster_points", static_cast<int>(config_.min_cluster_points)));
    config_.world_roi.x_min = declare_parameter("roi.x_min", config_.world_roi.x_min);
    config_.world_roi.x_max = declare_parameter("roi.x_max", config_.world_roi.x_max);
    config_.world_roi.y_min = declare_parameter("roi.y_min", config_.world_roi.y_min);
    config_.world_roi.y_max = declare_parameter("roi.y_max", config_.world_roi.y_max);
    config_.world_roi.z_min = declare_parameter("roi.z_min", config_.world_roi.z_min);
    config_.world_roi.z_max = declare_parameter("roi.z_max", config_.world_roi.z_max);

    box_model_.half_extents = Eigen::Vector3d(
      declare_parameter("box_size_x_m", 0.04) * 0.5,
      declare_parameter("box_size_y_m", 0.04) * 0.5,
      declare_parameter("box_size_z_m", 0.04) * 0.5);
    box_model_.extent_tolerance_m = declare_parameter(
      "box_extent_tolerance_m", box_model_.extent_tolerance_m);
    box_model_.inlier_tolerance_m = declare_parameter(
      "box_inlier_tolerance_m", box_model_.inlier_tolerance_m);
    box_model_.max_residual_m = declare_parameter(
      "max_residual_m", box_model_.max_residual_m);
    box_model_.min_inlier_ratio = declare_parameter(
      "min_inlier_ratio", box_model_.min_inlier_ratio);
    box_model_.anchor_z_to_plane = declare_parameter(
      "anchor_z_to_plane", box_model_.anchor_z_to_plane);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/mujoco_bridge/camera/color/image_raw", sensor_qos,
      std::bind(&ObjectPoseEstimatorNode::onRgb, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/mujoco_bridge/camera/depth/image_raw", sensor_qos,
      std::bind(&ObjectPoseEstimatorNode::onDepth, this, std::placeholders::_1));
    color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/mujoco_bridge/camera/color/camera_info", sensor_qos,
      std::bind(&ObjectPoseEstimatorNode::onColorInfo, this, std::placeholders::_1));
    depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/mujoco_bridge/camera/depth/camera_info", sensor_qos,
      std::bind(&ObjectPoseEstimatorNode::onDepthInfo, this, std::placeholders::_1));
    observation_sub_ = create_subscription<manipulation_interfaces::msg::BridgeObservation>(
      "/mujoco_bridge/episode_observation", rclcpp::QoS(10),
      std::bind(&ObjectPoseEstimatorNode::onObservation, this, std::placeholders::_1));
    pose_pub_ = create_publisher<manipulation_interfaces::msg::VisionObjectPose>(
      "~/object_pose", rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(),
      "RGB-D geometry estimator ready: exact timestamp matching, plane z=%.3fm, "
      "box %.3fx%.3fx%.3fm",
      config_.plane_z_m, 2.0 * box_model_.half_extents.x(),
      2.0 * box_model_.half_extents.y(), 2.0 * box_model_.half_extents.z());
  }

private:
  void onRgb(const ImageConstPtr message)
  {
    rgb_frames_[stampKey(message->header.stamp)] = message;
    trim(rgb_frames_);
    tryProcess(stampKey(message->header.stamp));
  }

  void onDepth(const ImageConstPtr message)
  {
    depth_frames_[stampKey(message->header.stamp)] = message;
    trim(depth_frames_);
    tryProcess(stampKey(message->header.stamp));
  }

  void onColorInfo(const CameraInfoConstPtr message)
  {
    color_info_[stampKey(message->header.stamp)] = message;
    trim(color_info_);
    tryProcess(stampKey(message->header.stamp));
  }

  void onDepthInfo(const CameraInfoConstPtr message)
  {
    depth_info_[stampKey(message->header.stamp)] = message;
    trim(depth_info_);
    tryProcess(stampKey(message->header.stamp));
  }

  void onObservation(const ObservationConstPtr message)
  {
    const Stamp key = stampKey(message->joint_state.header.stamp);
    observations_[key] = message;
    trim(observations_);
    tryProcess(key);
  }

  template<typename T>
  void trim(std::map<Stamp, T> & cache)
  {
    while (cache.size() > 30) {
      cache.erase(cache.begin());
    }
  }

  void tryProcess(Stamp key)
  {
    if (processed_.count(key) != 0 || rgb_frames_.count(key) == 0 ||
      depth_frames_.count(key) == 0 || color_info_.count(key) == 0 ||
      depth_info_.count(key) == 0 || observations_.count(key) == 0)
    {
      return;
    }
    processed_.insert(key);
    while (processed_.size() > 60) {
      processed_.erase(processed_.begin());
    }

    const auto & rgb = rgb_frames_.at(key);
    const auto & depth = depth_frames_.at(key);
    const auto & color_info = color_info_.at(key);
    const auto & depth_info = depth_info_.at(key);
    const auto & observation = observations_.at(key);
    const std::size_t rgb_row_bytes = static_cast<std::size_t>(rgb->width) * 3U;
    const std::size_t depth_row_bytes =
      static_cast<std::size_t>(depth->width) * sizeof(float);
    const bool matching_dimensions =
      rgb->width == depth->width && rgb->height == depth->height &&
      color_info->width == rgb->width && color_info->height == rgb->height &&
      depth_info->width == depth->width && depth_info->height == depth->height;
    const bool valid_frames =
      rgb->encoding == "rgb8" && !rgb->is_bigendian && rgb->width != 0 && rgb->height != 0 &&
      rgb->step >= rgb_row_bytes && rgb->data.size() >= rgb->step * rgb->height &&
      depth->encoding == "32FC1" && !depth->is_bigendian && depth->width != 0 &&
      depth->height != 0 && depth->step >= depth_row_bytes &&
      depth->data.size() >= depth->step * depth->height;
    const bool valid_camera_info =
      color_info->k[0] > 0.0 && color_info->k[4] > 0.0 &&
      depth_info->k[0] > 0.0 && depth_info->k[4] > 0.0;
    const bool matching_frames =
      !rgb->header.frame_id.empty() && rgb->header.frame_id == color_info->header.frame_id &&
      !depth->header.frame_id.empty() && depth->header.frame_id == depth_info->header.frame_id;
    if (!valid_frames || !matching_dimensions || !valid_camera_info || !matching_frames) {
      publishRejected(depth->header, *observation, RejectionReason::kInvalidInput);
      return;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        "world", depth->header.frame_id, rclcpp::Time(depth->header.stamp));
      const Eigen::Isometry3d world_from_optical = eigenTransform(transform);
      std::vector<float> values(
        static_cast<std::size_t>(depth->width) * static_cast<std::size_t>(depth->height));
      for (uint32_t row = 0; row < depth->height; ++row) {
        const auto * source = depth->data.data() + static_cast<std::size_t>(row) * depth->step;
        std::memcpy(
          values.data() + static_cast<std::size_t>(row) * depth->width,
          source, static_cast<std::size_t>(depth->width) * sizeof(float));
      }

      const SegmentationResult segmentation = segmentDepth(
        values, *depth_info, world_from_optical, config_);
      if (segmentation.rejection != RejectionReason::kNone) {
        publishRejected(
          depth->header, *observation, segmentation.rejection,
          segmentation.target_cluster->size());
        return;
      }
      const PoseEstimate estimate = estimateBoxPose(
        *segmentation.target_cluster, box_model_, config_.plane_z_m);
      publishEstimate(depth->header, *observation, estimate);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot transform %s to world: %s",
        depth->header.frame_id.c_str(), error.what());
      publishRejected(depth->header, *observation, RejectionReason::kInvalidInput);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "RGB-D processing failed: %s", error.what());
      publishRejected(depth->header, *observation, RejectionReason::kInvalidInput);
    }
  }

  void fillCommon(
    manipulation_interfaces::msg::VisionObjectPose & output,
    const std_msgs::msg::Header & header,
    const manipulation_interfaces::msg::BridgeObservation & observation) const
  {
    output.header = header;
    output.header.frame_id = "world";
    output.bridge_session = observation.bridge_session;
    output.generation = observation.generation;
    output.sample_sequence = observation.sample_sequence;
    output.pose.orientation.w = 1.0;
  }

  void publishRejected(
    const std_msgs::msg::Header & header,
    const manipulation_interfaces::msg::BridgeObservation & observation,
    RejectionReason reason,
    std::size_t point_count = 0)
  {
    manipulation_interfaces::msg::VisionObjectPose output;
    fillCommon(output, header, observation);
    output.point_count = static_cast<uint32_t>(point_count);
    output.rejection_reason = rejectionReasonName(reason);
    pose_pub_->publish(output);
  }

  void publishEstimate(
    const std_msgs::msg::Header & header,
    const manipulation_interfaces::msg::BridgeObservation & observation,
    const PoseEstimate & estimate)
  {
    manipulation_interfaces::msg::VisionObjectPose output;
    fillCommon(output, header, observation);
    output.pose.position.x = estimate.position.x();
    output.pose.position.y = estimate.position.y();
    output.pose.position.z = estimate.position.z();
    output.pose.orientation.w = estimate.orientation.w();
    output.pose.orientation.x = estimate.orientation.x();
    output.pose.orientation.y = estimate.orientation.y();
    output.pose.orientation.z = estimate.orientation.z();
    output.accepted = estimate.accepted;
    output.confidence = estimate.confidence;
    output.residual_m = estimate.residual_m;
    output.inlier_ratio = estimate.inlier_ratio;
    output.point_count = static_cast<uint32_t>(estimate.point_count);
    output.rejection_reason = rejectionReasonName(estimate.rejection);
    pose_pub_->publish(output);
  }

  SegmentationConfig config_;
  BoxModel box_model_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<manipulation_interfaces::msg::BridgeObservation>::SharedPtr
    observation_sub_;
  rclcpp::Publisher<manipulation_interfaces::msg::VisionObjectPose>::SharedPtr pose_pub_;
  std::map<Stamp, ImageConstPtr> rgb_frames_;
  std::map<Stamp, ImageConstPtr> depth_frames_;
  std::map<Stamp, CameraInfoConstPtr> color_info_;
  std::map<Stamp, CameraInfoConstPtr> depth_info_;
  std::map<Stamp, ObservationConstPtr> observations_;
  std::set<Stamp> processed_;
};

}  // namespace mujoco_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mujoco_perception::ObjectPoseEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
