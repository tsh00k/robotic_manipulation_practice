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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "manipulation_interfaces/msg/bridge_observation.hpp"
#include "manipulation_interfaces/msg/robot_mask_diagnostics.hpp"
#include "manipulation_interfaces/msg/vision_object_pose.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "mujoco_perception/geometry_pipeline.hpp"
#include "mujoco_perception/robot_mask.hpp"
#include "mujoco_perception/object_tracker.hpp"

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
    mask_config_.depth_tolerance_m = declare_parameter(
      "robot_mask.depth_tolerance_m", mask_config_.depth_tolerance_m);
    const std::string assets = ament_index_cpp::get_package_share_directory("robot_description") +
      "/mujoco/franka_emika_panda/assets";
    robot_meshes_ = loadRobotVisualMeshes(assets);
    robot_mask_filter_ = std::make_unique<RobotMaskFilter>(robot_meshes_);
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
    tracker_config_.supported_z_m = config_.plane_z_m + box_model_.half_extents.z();
    tracker_config_.association_slack_m = declare_parameter(
      "tracking.association_slack_m", tracker_config_.association_slack_m);
    tracker_config_.max_speed_m_s = declare_parameter(
      "tracking.max_speed_m_s", tracker_config_.max_speed_m_s);
    tracker_config_.min_confidence = declare_parameter(
      "tracking.min_confidence", tracker_config_.min_confidence);
    tracker_config_.max_prediction_age_s = declare_parameter(
      "tracking.max_prediction_age_s", tracker_config_.max_prediction_age_s);
    tracker_config_.released_prediction_age_s = declare_parameter(
      "tracking.released_prediction_age_s", tracker_config_.released_prediction_age_s);
    tracker_config_.uncertainty_growth_m_s = declare_parameter(
      "tracking.uncertainty_growth_m_s", tracker_config_.uncertainty_growth_m_s);
    tracker_config_.max_uncertainty_m = declare_parameter(
      "tracking.max_uncertainty_m", tracker_config_.max_uncertainty_m);
    tracker_config_.object_width_m = 2.0 * box_model_.half_extents.y();
    tracker_config_.width_tolerance_m = declare_parameter(
      "grasp.width_tolerance_m", tracker_config_.width_tolerance_m);
    tracker_config_.width_stability_m = declare_parameter(
      "grasp.width_stability_m", tracker_config_.width_stability_m);
    tracker_config_.grasp_stable_s = declare_parameter(
      "grasp.stable_s", tracker_config_.grasp_stable_s);
    tracker_config_.grasp_measurement_max_age_s = declare_parameter(
      "grasp.measurement_max_age_s", tracker_config_.grasp_measurement_max_age_s);
    tracker_config_.grasp_tcp_motion_m = declare_parameter(
      "grasp.tcp_motion_m", tracker_config_.grasp_tcp_motion_m);
    tracker_config_.grasp_tcp_angle_rad = declare_parameter(
      "grasp.tcp_angle_rad", tracker_config_.grasp_tcp_angle_rad);
    tracker_config_.closed_width_m = declare_parameter(
      "grasp.open_width_threshold_m", tracker_config_.closed_width_m);
    tracker_config_.tcp_distance_m = declare_parameter(
      "grasp.tcp_distance_m", tracker_config_.tcp_distance_m);
    tracker_config_.initial_uncertainty_m = declare_parameter(
      "tracking.initial_uncertainty_m", tracker_config_.initial_uncertainty_m);
    tracker_config_.support_tolerance_m = declare_parameter(
      "tracking.support_tolerance_m", tracker_config_.support_tolerance_m);
    tracker_config_.held_prediction_age_s = declare_parameter(
      "tracking.held_prediction_age_s", tracker_config_.held_prediction_age_s);
    tracker_config_.held_uncertainty_growth_m_s = declare_parameter(
      "tracking.held_uncertainty_growth_m_s", tracker_config_.held_uncertainty_growth_m_s);
    tracker_config_.slip_tolerance_m = declare_parameter(
      "tracking.slip_tolerance_m", tracker_config_.slip_tolerance_m);
    tracker_config_.max_sample_gap_s = declare_parameter(
      "tracking.max_sample_gap_s", tracker_config_.max_sample_gap_s);
    tracker_ = std::make_unique<ObjectTracker>(tracker_config_);

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
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100),
      std::bind(&ObjectPoseEstimatorNode::onTf, this, std::placeholders::_1));
    pose_pub_ = create_publisher<manipulation_interfaces::msg::VisionObjectPose>(
      "~/object_pose", rclcpp::QoS(10));
    predicted_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "~/debug/robot_predicted_depth", sensor_qos);
    mask_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "~/debug/robot_mask", sensor_qos);
    filtered_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "~/debug/filtered_depth", sensor_qos);
    foreground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/debug/foreground_points", sensor_qos);
    cluster_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/debug/target_cluster", sensor_qos);
    mask_diagnostics_pub_ = create_publisher<manipulation_interfaces::msg::RobotMaskDiagnostics>(
      "~/debug/robot_mask_diagnostics", rclcpp::QoS(10));
    pending_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&ObjectPoseEstimatorNode::retryPending, this));

    RCLCPP_INFO(
      get_logger(),
      "RGB-D geometry estimator ready: exact timestamp matching, plane z=%.3fm, "
      "box %.3fx%.3fx%.3fm",
      config_.plane_z_m, 2.0 * box_model_.half_extents.x(),
      2.0 * box_model_.half_extents.y(), 2.0 * box_model_.half_extents.z());
    RCLCPP_INFO(get_logger(), "Robot mask loaded %zu visual meshes", robot_meshes_.size());
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
    if (retired_sessions_.count(message->bridge_session) != 0) {
      return;
    }
    if (lifecycle_known_ && message->bridge_session == bridge_session_ &&
      message->generation < generation_)
    {
      return;
    }
    if (lifecycle_known_ && message->bridge_session != bridge_session_) {
      retired_sessions_.insert(bridge_session_);
      rgb_frames_.clear();
      depth_frames_.clear();
      color_info_.clear();
      depth_info_.clear();
      tf_frames_.clear();
    }
    if (lifecycle_known_ &&
      (message->bridge_session != bridge_session_ || message->generation != generation_))
    {
      observations_.clear();
      pending_since_.clear();
      processed_.clear();
      robot_samples_.clear();
    }
    lifecycle_known_ = true;
    bridge_session_ = message->bridge_session;
    generation_ = message->generation;
    if (!tracker_lifecycle_known_ || tracker_session_ != bridge_session_ ||
      tracker_generation_ != generation_)
    {
      tracker_->reset(bridge_session_, generation_);
      tracker_lifecycle_known_ = true;
      tracker_session_ = bridge_session_;
      tracker_generation_ = generation_;
    }
    const Stamp key = stampKey(message->joint_state.header.stamp);
    observations_[key] = message;
    robot_samples_[key] = message;
    trim(robot_samples_, 300);
    trim(observations_, 300);
    tryProcess(key);
  }

  void onTf(const tf2_msgs::msg::TFMessage::ConstSharedPtr message)
  {
    std::map<Stamp, std::set<std::string>> received;
    for (const auto & transform : message->transforms) {
      const Stamp key = stampKey(transform.header.stamp);
      received[key].insert(transform.child_frame_id);
    }
    for (auto & frame_set : received) {
      const Stamp key = frame_set.first;
      tf_frames_[key] = std::move(frame_set.second);
      trim(tf_frames_);
      tryProcess(key);
    }
  }

  bool hasRobotTf(Stamp key) const
  {
    const auto found = tf_frames_.find(key);
    if (found == tf_frames_.end()) {
      return false;
    }
    for (const auto & frame : dynamic_robot_frames_) {
      if (found->second.count(frame) == 0) {
        return false;
      }
    }
    return true;
  }

  void retryPending()
  {
    for (auto pending = pending_since_.begin(); pending != pending_since_.end(); ) {
      const Stamp key = pending->first;
      const bool expired = std::chrono::steady_clock::now() - pending->second >
        std::chrono::milliseconds(500);
      if (hasRobotTf(key)) {
        ++pending;
        tryProcess(key);
      } else if (expired) {
        const auto depth = depth_frames_.find(key);
        const auto observation = observations_.find(key);
        if (depth != depth_frames_.end() && observation != observations_.end()) {
          publishRejected(
            depth->second->header, *observation->second,
            RejectionReason::kMissingRobotTransform);
          publishMaskDiagnostics(
            depth->second->header, *observation->second, {}, {}, 0.0,
            rejectionReasonName(RejectionReason::kMissingRobotTransform));
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Pending sample lost inputs before robot TF timeout: depth=%d observation=%d",
            depth != depth_frames_.end(), observation != observations_.end());
        }
        processed_.insert(key);
        pending = pending_since_.erase(pending);
      } else {
        ++pending;
      }
    }
  }

  template<typename T>
  void trim(std::map<Stamp, T> & cache, std::size_t max_size = 30)
  {
    while (cache.size() > max_size) {
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
    if (!hasRobotTf(key)) {
      pending_since_.emplace(key, std::chrono::steady_clock::now());
      return;
    }
    pending_since_.erase(key);
    processed_.insert(key);
    while (processed_.size() > 60) {
      processed_.erase(processed_.begin());
    }

    const auto & rgb = rgb_frames_.at(key);
    const auto & depth = depth_frames_.at(key);
    const auto & color_info = color_info_.at(key);
    const auto & depth_info = depth_info_.at(key);
    const auto & observation = observations_.at(key);
    const auto processing_start = std::chrono::steady_clock::now();
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
        "world", depth->header.frame_id, rclcpp::Time(depth->header.stamp),
        rclcpp::Duration::from_seconds(0.1));
      const Eigen::Isometry3d world_from_optical = eigenTransform(transform);
      std::map<std::string, Eigen::Isometry3d> world_from_links;
      for (const auto & frame : robot_frames_) {
        world_from_links.emplace(
          frame, eigenTransform(
            tf_buffer_.lookupTransform(
              "world", frame, rclcpp::Time(depth->header.stamp),
              rclcpp::Duration::from_seconds(0.1))));
      }
      std::vector<float> values(
        static_cast<std::size_t>(depth->width) * static_cast<std::size_t>(depth->height));
      for (uint32_t row = 0; row < depth->height; ++row) {
        const auto * source = depth->data.data() + static_cast<std::size_t>(row) * depth->step;
        std::memcpy(
          values.data() + static_cast<std::size_t>(row) * depth->width,
          source, static_cast<std::size_t>(depth->width) * sizeof(float));
      }

      const auto mask_start = std::chrono::steady_clock::now();
      const RobotMaskResult mask = robot_mask_filter_->filter(
        values, *depth_info, world_from_optical.inverse(), world_from_links, mask_config_);
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - mask_start).count();
      publishMaskImages(depth->header, depth->width, depth->height, mask);
      if (!mask.valid) {
        publishRejected(depth->header, *observation, RejectionReason::kInvalidInput);
        publishMaskDiagnostics(
          depth->header, *observation, mask, {}, elapsed_ms,
          rejectionReasonName(RejectionReason::kInvalidInput));
        return;
      }
      SegmentationResult segmentation = segmentDepth(
        mask.filtered_depth, *depth_info, world_from_optical, config_);
      publishCloud(depth->header, *segmentation.foreground_points, foreground_pub_);
      const TrackingSample sample = trackingSample(depth->header, *observation);
      consumeRobotSamples(key);
      BoxModel model = box_model_;
      model.anchor_z_to_plane = model.anchor_z_to_plane && tracker_->anchorToSupport(sample);
      std::vector<PoseEstimate> candidates;
      for (const auto & cluster : segmentation.candidate_clusters) {
        // An elevated candidate must never inherit the support-height prior.
        BoxModel candidate_model = model;
        double max_z = -std::numeric_limits<double>::infinity();
        for (const auto & point : *cluster) {
          max_z = std::max(max_z, static_cast<double>(point.z));
        }
        if (max_z > config_.plane_z_m + 2.0 * box_model_.half_extents.z() +
          tracker_config_.support_tolerance_m)
        {
          candidate_model.anchor_z_to_plane = false;
        }
        candidates.push_back(estimateBoxPose(*cluster, candidate_model, config_.plane_z_m));
      }
      TrackingSample checked_sample = sample;
      if (segmentation.rejection == RejectionReason::kInvalidInput) {
        checked_sample.input_failure = rejectionReasonName(segmentation.rejection);
      }
      const TrackingResult tracking = tracker_->update(checked_sample, candidates);
      segmentation.target_cluster->clear();
      if (tracking.candidate_index >= 0) {
        segmentation.target_cluster = segmentation.candidate_clusters.at(
          static_cast<std::size_t>(tracking.candidate_index));
      }
      publishCloud(depth->header, *segmentation.target_cluster, cluster_pub_);
      publishMaskDiagnostics(
        depth->header, *observation, mask, segmentation, elapsed_ms,
        tracking.reason);
      publishTracking(
        depth->header, *observation, tracking,
        std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - processing_start).count());
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot transform %s to world: %s",
        depth->header.frame_id.c_str(), error.what());
      publishRejected(depth->header, *observation, RejectionReason::kMissingRobotTransform);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "RGB-D processing failed: %s", error.what());
      publishRejected(depth->header, *observation, RejectionReason::kInvalidInput);
    }
  }

  void publishMaskImages(
    const std_msgs::msg::Header & header, uint32_t width, uint32_t height,
    const RobotMaskResult & mask)
  {
    const auto publish_float = [&](const std::vector<float> & values,
        const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher) {
        sensor_msgs::msg::Image image;
        image.header = header;
        image.width = width;
        image.height = height;
        image.encoding = "32FC1";
        image.step = width * sizeof(float);
        image.data.resize(values.size() * sizeof(float));
        std::memcpy(image.data.data(), values.data(), image.data.size());
        publisher->publish(image);
      };
    publish_float(mask.predicted_depth, predicted_pub_);
    publish_float(mask.filtered_depth, filtered_pub_);
    sensor_msgs::msg::Image image;
    image.header = header;
    image.width = width;
    image.height = height;
    image.encoding = "mono8";
    image.step = width;
    image.data = mask.mask;
    mask_pub_->publish(image);
  }

  void publishCloud(
    const std_msgs::msg::Header & header,
    const pcl::PointCloud<pcl::PointXYZ> & points,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
  {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(points, message);
    message.header = header;
    message.header.frame_id = "world";
    publisher->publish(message);
  }

  void publishMaskDiagnostics(
    const std_msgs::msg::Header & header,
    const manipulation_interfaces::msg::BridgeObservation & observation,
    const RobotMaskResult & mask, const SegmentationResult & segmentation,
    double elapsed_ms, const std::string & status)
  {
    manipulation_interfaces::msg::RobotMaskDiagnostics output;
    output.header = header;
    output.bridge_session = observation.bridge_session;
    output.generation = observation.generation;
    output.sample_sequence = observation.sample_sequence;
    output.projected_pixels = static_cast<uint32_t>(mask.projected_pixels);
    output.masked_pixels = static_cast<uint32_t>(mask.masked_pixels);
    output.comparison_pixels = static_cast<uint32_t>(mask.comparison_pixels);
    output.mismatch_pixels = static_cast<uint32_t>(mask.mismatch_pixels);
    output.foreground_points = static_cast<uint32_t>(segmentation.foreground_points->size());
    output.target_cluster_points = static_cast<uint32_t>(segmentation.target_cluster->size());
    output.depth_tolerance_m = mask_config_.depth_tolerance_m;
    output.elapsed_ms = elapsed_ms;
    output.status = status;
    mask_diagnostics_pub_->publish(output);
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
    consumeRobotSamples(stampKey(header.stamp));
    TrackingSample sample = trackingSample(header, observation);
    sample.input_failure = rejectionReasonName(reason);
    publishTracking(header, observation, tracker_->update(sample, {}), 0.0);
    (void)point_count;
  }

  void consumeRobotSamples(Stamp key)
  {
    for (auto it = robot_samples_.begin(); it != robot_samples_.end() && it->first <= key; ) {
      tracker_->observeRobot(trackingSample(it->second->joint_state.header, *it->second));
      it = robot_samples_.erase(it);
    }
  }

  TrackingSample trackingSample(
    const std_msgs::msg::Header & header,
    const manipulation_interfaces::msg::BridgeObservation & observation) const
  {
    TrackingSample sample;
    sample.session = observation.bridge_session;
    sample.generation = observation.generation;
    sample.sequence = observation.sample_sequence;
    sample.time_s = static_cast<double>(stampKey(header.stamp)) * 1e-9;
    try {
      sample.world_from_tcp = eigenTransform(observation.world_to_hand_tcp);
    } catch (const std::exception &) {
      sample.input_failure = "INVALID_TCP_STATE";
    }
    sample.gripper_width_m = 0.0;
    int fingers = 0;
    for (std::size_t i = 0; i < observation.joint_state.name.size(); ++i) {
      const auto & name = observation.joint_state.name[i];
      if ((name == "finger_joint1" || name == "finger_joint2") &&
        i < observation.joint_state.position.size())
      {
        const double position = observation.joint_state.position[i];
        // MuJoCo soft limits can produce micrometre-scale undershoot at zero.
        if (!std::isfinite(position) || position < -1e-5 || position > 0.045) {
          sample.input_failure = "INVALID_GRIPPER_STATE";
        }
        sample.gripper_width_m += std::max(0.0, position);
        ++fingers;
      }
    }
    if (fingers != 2) {
      sample.input_failure = "MISSING_GRIPPER_STATE";
    }
    return sample;
  }

  void publishTracking(
    const std_msgs::msg::Header & header,
    const manipulation_interfaces::msg::BridgeObservation & observation,
    const TrackingResult & tracking, double processing_ms)
  {
    const auto & estimate = tracking.estimate;
    manipulation_interfaces::msg::VisionObjectPose output;
    fillCommon(output, header, observation);
    output.evidence_state = static_cast<uint8_t>(tracking.state);
    output.pose_valid = tracking.pose_valid;
    output.grasp_state = static_cast<uint8_t>(tracking.grasp_state);
    output.attachment_valid = tracking.attachment_valid;
    output.grasp_evidence_stamp = rclcpp::Time(
      static_cast<int64_t>(std::llround(tracking.grasp_evidence_time_s * 1e9)));
    output.attachment_stamp = rclcpp::Time(
      static_cast<int64_t>(std::llround(tracking.attachment_time_s * 1e9)));
    output.attachment_measurement_sequence = tracking.attachment_measurement_sequence;
    output.grasp_reason = tracking.grasp_reason;
    output.attachment_basis = tracking.attachment_basis;
    output.last_measurement_sequence = tracking.measurement_sequence;
    output.last_measurement_stamp = rclcpp::Time(
      static_cast<int64_t>(std::llround(tracking.measurement_time_s * 1e9)));
    output.prediction_age_s = tracking.prediction_age_s;
    output.position_uncertainty_m = tracking.uncertainty_m;
    output.processing_ms = processing_ms;
    if (tracking.pose_valid) {
      output.pose.position.x = estimate.position.x();
      output.pose.position.y = estimate.position.y();
      output.pose.position.z = estimate.position.z();
      output.pose.orientation.w = estimate.orientation.w();
      output.pose.orientation.x = estimate.orientation.x();
      output.pose.orientation.y = estimate.orientation.y();
      output.pose.orientation.z = estimate.orientation.z();
    }
    output.accepted = tracking.state == EvidenceState::kMeasured;
    output.confidence = estimate.confidence;
    output.orientation_ambiguous = estimate.orientation_ambiguous;
    output.residual_m = estimate.residual_m;
    output.inlier_ratio = estimate.inlier_ratio;
    output.point_count = static_cast<uint32_t>(estimate.point_count);
    output.rejection_reason = tracking.reason;
    pose_pub_->publish(output);
  }

  SegmentationConfig config_;
  BoxModel box_model_;
  TrackerConfig tracker_config_;
  std::unique_ptr<ObjectTracker> tracker_;
  bool tracker_lifecycle_known_ = false;
  uint64_t tracker_session_ = 0;
  uint64_t tracker_generation_ = 0;
  std::set<uint64_t> retired_sessions_;
  RobotMaskConfig mask_config_;
  std::vector<RobotMesh> robot_meshes_;
  std::unique_ptr<RobotMaskFilter> robot_mask_filter_;
  const std::vector<std::string> robot_frames_{
    "link0", "link1", "link2", "link3", "link4", "link5", "link6", "link7",
    "hand", "left_finger", "right_finger"};
  const std::vector<std::string> dynamic_robot_frames_{
    "link1", "link2", "link3", "link4", "link5", "link6", "link7",
    "left_finger", "right_finger"};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<manipulation_interfaces::msg::BridgeObservation>::SharedPtr
    observation_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Publisher<manipulation_interfaces::msg::VisionObjectPose>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr predicted_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr foreground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
  rclcpp::Publisher<manipulation_interfaces::msg::RobotMaskDiagnostics>::SharedPtr
    mask_diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr pending_timer_;
  std::map<Stamp, ImageConstPtr> rgb_frames_;
  std::map<Stamp, ImageConstPtr> depth_frames_;
  std::map<Stamp, CameraInfoConstPtr> color_info_;
  std::map<Stamp, CameraInfoConstPtr> depth_info_;
  std::map<Stamp, ObservationConstPtr> observations_;
  std::map<Stamp, ObservationConstPtr> robot_samples_;
  std::map<Stamp, std::set<std::string>> tf_frames_;
  std::map<Stamp, std::chrono::steady_clock::time_point> pending_since_;
  std::set<Stamp> processed_;
  bool lifecycle_known_ = false;
  uint64_t bridge_session_ = 0;
  uint64_t generation_ = 0;
};

}  // namespace mujoco_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mujoco_perception::ObjectPoseEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
