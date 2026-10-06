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
#include "manipulation_interfaces/msg/block_candidate.hpp"
#include "manipulation_interfaces/msg/initial_bin_pose.hpp"
#include "manipulation_interfaces/msg/initial_box_pose.hpp"
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
#include "mujoco_perception/initial_pose_estimator.hpp"
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
    box_model_.min_visible_extent_m = declare_parameter(
      "box_min_visible_extent_m", box_model_.min_visible_extent_m);
    box_model_.confidence_reference_points = declare_parameter(
      "box_confidence_reference_points", box_model_.confidence_reference_points);
    box_model_.inlier_tolerance_m = declare_parameter(
      "box_inlier_tolerance_m", box_model_.inlier_tolerance_m);
    box_model_.max_residual_m = declare_parameter(
      "max_residual_m", box_model_.max_residual_m);
    box_model_.min_inlier_ratio = declare_parameter(
      "min_inlier_ratio", box_model_.min_inlier_ratio);
    box_model_.anchor_z_to_plane = declare_parameter(
      "anchor_z_to_plane", box_model_.anchor_z_to_plane);
    tracker_config_.min_confidence = declare_parameter(
      "tracking.min_confidence", tracker_config_.min_confidence);
    tracker_ = std::make_unique<ObjectTracker>(tracker_config_);

    // Initial box and bin detectors (Week 4.1 Stages 5, 6): the table height, the box and the
    // depth range are the ones the older pipeline already uses, so there is one source of truth
    // for them. The bin's own numbers are the scene's (InitialBinConfig).
    InitialBoxConfig initial_box_config;
    initial_box_config.plane_z_m = config_.plane_z_m;
    initial_box_config.half_extents = box_model_.half_extents;
    initial_box_config.depth_min_m = config_.depth_min_m;
    initial_box_config.depth_max_m = config_.depth_max_m;
    InitialBinConfig initial_bin_config;
    initial_bin_config.plane_z_m = config_.plane_z_m;
    initial_bin_config.depth_min_m = config_.depth_min_m;
    initial_bin_config.depth_max_m = config_.depth_max_m;
    const int initial_box_frames = declare_parameter("initial_box.frames", 10);
    if (initial_box_frames < 1) {
      throw std::invalid_argument("initial_box.frames must be at least 1");
    }
    initial_pose_estimator_ = std::make_unique<InitialPoseEstimator>(
      static_cast<std::size_t>(initial_box_frames), initial_box_config, initial_bin_config);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/mujoco_bridge/camera/depth/image_raw", sensor_qos,
      std::bind(&ObjectPoseEstimatorNode::onDepth, this, std::placeholders::_1));
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
    initial_box_pub_ = create_publisher<manipulation_interfaces::msg::InitialBoxPose>(
      "~/initial_box_pose", rclcpp::QoS(10));
    initial_bin_pub_ = create_publisher<manipulation_interfaces::msg::InitialBinPose>(
      "~/initial_bin_pose", rclcpp::QoS(10));
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
      "Depth geometry estimator ready: exact timestamp matching, plane z=%.3fm, "
      "box %.3fx%.3fx%.3fm, initial box and bin from the mean of %d frames",
      config_.plane_z_m, 2.0 * box_model_.half_extents.x(),
      2.0 * box_model_.half_extents.y(), 2.0 * box_model_.half_extents.z(), initial_box_frames);
    RCLCPP_INFO(get_logger(), "Robot mask loaded %zu visual meshes", robot_meshes_.size());
  }

private:
  void onDepth(const ImageConstPtr message)
  {
    depth_frames_[stampKey(message->header.stamp)] = message;
    trim(depth_frames_);
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
      depth_frames_.clear();
      depth_info_.clear();
      tf_frames_.clear();
    }
    if (lifecycle_known_ &&
      (message->bridge_session != bridge_session_ || message->generation != generation_))
    {
      observations_.clear();
      pending_since_.clear();
      processed_.clear();
      initial_pose_estimator_->reset();  // another episode: the box is somewhere else
    }
    lifecycle_known_ = true;
    const auto previous_attachment = attachment_state_;
    attachment_state_ = static_cast<AttachmentState>(message->attachment_state);
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
    if (previous_attachment == AttachmentState::kAttached &&
      attachment_state_ != AttachmentState::kAttached)
    {
      // Release starts a new visual observation episode. Do not process RGB-D
      // frames captured while the estimator was intentionally resting.
      tracker_->reset(bridge_session_, generation_);
      depth_frames_.clear();
      depth_info_.clear();
      pending_since_.clear();
      processed_.clear();
      // The box has been carried: whatever the window holds is a picture of the old scene.
      initial_pose_estimator_->reset();
    }
    const Stamp key = stampKey(message->joint_state.header.stamp);
    observations_[key] = message;
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
          publishInitialPosesUnusable(
            depth->second->header, *observation->second,
            RejectionReason::kMissingRobotTransform);
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
    if (attachment_state_ == AttachmentState::kAttached) {
      return;
    }
    if (processed_.count(key) != 0 || depth_frames_.count(key) == 0 ||
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

    const auto & depth = depth_frames_.at(key);
    const auto & depth_info = depth_info_.at(key);
    const auto & observation = observations_.at(key);
    const auto processing_start = std::chrono::steady_clock::now();
    const std::size_t depth_row_bytes =
      static_cast<std::size_t>(depth->width) * sizeof(float);
    const bool matching_dimensions =
      depth_info->width == depth->width && depth_info->height == depth->height;
    const bool valid_frames =
      depth->encoding == "32FC1" && !depth->is_bigendian && depth->width != 0 &&
      depth->height != 0 && depth->step >= depth_row_bytes &&
      depth->data.size() >= depth->step * depth->height;
    const bool valid_camera_info = depth_info->k[0] > 0.0 && depth_info->k[4] > 0.0;
    const bool matching_frames =
      !depth->header.frame_id.empty() && depth->header.frame_id == depth_info->header.frame_id;
    if (!valid_frames || !matching_dimensions || !valid_camera_info || !matching_frames) {
      publishRejected(depth->header, *observation, RejectionReason::kInvalidInput);
      publishInitialPosesUnusable(depth->header, *observation, RejectionReason::kInvalidInput);
      return;
    }

    bool initial_box_published = false;
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
        publishInitialPosesUnusable(depth->header, *observation, RejectionReason::kInvalidInput);
        publishMaskDiagnostics(
          depth->header, *observation, mask, {}, elapsed_ms,
          rejectionReasonName(RejectionReason::kInvalidInput));
        return;
      }
      // The new detector looks at the same masked depth, before the older pipeline touches it.
      updateInitialPoses(
        depth->header, *observation, mask.filtered_depth, *depth_info, world_from_optical);
      initial_box_published = true;
      SegmentationResult segmentation = segmentDepth(
        mask.filtered_depth, *depth_info, world_from_optical, config_);
      publishCloud(depth->header, *segmentation.foreground_points, foreground_pub_);
      TrackingSample sample = trackingSample(depth->header, *observation);
      BoxModel model = box_model_;
      model.anchor_z_to_plane = model.anchor_z_to_plane && tracker_->anchorToSupport(sample);
      sample.support_prior_used = model.anchor_z_to_plane;
      std::vector<PoseEstimate> candidates;
      for (const auto & cluster : segmentation.candidate_clusters) {
        candidates.push_back(estimateBoxPose(*cluster, model, config_.plane_z_m));
        const auto & candidate = candidates.back();
        if (!candidate.geometry_valid || candidate.confidence < tracker_config_.min_confidence) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Candidate %zu: geometry=%s reason=%s points=%zu confidence=%.3f "
            "residual=%.4fm inlier=%.3f",
            candidates.size() - 1, candidate.geometry_valid ? "valid" : "invalid",
            rejectionReasonName(candidate.rejection), candidate.point_count,
            candidate.confidence, candidate.residual_m, candidate.inlier_ratio);
        }
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
      publishInitialPosesUnusable(
        depth->header, *observation, RejectionReason::kMissingRobotTransform);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Depth processing failed: %s", error.what());
      publishRejected(depth->header, *observation, RejectionReason::kInvalidInput);
      if (!initial_box_published) {
        publishInitialPosesUnusable(depth->header, *observation, RejectionReason::kInvalidInput);
      }
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

  // Adds a usable frame to the initial-pose window and publishes where the box and the bin
  // stand, one message each.
  void updateInitialPoses(
    const std_msgs::msg::Header & header,
    const manipulation_interfaces::msg::BridgeObservation & observation,
    const std::vector<float> & masked_depth, const sensor_msgs::msg::CameraInfo & camera_info,
    const Eigen::Isometry3d & world_from_optical)
  {
    const auto start = std::chrono::steady_clock::now();
    const InitialPoseEstimate estimate =
      initial_pose_estimator_->update(masked_depth, camera_info, world_from_optical);
    manipulation_interfaces::msg::InitialBoxPose box;
    fillInitialPose(
      box, header, observation, estimate, estimate.boxState(), estimate.box,
      initialBoxRejectionName(estimate.box.rejection));
    manipulation_interfaces::msg::InitialBinPose bin;
    fillInitialPose(
      bin, header, observation, estimate, estimate.binState(), estimate.bin,
      initialBinRejectionName(estimate.bin.rejection));
    // The time for averaging and both detections, the same on both messages.
    box.processing_ms = bin.processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    initial_box_pub_->publish(box);
    initial_bin_pub_->publish(bin);
  }

  // The fields InitialBoxPose and InitialBinPose share.
  template<typename Message, typename Detection>
  void fillInitialPose(
    Message & output, const std_msgs::msg::Header & header,
    const manipulation_interfaces::msg::BridgeObservation & observation,
    const InitialPoseEstimate & estimate, InitialPoseState state, const Detection & detection,
    const char * rejection_name) const
  {
    fillInitialPoseCommon(output, header, observation);
    output.state = static_cast<uint8_t>(state);
    output.frames_averaged = static_cast<uint32_t>(estimate.frames_averaged);
    output.frames_required = static_cast<uint32_t>(estimate.frames_required);
    switch (state) {
      case InitialPoseState::kWarmingUp:
        output.reason = "WINDOW_FILLING";
        break;
      case InitialPoseState::kNotMeasured:
        output.reason = rejection_name;
        break;
      case InitialPoseState::kMeasured:
        output.position.x = detection.position.x();
        output.position.y = detection.position.y();
        output.position.z = detection.position.z();
        output.yaw_rad = detection.yaw_rad;
        break;
    }
    output.valid_depth_pixels = static_cast<uint32_t>(detection.valid_depth_pixels);
    for (const auto & candidate : detection.candidates) {
      manipulation_interfaces::msg::BlockCandidate item;
      item.pixels = static_cast<uint32_t>(candidate.pixels);
      item.side_along_m = candidate.side_along_m;
      item.side_across_m = candidate.side_across_m;
      item.center_x = candidate.center_xy.x();
      item.center_y = candidate.center_xy.y();
      item.matches = candidate.matches;
      output.candidates.push_back(item);
    }
  }

  // A frame that could not be used (bad input, no robot TF): say so, for the box and for the
  // bin. It is not added to the window, so the window holds the last usable frames, and a
  // consumer sees a gap, not a pose.
  void publishInitialPosesUnusable(
    const std_msgs::msg::Header & header,
    const manipulation_interfaces::msg::BridgeObservation & observation,
    RejectionReason reason)
  {
    const auto frames_averaged = static_cast<uint32_t>(initial_pose_estimator_->framesInWindow());
    const auto frames_required = static_cast<uint32_t>(initial_pose_estimator_->frames());
    manipulation_interfaces::msg::InitialBoxPose box;
    fillInitialPoseCommon(box, header, observation);
    box.state = static_cast<uint8_t>(InitialPoseState::kNotMeasured);
    box.reason = rejectionReasonName(reason);
    box.frames_averaged = frames_averaged;
    box.frames_required = frames_required;
    initial_box_pub_->publish(box);
    manipulation_interfaces::msg::InitialBinPose bin;
    fillInitialPoseCommon(bin, header, observation);
    bin.state = static_cast<uint8_t>(InitialPoseState::kNotMeasured);
    bin.reason = rejectionReasonName(reason);
    bin.frames_averaged = frames_averaged;
    bin.frames_required = frames_required;
    initial_bin_pub_->publish(bin);
  }

  template<typename Message>
  void fillInitialPoseCommon(
    Message & output, const std_msgs::msg::Header & header,
    const manipulation_interfaces::msg::BridgeObservation & observation) const
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    output.header = header;
    output.header.frame_id = "world";
    output.bridge_session = observation.bridge_session;
    output.generation = observation.generation;
    output.sample_sequence = observation.sample_sequence;
    output.position.x = nan;
    output.position.y = nan;
    output.position.z = nan;
    output.yaw_rad = nan;
  }

  void publishRejected(
    const std_msgs::msg::Header & header,
    const manipulation_interfaces::msg::BridgeObservation & observation,
    RejectionReason reason,
    std::size_t point_count = 0)
  {
    TrackingSample sample = trackingSample(header, observation);
    sample.input_failure = rejectionReasonName(reason);
    publishTracking(header, observation, tracker_->update(sample, {}), 0.0);
    (void)point_count;
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
    sample.attachment_state = static_cast<AttachmentState>(observation.attachment_state);
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
    output.grasp_state = static_cast<uint8_t>(tracking.grasp_state);
    output.attachment_valid = tracking.attachment_valid;
    output.last_measurement_sequence = tracking.measurement_sequence;
    output.processing_ms = processing_ms;
    if (tracking.state == EvidenceState::kMeasured ||
      tracking.state == EvidenceState::kPredicted)
    {
      output.pose.position.x = estimate.position.x();
      output.pose.position.y = estimate.position.y();
      output.pose.position.z = estimate.position.z();
      output.pose.orientation.w = estimate.orientation.w();
      output.pose.orientation.x = estimate.orientation.x();
      output.pose.orientation.y = estimate.orientation.y();
      output.pose.orientation.z = estimate.orientation.z();
    }
    output.confidence = estimate.confidence;
    output.orientation_ambiguous = estimate.orientation_ambiguous;
    output.residual_m = estimate.residual_m;
    output.inlier_ratio = estimate.inlier_ratio;
    output.point_count = static_cast<uint32_t>(estimate.point_count);
    output.state_reason = tracking.reason;
    output.diagnostic_stage = static_cast<uint8_t>(tracking.diagnostic_stage);
    output.candidate_count = static_cast<uint32_t>(tracking.candidate_count);
    output.eligible_candidate_count = static_cast<uint32_t>(tracking.eligible_candidate_count);
    output.support_prior_used = tracking.support_prior_used;
    pose_pub_->publish(output);
  }

  SegmentationConfig config_;
  BoxModel box_model_;
  TrackerConfig tracker_config_;
  std::unique_ptr<ObjectTracker> tracker_;
  std::unique_ptr<InitialPoseEstimator> initial_pose_estimator_;
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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<manipulation_interfaces::msg::BridgeObservation>::SharedPtr
    observation_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Publisher<manipulation_interfaces::msg::VisionObjectPose>::SharedPtr pose_pub_;
  rclcpp::Publisher<manipulation_interfaces::msg::InitialBoxPose>::SharedPtr initial_box_pub_;
  rclcpp::Publisher<manipulation_interfaces::msg::InitialBinPose>::SharedPtr initial_bin_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr predicted_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr foreground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
  rclcpp::Publisher<manipulation_interfaces::msg::RobotMaskDiagnostics>::SharedPtr
    mask_diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr pending_timer_;
  std::map<Stamp, ImageConstPtr> depth_frames_;
  std::map<Stamp, CameraInfoConstPtr> depth_info_;
  std::map<Stamp, ObservationConstPtr> observations_;
  std::map<Stamp, std::set<std::string>> tf_frames_;
  std::map<Stamp, std::chrono::steady_clock::time_point> pending_since_;
  std::set<Stamp> processed_;
  bool lifecycle_known_ = false;
  uint64_t bridge_session_ = 0;
  uint64_t generation_ = 0;
  AttachmentState attachment_state_ = AttachmentState::kNotAttached;
};

}  // namespace mujoco_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mujoco_perception::ObjectPoseEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
