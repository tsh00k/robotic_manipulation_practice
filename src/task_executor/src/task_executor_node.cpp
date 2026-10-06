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
#include <array>
#include <cinttypes>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/msg/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <manipulation_interfaces/msg/bridge_observation.hpp>
#include <manipulation_interfaces/msg/episode_outcome.hpp>
#include <manipulation_interfaces/msg/initial_bin_pose.hpp>
#include <manipulation_interfaces/msg/initial_box_pose.hpp>
#include <manipulation_interfaces/msg/initial_pose_latch.hpp>
#include <manipulation_interfaces/srv/reset_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "task_executor/bin_containment.hpp"
#include "task_executor/cartesian_waypoint_source.hpp"
#include "task_executor/diff_ik_waypoint_source.hpp"
#include "task_executor/episode_controller.hpp"
#include "task_executor/keyframe_waypoint_source.hpp"
#include "task_executor/observation_snapshot.hpp"
#include "task_executor/phase.hpp"
#include "task_executor/pose_latch.hpp"
#include "task_executor/task_executor_config.hpp"
#include "task_executor/waypoint_source.hpp"

namespace task_executor
{

// ROS adapter for the episode controller.
//
// use_sim_time is auto-declared by rclcpp::Node and set by demo.launch.py.
// The adapter supplies ROS simulation time for phase durations and steady time
// for reset/observation watchdogs.
class TaskExecutorNode : public rclcpp::Node
{
public:
  // Choose the waypoint source, then connect the controller to the ROS graph.
  TaskExecutorNode()
  : Node("task_executor"), config_(loadTaskExecutorConfig(*this))
  {
    if (config_.waypoint_mode == WaypointMode::kDiffIk) {
      const std::string share = ament_index_cpp::get_package_share_directory(
        "franka_description");
      diff_ik_source_ = std::make_unique<DiffIkWaypointSource>(
        arm_kinematics::loadFrankaFerModel(
          share + "/robots/fer/kinematics.yaml",
          share + "/robots/fer/joint_limits.yaml"),
        std::make_shared<PickPlaceCartesianWaypointSource>(config_.task.geometry()));
      waypoint_source_ = diff_ik_source_.get();
    } else if (config_.waypoint_mode == WaypointMode::kKeyframe) {
      waypoint_source_ = &keyframe_source_;
    } else {
      throw std::invalid_argument("waypoint_source must be diff_ik or keyframe");
    }
    // Between the reset and the first admitted observation a vision executor waits for the
    // initial pose to be latched, which takes longer than the 5 s that is right for a stalled
    // stream; the oracle source keeps the plain watchdog.
    std::optional<std::chrono::steady_clock::duration> awaiting_timeout;
    if (config_.observation_source == ObservationSource::kVision) {
      awaiting_timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(config_.latch.timeout_s));
    }
    controller_ = std::make_unique<EpisodeController>(
      *waypoint_source_, config_.fsm, diff_ik_source_.get(), std::chrono::seconds(5),
      awaiting_timeout);
    RCLCPP_INFO(get_logger(), "waypoint source: %s", waypointModeName(config_.waypoint_mode));
    RCLCPP_INFO(
      get_logger(), "observation source: %s (vision confidence>=%.3f residual<=%.4fm inlier>=%.3f)",
      observationSourceName(config_.observation_source), config_.vision_min_confidence,
      config_.vision_max_residual_m, config_.vision_min_inlier_ratio);
    RCLCPP_INFO(
      get_logger(),
      "placement configuration: task_tcp_xy=[%.5f %.5f] verify_box_xy=[%.5f %.5f] "
      "verify_radius=%.5fm allow_target_mismatch=%d",
      config_.task.tcp_target_x_m, config_.task.tcp_target_y_m,
      config_.verification.box_target_x_m, config_.verification.box_target_y_m,
      config_.verification.radius_m, config_.allow_target_mismatch);

    LatchParams box_latch_params;
    box_latch_params.frames = config_.latch.frames;
    box_latch_params.max_position_spread_m = config_.latch.max_position_spread_m;
    box_latch_params.max_yaw_spread_rad = config_.latch.max_yaw_spread_rad;
    box_latch_params.yaw_period_rad = M_PI / 2.0;  // a square box repeats every 90 degrees
    LatchParams bin_latch_params = box_latch_params;
    bin_latch_params.yaw_period_rad = M_PI;  // a rectangle repeats every 180 degrees
    box_latch_ = std::make_unique<PoseLatch>(box_latch_params);
    bin_latch_ = std::make_unique<PoseLatch>(bin_latch_params);

    joint_command_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/mujoco_bridge/joint_command", rclcpp::QoS(10));
    gripper_command_pub_ = create_publisher<control_msgs::msg::GripperCommand>(
      "/mujoco_bridge/gripper_command", rclcpp::QoS(10));
    reset_client_ = create_client<manipulation_interfaces::srv::ResetScene>(
      "/mujoco_bridge/reset_with_generation");
    observation_sub_ = create_subscription<manipulation_interfaces::msg::BridgeObservation>(
      "/mujoco_bridge/episode_observation", rclcpp::QoS(10),
      std::bind(&TaskExecutorNode::onObservation, this, std::placeholders::_1));

    // The bridge's true bin pose, for the oracle source only (Week 4.1 Stage 8). Latched: the
    // bridge publishes it once at startup, and the bin does not move.
    if (config_.observation_source == ObservationSource::kOracle && config_.place_into_bin) {
      oracle_bin_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mujoco_bridge/ground_truth/bin_pose", rclcpp::QoS(1).transient_local(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {oracle_bin_ = msg;});
    }
    controller_->setPlacement(
      config_.task.fixedPlace(), config_.verification.box_target_x_m,
      config_.verification.box_target_y_m);
    initial_box_sub_ = create_subscription<manipulation_interfaces::msg::InitialBoxPose>(
      "/object_pose_estimator/initial_box_pose", rclcpp::QoS(10),
      [this](const manipulation_interfaces::msg::InitialBoxPose::SharedPtr msg) {
        bindLatchToEpisode();
        box_latch_->offer(latchSample(*msg));
        latest_initial_box_ = msg;
      });
    initial_bin_sub_ = create_subscription<manipulation_interfaces::msg::InitialBinPose>(
      "/object_pose_estimator/initial_bin_pose", rclcpp::QoS(10),
      [this](const manipulation_interfaces::msg::InitialBinPose::SharedPtr msg) {
        bindLatchToEpisode();
        bin_latch_->offer(latchSample(*msg));
      });
    latch_pub_ = create_publisher<manipulation_interfaces::msg::InitialPoseLatch>(
      "~/initial_pose_latch", rclcpp::QoS(10));

    // Private names, same reasoning as mujoco_bridge's ~/reset (architecture.md
    // 2.2): a capability of *this* node instance, not a system-wide singleton.
    // week2.md Stage J: the episode runner is the intended (only expected) caller
    // of ~/start_episode, and the only expected subscriber of ~/episode_outcome.
    start_episode_sub_ = create_subscription<std_msgs::msg::Empty>(
      "~/start_episode", rclcpp::QoS(10),
      std::bind(&TaskExecutorNode::onStartEpisode, this, std::placeholders::_1));
    episode_outcome_pub_ = create_publisher<manipulation_interfaces::msg::EpisodeOutcome>(
      "~/episode_outcome", rclcpp::QoS(10));

    // A fixed 20Hz decision tick, not the physics rate: this node reasons about
    // phases and settled joint targets, not individual mj_step calls. Triggered by
    // a wall timer (rclcpp::TimerBase has no sim-time variant); the controller
    // receives simulation time separately for phase durations.
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&TaskExecutorNode::onTimer, this));
  }

private:
  // Keep the newest ROS sample; the timer performs conversion and admission.
  void onObservation(const manipulation_interfaces::msg::BridgeObservation::SharedPtr msg)
  {
    if (bridge_attachment_known_ &&
      last_bridge_attachment_state_ ==
      manipulation_interfaces::msg::BridgeObservation::ATTACHMENT_ATTACHED &&
      msg->attachment_state !=
      manipulation_interfaces::msg::BridgeObservation::ATTACHMENT_ATTACHED)
    {
      // A release ends the previous visual pose's validity. The estimator will
      // publish a fresh MEASURED result after it resumes.
      latest_initial_box_.reset();
      last_vision_sequence_processed_ = 0;
    }
    bridge_attachment_known_ = true;
    last_bridge_attachment_state_ = msg->attachment_state;
    latest_observation_ = msg;
    observation_cache_[msg->sample_sequence] = msg;
    while (observation_cache_.size() > 30) {observation_cache_.erase(observation_cache_.begin());}
  }

  // The lifecycle the latches belong to: bind them to the controller's current session and
  // generation, and drop everything from the one before. A retry has a new generation.
  void bindLatchToEpisode()
  {
    const auto state = controller_->state();
    if (state != EpisodeState::kAwaitingObservation && state != EpisodeState::kReady) {
      return;
    }
    if (latch_bound_ && latch_session_ == controller_->bridgeSession() &&
      latch_generation_ == controller_->generation())
    {
      return;
    }
    latch_bound_ = true;
    latch_session_ = controller_->bridgeSession();
    latch_generation_ = controller_->generation();
    latch_failed_ = false;
    box_latch_->reset(latch_session_, latch_generation_);
    bin_latch_->reset(latch_session_, latch_generation_);
  }

  void forgetLatch()
  {
    latch_bound_ = false;
    latch_failed_ = false;
  }

  template<typename Message>
  static LatchSample latchSample(const Message & message)
  {
    LatchSample sample;
    sample.bridge_session = message.bridge_session;
    sample.generation = message.generation;
    sample.sequence = message.sample_sequence;
    sample.stamp_s = rclcpp::Time(message.header.stamp).seconds();
    sample.measured = message.state == Message::MEASURED;
    sample.reason = message.reason;
    sample.x = message.position.x;
    sample.y = message.position.y;
    sample.z = message.position.z;
    sample.yaw_rad = message.yaw_rad;
    return sample;
  }

  bool latchComplete() const
  {
    return latch_bound_ && box_latch_->latched() &&
           (!config_.place_into_bin || bin_latch_->latched());
  }

  // Hands the controller this episode's place target before anything is admitted. Into the
  // bin: the latched vision bin, or the bridge's true bin for the oracle source; returns false
  // while that is not known yet, so nothing is admitted and no command is sent. Not into the
  // bin: the configured fixed target (the legacy scene has no bin).
  bool applyPlacement()
  {
    if (!config_.place_into_bin) {
      controller_->setPlacement(
        config_.task.fixedPlace(), config_.verification.box_target_x_m,
        config_.verification.box_target_y_m);
      return true;
    }
    PlaceTarget bin;
    if (config_.observation_source == ObservationSource::kOracle) {
      if (!oracle_bin_) {
        last_observation_failure_reason_ = "ORACLE_BIN_POSE_MISSING";
        return false;
      }
      const auto & q = oracle_bin_->pose.orientation;
      bin = {oracle_bin_->pose.position.x, oracle_bin_->pose.position.y,
        oracle_bin_->pose.position.z,
        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
    } else {
      if (!latchComplete()) {
        return false;
      }
      const auto & latched = *bin_latch_->pose();
      bin = {latched.x, latched.y, latched.z, latched.yaw_rad};
    }
    controller_->setPlacement(bin, bin.x, bin.y, true);
    return true;
  }

  // Which of the objects the wait is still missing, and why, for the outcome.
  std::string latchWaitReason() const
  {
    std::string reason;
    if (!box_latch_->latched()) {reason += "BOX:" + box_latch_->status();}
    if (config_.place_into_bin && !bin_latch_->latched()) {
      reason += (reason.empty() ? "" : " ") + std::string("BIN:") + bin_latch_->status();
    }
    return reason;
  }

  manipulation_interfaces::msg::LatchedPose latchedPoseMessage(
    const PoseLatch & latch, bool required) const
  {
    manipulation_interfaces::msg::LatchedPose message;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    message.required = required;
    message.latched = latch.latched();
    message.status = latch.status();
    message.position.x = message.position.y = message.position.z = nan;
    message.yaw_rad = message.position_spread_m = message.yaw_spread_rad = nan;
    if (latch.pose()) {
      const auto & pose = *latch.pose();
      message.position.x = pose.x;
      message.position.y = pose.y;
      message.position.z = pose.z;
      message.yaw_rad = pose.yaw_rad;
      message.source_sequence = pose.sequence;
      message.source_stamp = rclcpp::Time(static_cast<int64_t>(pose.stamp_s * 1e9));
      message.position_spread_m = pose.position_spread_m;
      message.yaw_spread_rad = pose.yaw_spread_rad;
    }
    return message;
  }

  void publishLatchStatus()
  {
    if (config_.observation_source != ObservationSource::kVision || !latch_bound_) {
      return;
    }
    manipulation_interfaces::msg::InitialPoseLatch message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "world";
    message.bridge_session = latch_session_;
    message.generation = latch_generation_;
    message.state = latch_failed_ ? manipulation_interfaces::msg::InitialPoseLatch::FAILED :
      (latchComplete() ? manipulation_interfaces::msg::InitialPoseLatch::LATCHED :
      manipulation_interfaces::msg::InitialPoseLatch::WAITING);
    message.box = latchedPoseMessage(*box_latch_, true);
    message.bin = latchedPoseMessage(*bin_latch_, config_.place_into_bin);
    latch_pub_->publish(message);
  }

  // Convert one bridge message and pass it through the controller's freshness gate.
  void collectObservation()
  {
    manipulation_interfaces::msg::BridgeObservation::SharedPtr bridge;
    geometry_msgs::msg::PoseStamped object_pose;
    std::string object_source = observationSourceName(config_.observation_source);
    double confidence = config_.observation_source == ObservationSource::kOracle ? 1.0 : 0.0;
    double residual_m = 0.0;
    std::string state_reason;

    bridge = latest_observation_;
    if (!bridge) {return;}
    if (config_.observation_source == ObservationSource::kVision) {
      bindLatchToEpisode();
    }
    if (!applyPlacement()) {
      return;  // waiting for the bin
    }
    if (config_.observation_source == ObservationSource::kOracle) {
      object_pose = bridge->object_pose;
    } else if (controller_->phase() != Phase::kVerify) {
      // Every phase but VERIFY works on the pose latched at the start of the episode, not on
      // a stream that keeps changing; OPEN and RETRACT do not need the box at all, and while
      // the open hand is still over the bin it can hide the box (Week 4.1 Stage 11, 11.3).
      // Until it is latched nothing is admitted, so the controller stays in its wait and sends
      // no command.
      if (!latchComplete()) {
        return;
      }
      const auto & box = *box_latch_->pose();
      object_pose.header = bridge->joint_state.header;
      object_pose.header.frame_id = "world";
      object_pose.pose.position.x = box.x;
      object_pose.pose.position.y = box.y;
      object_pose.pose.position.z = box.z;
      object_pose.pose.orientation.w = std::cos(box.yaw_rad / 2.0);
      object_pose.pose.orientation.z = std::sin(box.yaw_rad / 2.0);
      // The latch has no confidence or residual: the status message carries its quality.
      confidence = std::numeric_limits<double>::quiet_NaN();
      residual_m = std::numeric_limits<double>::quiet_NaN();
      state_reason = "LATCHED";
      last_observation_source_ = object_source;
      last_observation_confidence_ = confidence;
      last_observation_residual_m_ = residual_m;
    } else {
      // VERIFY (Week 4.1 Stage 11): where the box is NOW, from the initial-pose estimator,
      // which emptied its window at the release and measures the box wherever it landed, in
      // the bin as well as on the table. The older ~/object_pose detector is not used: it
      // rejects a box lying in the bin.
      if (!latest_initial_box_) {return;}
      const auto vision = latest_initial_box_;
      const auto bridge_it = observation_cache_.find(vision->sample_sequence);
      if (bridge_it == observation_cache_.end() ||
        bridge_it->second->bridge_session != vision->bridge_session ||
        bridge_it->second->generation != vision->generation ||
        vision->sample_sequence <= last_vision_sequence_processed_)
      {
        return;
      }
      bridge = bridge_it->second;
      last_vision_sequence_processed_ = vision->sample_sequence;
      confidence = std::numeric_limits<double>::quiet_NaN();
      residual_m = std::numeric_limits<double>::quiet_NaN();
      state_reason = vision->reason;
      last_observation_source_ = object_source;
      last_observation_confidence_ = confidence;
      last_observation_residual_m_ = residual_m;
      if (vision->state != manipulation_interfaces::msg::InitialBoxPose::MEASURED) {
        if (vision->state == manipulation_interfaces::msg::InitialBoxPose::NOT_MEASURED &&
          vision->bridge_session == controller_->bridgeSession() &&
          vision->generation == controller_->generation())
        {
          last_observation_failure_reason_ = "VISION_REJECTED:" + vision->reason;
          last_observation_rejected_ = true;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "box after the release not measured (%s); waiting", vision->reason.c_str());
        }
        return;
      }
      object_pose.header = vision->header;
      object_pose.header.frame_id = "world";
      object_pose.pose.position = vision->position;
      object_pose.pose.orientation.w = std::cos(vision->yaw_rad / 2.0);
      object_pose.pose.orientation.z = std::sin(vision->yaw_rad / 2.0);
    }

    auto snapshot = makeObservationSnapshot(
      bridge->joint_state, object_pose, bridge->left_finger_contact, bridge->right_finger_contact,
      bridge->world_to_hand_tcp);
    if (!snapshot) {
      return;
    }
    ObservationEnvelope envelope;
    envelope.frame = *snapshot;
    envelope.bridge_session = bridge->bridge_session;
    envelope.generation = bridge->generation;
    envelope.sample_sequence = bridge->sample_sequence;
    envelope.sim_time_s = rclcpp::Time(bridge->joint_state.header.stamp).seconds();
    envelope.frame.object_source = object_source;
    envelope.frame.object_confidence = confidence;
    envelope.frame.object_residual_m = residual_m;
    envelope.frame.object_state_reason = state_reason;
    envelope.frame.attachment_state = bridge->attachment_state;
    last_observation_source_ = object_source;
    last_observation_confidence_ = confidence;
    last_observation_residual_m_ = residual_m;
    last_observation_rejected_ = false;
    if (config_.place_into_bin && controller_->phase() == Phase::kVerify) {
      const auto & place = controller_->placeTarget();
      const auto & q = object_pose.pose.orientation;
      const Containment containment = boxInBin(
        {object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z,
          std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))},
        {place.x, place.y, place.support_z, place.yaw_rad});
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "verify: box xyz=[%.4f %.4f %.4f] in bin: %s, smallest corner clearance %.1f mm "
        "(needs >= 5.0), height error %.1f mm (needs |e| <= 5.0)",
        object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z,
        containment.inside ? "yes" : "no", containment.min_clearance_m * 1000.0,
        containment.height_error_m * 1000.0);
    }
    const auto actions = controller_->onObservation(
      envelope, std::chrono::steady_clock::now(), get_clock()->now().seconds());
    // Admission can end the episode on timeout or supersession before tick runs.
    executeActions(actions);
    if (actions.finished) {return;}
    if (actions.diagnostics.empty() ||
      actions.diagnostics.front().kind != DiagnosticEvent::Kind::kObservationAccepted)
    {
      return;
    }
    if (!observation_generation_logged_) {
      const auto & place = controller_->placeTarget();
      RCLCPP_INFO(
        get_logger(), "accepted observation session=%" PRIu64 " generation=%" PRIu64
        " stamp=%.3fs; place target xy=[%.5f %.5f] support_z=%.4f (%s)",
        controller_->bridgeSession(), controller_->generation(), envelope.sim_time_s,
        place.x, place.y, place.support_z,
        !config_.place_into_bin ? "fixed" :
        (config_.observation_source == ObservationSource::kOracle ? "oracle bin" :
        "latched vision bin"));
      observation_generation_logged_ = true;
    }
  }

  // Translate one domain target into the bridge's arm and gripper commands.
  void publishTarget(const JointTarget & target)
  {
    trajectory_msgs::msg::JointTrajectory joint_msg;
    joint_msg.joint_names.assign(kArmJointNames.begin(), kArmJointNames.end());
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(target.arm_positions.begin(), target.arm_positions.end());
    joint_msg.points.push_back(point);
    joint_command_pub_->publish(joint_msg);

    control_msgs::msg::GripperCommand gripper_msg;
    gripper_msg.position = target.gripper_width_m;
    gripper_msg.max_effort = 0.0;
    gripper_command_pub_->publish(gripper_msg);
  }

  // Send a reset only when the service is ready; confirm only a submitted request.
  void requestReset(const ResetRequest & reset)
  {
    if (!reset_client_->service_is_ready()) {
      RCLCPP_WARN_ONCE(get_logger(), "reset_with_generation not available yet");
      return;
    }
    const uint64_t request_id = reset.request_id;
    auto request = std::make_shared<manipulation_interfaces::srv::ResetScene::Request>();
    try {
      reset_client_->async_send_request(
        request, [this, request_id](
          rclcpp::Client<manipulation_interfaces::srv::ResetScene>::SharedFuture future) {
          // A later start or retry may have replaced this request while it was in flight.
          if (controller_->requestId() != request_id) {
            return;
          }
          ResetReceipt receipt;
          try {
            const auto response = future.get();
            receipt = {response->success, response->bridge_session, response->generation};
            RCLCPP_INFO(
              get_logger(), "reset: success=%d session=%" PRIu64 " generation=%" PRIu64
              " message='%s'", response->success, response->bridge_session,
              response->generation, response->message.c_str());
          } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "reset service failed: %s", e.what());
          }
          const auto actions = controller_->onResetResponse(
            request_id, receipt, get_clock()->now().seconds());
          executeActions(actions);
        });
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "reset request not sent: %s", e.what());
      return;
    }
    controller_->onResetRequestSent(request_id);
  }

  // Replace any current episode and execute the new reset request.
  // The new token makes delayed replies from the previous episode harmless.
  void onStartEpisode(const std_msgs::msg::Empty::SharedPtr)
  {
    latest_observation_.reset();
    latest_initial_box_.reset();
    observation_cache_.clear();
    last_vision_sequence_processed_ = 0;
    bridge_attachment_known_ = false;
    last_bridge_attachment_state_ =
      manipulation_interfaces::msg::BridgeObservation::ATTACHMENT_NOT_ATTACHED;
    last_observation_failure_reason_.clear();
    last_observation_source_ = observationSourceName(config_.observation_source);
    last_observation_confidence_ = config_.observation_source ==
      ObservationSource::kOracle ? 1.0 : 0.0;
    last_observation_residual_m_ = 0.0;
    last_observation_rejected_ = false;
    observation_generation_logged_ = false;
    forgetLatch();
    const auto actions = controller_->startEpisode(
      std::chrono::steady_clock::now(), get_clock()->now().seconds());
    executeActions(actions);
    RCLCPP_INFO(
      get_logger(), "episode placement: task_tcp_xy=[%.5f %.5f] verify_box_xy=[%.5f %.5f]",
      config_.task.tcp_target_x_m, config_.task.tcp_target_y_m,
      config_.verification.box_target_x_m, config_.verification.box_target_y_m);
    RCLCPP_INFO(get_logger(), "episode start requested");
  }

  // Serialize the final telemetry into the public outcome message and log placement.
  void publishEpisodeOutcome(const EpisodeFinished & finished)
  {
    manipulation_interfaces::msg::EpisodeOutcome outcome;
    outcome.success = finished.success;
    outcome.failure_code = finished.failure_code;
    outcome.observation_source = last_observation_source_;
    outcome.observation_confidence = last_observation_confidence_;
    outcome.observation_residual_m = last_observation_residual_m_;
    outcome.observation_failure_layer = finished.success ? "" :
      (finished.failure_code.rfind("VISION_", 0) == 0 ? "perception" : "execution");
    outcome.observation_failure_reason = last_observation_failure_reason_;
    outcome.retries = finished.retry_count;
    finished.telemetry.appendTo(outcome);
    episode_outcome_pub_->publish(outcome);
    RCLCPP_INFO(
      get_logger(), "episode outcome published: success=%d failure_code=%s retries=%u",
      outcome.success, outcome.failure_code.c_str(), outcome.retries);
    if (controller_->latestObservation() && !last_observation_rejected_) {
      const auto & pose = controller_->latestObservation()->frame.object_pose;
      const double dx = pose.x -
        config_.verification.box_target_x_m;
      const double dy = pose.y -
        config_.verification.box_target_y_m;
      RCLCPP_INFO(
        get_logger(),
        "final placement: box_xy=[%.5f %.5f] verify_xy=[%.5f %.5f] error=%.5fm",
        pose.x, pose.y,
        config_.verification.box_target_x_m, config_.verification.box_target_y_m,
        std::hypot(dx, dy));
    }
  }

  // Apply controller decisions in order: current target, transition, reset, outcome.
  void executeActions(const EpisodeActions & actions)
  {
    // Log the solved TCP target once on phase entry, before its command is published.
    if (actions.target_diagnostics) {
      const auto & data = *actions.target_diagnostics;
      const Eigen::Quaterniond target_rotation(data.tcp_target.linear());
      RCLCPP_INFO(
        get_logger(),
        "phase %s target_frame=world tcp_xyz=[%.4f %.4f %.4f] "
        "tcp_qwxyz=[%.4f %.4f %.4f %.4f] "
        "ik_iterations=%zu ik_pos_err=%.6fm ik_rot_err=%.6frad sigma_min=%.6f",
        phaseName(actions.target->phase), data.tcp_target.translation().x(),
        data.tcp_target.translation().y(), data.tcp_target.translation().z(),
        target_rotation.w(), target_rotation.x(), target_rotation.y(), target_rotation.z(),
        data.ik.iterations, data.ik.position_error, data.ik.orientation_error,
        data.ik.minimum_singular_value);
    }
    // A transition still commands the phase being left on this tick.
    if (actions.target) {
      publishTarget(actions.target->target);
    }
    // Report the phase decision only after publishing that phase's target.
    if (actions.transition) {
      const auto & transition = *actions.transition;
      const auto & target = transition.target.arm_positions;
      RCLCPP_INFO(
        get_logger(),
        "phase %s -> %s: target=[%.3f %.3f %.3f %.3f %.3f %.3f %.3f] "
        "joint_err=%.4frad tcp_pos_err=%.4fm tcp_rot_err=%.4frad "
        "elapsed=%.2fs exit=%s",
        phaseName(transition.from), phaseName(transition.to), target[0], target[1],
        target[2], target[3], target[4], target[5], target[6],
        transition.telemetry.joint_tracking_error_rad,
        transition.telemetry.actual_tcp_position_error_m,
        transition.tcp_rotation_error_rad, transition.telemetry.duration_s,
        exitReasonName(transition.reason));
    }
    // Admission logs are handled earlier; this path logs IK exceptions.
    for (const auto & diagnostic : actions.diagnostics) {
      // IK failed before a valid target could be emitted.
      if (diagnostic.kind == DiagnosticEvent::Kind::kIkFailed) {
        RCLCPP_ERROR(
          get_logger(), "phase %s IK_FAILED: %s", phaseName(controller_->phase()),
          diagnostic.code.c_str());
      }
    }
    // Drop the old ROS sample before submitting a new reset generation.
    if (actions.reset_request) {
      latest_observation_.reset();
      latest_initial_box_.reset();
      observation_cache_.clear();
      last_vision_sequence_processed_ = 0;
      bridge_attachment_known_ = false;
      last_bridge_attachment_state_ =
        manipulation_interfaces::msg::BridgeObservation::ATTACHMENT_NOT_ATTACHED;
      observation_generation_logged_ = false;
      last_observation_rejected_ = false;
      forgetLatch();  // a retry is a new generation: detect and latch again
      requestReset(*actions.reset_request);
    }
    // The controller emits this only once; publish after any same-tick target.
    if (actions.finished) {
      EpisodeFinished finished = *actions.finished;
      // A vision episode that never got past the wait, while the bridge was publishing for
      // this generation, failed to latch the initial pose: say so (perception layer) instead
      // of the generic stale observation.
      if (!finished.success && finished.failure_code == "OBSERVATION_STALE" &&
        config_.observation_source == ObservationSource::kVision && latch_bound_ &&
        !latchComplete() && latest_observation_ &&
        latest_observation_->generation == controller_->generation())
      {
        finished.failure_code = "VISION_LATCH_TIMEOUT";
        last_observation_failure_reason_ = latchWaitReason();
        latch_failed_ = true;
        publishLatchStatus();
      }
      // Keep the failure context in logs without changing the outcome value.
      if (!finished.success) {
        RCLCPP_ERROR(
          get_logger(), "episode %s: session=%" PRIu64 " expected generation=%" PRIu64
          ", latest=%" PRIu64 "%s%s", finished.failure_code.c_str(),
          controller_->bridgeSession(), controller_->generation(),
          latest_observation_ ? latest_observation_->generation : uint64_t{0},
          last_observation_failure_reason_.empty() ? "" : " ",
          last_observation_failure_reason_.c_str());
      }
      publishEpisodeOutcome(finished);
    }
  }

  // Drive one admission and decision cycle with separate sim and steady clocks.
  void onTimer()
  {
    collectObservation();
    publishLatchStatus();
    const auto actions = controller_->tick(
      get_clock()->now().seconds(), std::chrono::steady_clock::now());
    executeActions(actions);
  }

  KeyframeWaypointSource keyframe_source_;
  std::unique_ptr<DiffIkWaypointSource> diff_ik_source_;
  WaypointSource * waypoint_source_ = nullptr;
  std::unique_ptr<EpisodeController> controller_;
  TaskExecutorConfig config_;
  bool observation_generation_logged_ = false;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<control_msgs::msg::GripperCommand>::SharedPtr gripper_command_pub_;
  rclcpp::Publisher<manipulation_interfaces::msg::EpisodeOutcome>::SharedPtr
    episode_outcome_pub_;
  rclcpp::Client<manipulation_interfaces::srv::ResetScene>::SharedPtr reset_client_;
  rclcpp::Subscription<manipulation_interfaces::msg::BridgeObservation>::SharedPtr
    observation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr oracle_bin_sub_;
  geometry_msgs::msg::PoseStamped::SharedPtr oracle_bin_;
  rclcpp::Subscription<manipulation_interfaces::msg::InitialBoxPose>::SharedPtr initial_box_sub_;
  rclcpp::Subscription<manipulation_interfaces::msg::InitialBinPose>::SharedPtr initial_bin_sub_;
  rclcpp::Publisher<manipulation_interfaces::msg::InitialPoseLatch>::SharedPtr latch_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_episode_sub_;
  std::unique_ptr<PoseLatch> box_latch_;
  std::unique_ptr<PoseLatch> bin_latch_;
  bool latch_bound_ = false;
  uint64_t latch_session_ = 0;
  uint64_t latch_generation_ = 0;
  bool latch_failed_ = false;
  manipulation_interfaces::msg::BridgeObservation::SharedPtr latest_observation_;
  manipulation_interfaces::msg::InitialBoxPose::SharedPtr latest_initial_box_;
  std::map<uint64_t, manipulation_interfaces::msg::BridgeObservation::SharedPtr>
  observation_cache_;
  uint64_t last_vision_sequence_processed_ = 0;
  bool bridge_attachment_known_ = false;
  uint8_t last_bridge_attachment_state_ =
    manipulation_interfaces::msg::BridgeObservation::ATTACHMENT_NOT_ATTACHED;
  std::string last_observation_source_ = "oracle";
  double last_observation_confidence_ = 1.0;
  double last_observation_residual_m_ = 0.0;
  bool last_observation_rejected_ = false;
  std::string last_observation_failure_reason_;
};

}  // namespace task_executor

// Start the single-threaded ROS executor used by the controller's callback order.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<task_executor::TaskExecutorNode> node;
  try {
    node = std::make_shared<task_executor::TaskExecutorNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("task_executor"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
