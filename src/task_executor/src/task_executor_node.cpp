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
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/msg/gripper_command.hpp>
#include <manipulation_interfaces/msg/bridge_observation.hpp>
#include <manipulation_interfaces/msg/episode_outcome.hpp>
#include <manipulation_interfaces/srv/reset_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "task_executor/cartesian_waypoint_source.hpp"
#include "task_executor/diff_ik_waypoint_source.hpp"
#include "task_executor/episode_controller.hpp"
#include "task_executor/keyframe_waypoint_source.hpp"
#include "task_executor/observation_snapshot.hpp"
#include "task_executor/phase.hpp"
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
    controller_ = std::make_unique<EpisodeController>(
      *waypoint_source_, config_.fsm, diff_ik_source_.get());
    RCLCPP_INFO(get_logger(), "waypoint source: %s", waypointModeName(config_.waypoint_mode));
    RCLCPP_INFO(
      get_logger(),
      "placement configuration: task_tcp_xy=[%.5f %.5f] verify_box_xy=[%.5f %.5f] "
      "verify_radius=%.5fm allow_target_mismatch=%d",
      config_.task.tcp_target_x_m, config_.task.tcp_target_y_m,
      config_.verification.box_target_x_m, config_.verification.box_target_y_m,
      config_.verification.radius_m, config_.allow_target_mismatch);

    joint_command_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/mujoco_bridge/joint_command", rclcpp::QoS(10));
    gripper_command_pub_ = create_publisher<control_msgs::msg::GripperCommand>(
      "/mujoco_bridge/gripper_command", rclcpp::QoS(10));
    reset_client_ = create_client<manipulation_interfaces::srv::ResetScene>(
      "/mujoco_bridge/reset_with_generation");
    observation_sub_ = create_subscription<manipulation_interfaces::msg::BridgeObservation>(
      "/mujoco_bridge/episode_observation", rclcpp::QoS(10),
      std::bind(&TaskExecutorNode::onObservation, this, std::placeholders::_1));

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
    latest_observation_ = msg;
  }

  // Convert one bridge message and pass it through the controller's freshness gate.
  void collectObservation()
  {
    if (!latest_observation_) {
      return;
    }
    auto snapshot = makeObservationSnapshot(
      latest_observation_->joint_state, latest_observation_->object_pose,
      latest_observation_->left_finger_contact, latest_observation_->right_finger_contact,
      latest_observation_->world_to_hand_tcp);
    if (!snapshot) {
      return;
    }
    ObservationEnvelope envelope;
    envelope.frame = *snapshot;
    envelope.bridge_session = latest_observation_->bridge_session;
    envelope.generation = latest_observation_->generation;
    envelope.sample_sequence = latest_observation_->sample_sequence;
    envelope.sim_time_s = rclcpp::Time(latest_observation_->joint_state.header.stamp).seconds();
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
      RCLCPP_INFO(
        get_logger(), "accepted observation session=%" PRIu64 " generation=%" PRIu64
        " stamp=%.3fs", controller_->bridgeSession(), controller_->generation(),
        envelope.sim_time_s);
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
    observation_generation_logged_ = false;
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
    outcome.retries = finished.retry_count;
    finished.telemetry.appendTo(outcome);
    episode_outcome_pub_->publish(outcome);
    RCLCPP_INFO(
      get_logger(), "episode outcome published: success=%d failure_code=%s retries=%u",
      outcome.success, outcome.failure_code.c_str(), outcome.retries);
    if (latest_observation_) {
      const double dx = latest_observation_->object_pose.pose.position.x -
        config_.verification.box_target_x_m;
      const double dy = latest_observation_->object_pose.pose.position.y -
        config_.verification.box_target_y_m;
      RCLCPP_INFO(
        get_logger(),
        "final placement: box_xy=[%.5f %.5f] verify_xy=[%.5f %.5f] error=%.5fm",
        latest_observation_->object_pose.pose.position.x,
        latest_observation_->object_pose.pose.position.y,
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
      observation_generation_logged_ = false;
      requestReset(*actions.reset_request);
    }
    // The controller emits this only once; publish after any same-tick target.
    if (actions.finished) {
      // Keep the failure context in logs without changing the outcome value.
      if (!actions.finished->success) {
        RCLCPP_ERROR(
          get_logger(), "episode %s: session=%" PRIu64 " expected generation=%" PRIu64
          ", latest=%" PRIu64, actions.finished->failure_code.c_str(),
          controller_->bridgeSession(), controller_->generation(),
          latest_observation_ ? latest_observation_->generation : uint64_t{0});
      }
      publishEpisodeOutcome(*actions.finished);
    }
  }

  // Drive one admission and decision cycle with separate sim and steady clocks.
  void onTimer()
  {
    collectObservation();
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
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_episode_sub_;
  manipulation_interfaces::msg::BridgeObservation::SharedPtr latest_observation_;
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
