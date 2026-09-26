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

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <manipulation_interfaces/msg/episode_outcome.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "task_executor/fsm.hpp"
#include "task_executor/cartesian_waypoint_source.hpp"
#include "task_executor/diff_ik_waypoint_source.hpp"
#include "task_executor/keyframe_waypoint_source.hpp"
#include "task_executor/phase.hpp"
#include "task_executor/waypoint_source.hpp"

namespace task_executor
{

namespace
{
constexpr std::array<const char *, 7> kArmJointNames = {
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
}  // namespace

// FSM glue (week2.md Stage I): reads /joint_states + mujoco_bridge's
// ~/ground_truth/* topics, calls the pure step() function in fsm.hpp/cpp once per
// tick, and publishes whatever JointTarget the current phase's WaypointSource
// returns. Deliberately thin and untested -- same "do not mock onTimer" discipline
// mujoco_bridge_node.cpp follows (week2.md 2.1.1 point 3): all the actual
// decision-making already lives in fsm.cpp (unit tested) and the selected
// WaypointSource.
//
// use_sim_time is NOT declared here explicitly -- rclcpp::Node auto-declares it,
// and demo.launch.py sets it via node parameters the same way it does for
// mujoco_bridge_node. Every elapsed-time computation below uses
// get_clock()->now(), never a std::chrono wall clock -- this node is downstream of
// /clock, not the source of it (CLAUDE.md's "两侧都禁止" rule).
class TaskExecutorNode : public rclcpp::Node
{
public:
  TaskExecutorNode()
  : Node("task_executor")
  {
    params_.grasp_criteria.box_width_m = declare_parameter("grasp.box_width_m", 0.04);
    params_.grasp_criteria.width_epsilon_m = declare_parameter("grasp.width_epsilon_m", 0.01);
    params_.grasp_criteria.lift_height_threshold_m =
      declare_parameter("grasp.lift_height_threshold_m", 0.26);
    params_.grasp_criteria.region_radius_m = declare_parameter("grasp.region_radius_m", 0.05);

    params_.position_epsilon_rad = declare_parameter("fsm.position_epsilon_rad", 0.05);
    params_.grasp_position_epsilon_rad =
      declare_parameter("fsm.grasp_position_epsilon_rad", 0.3);
    params_.velocity_epsilon_rad_s = declare_parameter("fsm.velocity_epsilon_rad_s", 0.05);
    params_.min_settle_s = declare_parameter("fsm.min_settle_s", 0.5);
    // These two were accidentally left off this list -- both silently used the
    // FsmParams struct default (2.0s each) with no way to override at runtime,
    // breaking the "all tunable thresholds live in FsmParams / are ROS params"
    // consistency this constructor is otherwise deliberate about (week2.md 10.3.2
    // caught this while preparing to run the week2.md 10.10.2.1 diagnostic plan,
    // which needs to tune lift_settle_grace_s down to reproduce the original bug).
    params_.lift_settle_grace_s = declare_parameter("fsm.lift_settle_grace_s", 2.0);
    params_.close_settle_s = declare_parameter("fsm.close_settle_s", 2.0);
    params_.phase_timeout_s = declare_parameter("fsm.phase_timeout_s", 6.0);
    params_.max_retries = declare_parameter("fsm.max_retries", 3);

    const std::string waypoint_mode = declare_parameter("waypoint_source", "diff_ik");
    PickPlaceGeometry geometry;
    geometry.place_x_m = declare_parameter("target.place_x_m", 0.5);
    geometry.place_y_m = declare_parameter("target.place_y_m", 0.3);
    geometry.hover_height_m = declare_parameter("target.hover_height_m", 0.15);
    geometry.place_tcp_above_box_center_m = declare_parameter(
      "target.place_tcp_above_box_center_m", 0.05);
    geometry.tool_yaw_rad = declare_parameter("target.tool_yaw_rad", 0.0);
    // The legacy joint-space baseline still lands near its measured point.
    const bool legacy = waypoint_mode == "keyframe";
    params_.place_x_m = declare_parameter("verify.place_x_m", legacy ? 0.43 : 0.5);
    params_.place_y_m = declare_parameter("verify.place_y_m", legacy ? 0.31 : 0.3);
    params_.place_region_radius_m = declare_parameter("verify.place_region_radius_m", 0.08);

    if (waypoint_mode == "diff_ik") {
      const std::string share = ament_index_cpp::get_package_share_directory(
        "franka_description");
      diff_ik_source_ = std::make_unique<DiffIkWaypointSource>(
        arm_kinematics::loadFrankaFerModel(
          share + "/robots/fer/kinematics.yaml",
          share + "/robots/fer/joint_limits.yaml"),
        std::make_shared<PickPlaceCartesianWaypointSource>(geometry));
      waypoint_source_ = diff_ik_source_.get();
    } else if (waypoint_mode == "keyframe") {
      waypoint_source_ = &keyframe_source_;
    } else {
      throw std::invalid_argument("waypoint_source must be diff_ik or keyframe");
    }
    RCLCPP_INFO(get_logger(), "waypoint source: %s", waypoint_mode.c_str());

    joint_command_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/mujoco_bridge/joint_command", rclcpp::QoS(10));
    gripper_command_pub_ = create_publisher<control_msgs::msg::GripperCommand>(
      "/mujoco_bridge/gripper_command", rclcpp::QoS(10));
    reset_client_ = create_client<std_srvs::srv::Trigger>("/mujoco_bridge/reset");

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10),
      std::bind(&TaskExecutorNode::onJointState, this, std::placeholders::_1));
    object_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mujoco_bridge/ground_truth/object_pose", rclcpp::QoS(10),
      std::bind(&TaskExecutorNode::onObjectPose, this, std::placeholders::_1));
    left_contact_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/mujoco_bridge/ground_truth/left_finger_contact", rclcpp::QoS(10),
      std::bind(&TaskExecutorNode::onLeftContact, this, std::placeholders::_1));
    right_contact_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/mujoco_bridge/ground_truth/right_finger_contact", rclcpp::QoS(10),
      std::bind(&TaskExecutorNode::onRightContact, this, std::placeholders::_1));

    // Private names, same reasoning as mujoco_bridge's ~/reset (architecture.md
    // 2.2): a capability of *this* node instance, not a system-wide singleton.
    // week2.md Stage J: the episode runner is the intended (only expected) caller
    // of ~/start_episode, and the only expected subscriber of ~/episode_outcome.
    start_episode_sub_ = create_subscription<std_msgs::msg::Empty>(
      "~/start_episode", rclcpp::QoS(10),
      std::bind(&TaskExecutorNode::onStartEpisode, this, std::placeholders::_1));
    episode_outcome_pub_ = create_publisher<manipulation_interfaces::msg::EpisodeOutcome>(
      "~/episode_outcome", rclcpp::QoS(10));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // A fixed 20Hz decision tick, not the physics rate: this node reasons about
    // phases and settled joint targets, not individual mj_step calls. Triggered by
    // a wall timer (rclcpp::TimerBase has no sim-time variant), but every duration
    // used below (elapsed_in_phase_s) comes from get_clock()->now() deltas, which
    // is sim time once use_sim_time is set -- the trigger cadence itself does not
    // need to be sim-accurate, only the measurements taken at each trigger do.
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&TaskExecutorNode::onTimer, this));
  }

private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_joint_state_ = msg;
  }

  void onObjectPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_object_pose_ = msg;
  }

  void onLeftContact(const std_msgs::msg::Bool::SharedPtr msg) {left_contact_ = msg->data;}
  void onRightContact(const std_msgs::msg::Bool::SharedPtr msg) {right_contact_ = msg->data;}

  // Returns nullopt until every joint has been seen at least once. Searching
  // /joint_states by name every tick (rather than caching a name->index map, the
  // way mujoco_bridge_node.cpp does for its own performance-sensitive publish
  // path) is fine here: 9 names, 20 times a second, and this node never touches
  // mjData.
  std::optional<ArmState> extractArmState() const
  {
    if (!latest_joint_state_) {
      return std::nullopt;
    }
    ArmState arm;
    for (size_t i = 0; i < kArmJointNames.size(); ++i) {
      const auto it = std::find(
        latest_joint_state_->name.begin(), latest_joint_state_->name.end(),
        std::string(kArmJointNames[i]));
      if (it == latest_joint_state_->name.end()) {
        return std::nullopt;
      }
      const size_t idx = std::distance(latest_joint_state_->name.begin(), it);
      arm.positions[i] = latest_joint_state_->position[idx];
      arm.velocities[i] = latest_joint_state_->velocity[idx];
    }
    return arm;
  }

  std::optional<double> extractGripperWidth() const
  {
    if (!latest_joint_state_) {
      return std::nullopt;
    }
    double width = 0.0;
    int found = 0;
    for (size_t i = 0; i < latest_joint_state_->name.size(); ++i) {
      if (latest_joint_state_->name[i] == "finger_joint1" ||
        latest_joint_state_->name[i] == "finger_joint2")
      {
        width += latest_joint_state_->position[i];
        ++found;
      }
    }
    if (found != 2) {
      return std::nullopt;
    }
    return width;
  }

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

  static double maxAbsError(const std::array<double, 7> & a, const std::array<double, 7> & b)
  {
    double worst = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
      worst = std::max(worst, std::abs(a[i] - b[i]));
    }
    return worst;
  }

  void requestReset()
  {
    reset_pending_ = true;
    if (!reset_client_->service_is_ready()) {
      RCLCPP_WARN_ONCE(get_logger(), "~/reset not available yet, waiting for service");
      return;
    }
    reset_request_sent_ = true;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    reset_client_->async_send_request(
      request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        reset_request_sent_ = false;
        RCLCPP_INFO(
          get_logger(), "reset: success=%d message='%s'", response->success,
          response->message.c_str());
        if (response->success) {
          // Discard samples queued before the reset, then allow fresh physics and
          // TF publications before using an observed joint state as an IK seed.
          latest_joint_state_.reset();
          latest_object_pose_.reset();
          left_contact_.reset();
          right_contact_.reset();
          reset_ready_after_ = get_clock()->now() + rclcpp::Duration::from_seconds(0.1);
          phase_start_time_ = get_clock()->now();
          reset_pending_ = false;
        }
      });
  }

  // Episode boundary (week2.md Stage J): resets this node's own bookkeeping and
  // asks mujoco_bridge to reset the scene, WITHOUT restarting the process -- see
  // phase.hpp's comment on why that used to be the assumption. Idempotent by
  // design: always jumps straight to kHome regardless of current phase_, because
  // the only expected caller is the episode runner, and it only calls this between
  // episodes (after observing a ~/episode_outcome), never mid-episode.
  void onStartEpisode(const std_msgs::msg::Empty::SharedPtr)
  {
    if (diff_ik_source_) {
      diff_ik_source_->beginEpisode();
    }
    logged_target_phase_.reset();
    phase_ = Phase::kHome;
    retry_count_ = 0;
    last_failure_reason_ = ExitReason::kNone;
    phase_names_log_.clear();
    phase_durations_log_.clear();
    target_tcp_x_log_.clear();
    target_tcp_y_log_.clear();
    target_tcp_z_log_.clear();
    ik_position_error_log_.clear();
    joint_tracking_error_log_.clear();
    actual_tcp_position_error_log_.clear();
    phase_start_time_ = get_clock()->now();
    requestReset();
    RCLCPP_INFO(get_logger(), "episode start requested");
  }

  void publishEpisodeOutcome(bool success)
  {
    manipulation_interfaces::msg::EpisodeOutcome outcome;
    outcome.success = success;
    outcome.failure_code = success ? "NONE" : exitReasonName(last_failure_reason_);
    outcome.retries = retry_count_;
    outcome.phase_names = phase_names_log_;
    outcome.phase_durations_s = phase_durations_log_;
    outcome.target_tcp_x_m = target_tcp_x_log_;
    outcome.target_tcp_y_m = target_tcp_y_log_;
    outcome.target_tcp_z_m = target_tcp_z_log_;
    outcome.ik_position_error_m = ik_position_error_log_;
    outcome.joint_tracking_error_rad = joint_tracking_error_log_;
    outcome.actual_tcp_position_error_m = actual_tcp_position_error_log_;
    episode_outcome_pub_->publish(outcome);
    RCLCPP_INFO(
      get_logger(), "episode outcome published: success=%d failure_code=%s retries=%u",
      outcome.success, outcome.failure_code.c_str(), outcome.retries);
    if (latest_object_pose_) {
      const double dx = latest_object_pose_->pose.position.x - params_.place_x_m;
      const double dy = latest_object_pose_->pose.position.y - params_.place_y_m;
      RCLCPP_INFO(
        get_logger(),
        "final placement: box_xy=[%.5f %.5f] verify_xy=[%.5f %.5f] error=%.5fm",
        latest_object_pose_->pose.position.x, latest_object_pose_->pose.position.y,
        params_.place_x_m, params_.place_y_m, std::hypot(dx, dy));
    }
  }

  void onTimer()
  {
    if (phase_start_time_.nanoseconds() == 0) {
      return;  // Idle: no ~/start_episode received yet this run.
    }
    if (phase_ == Phase::kDone || phase_ == Phase::kFailed) {
      return;  // Terminal: stop publishing/deciding until the next ~/start_episode.
    }
    if (reset_pending_) {
      if (!reset_request_sent_) {
        requestReset();
      }
      return;
    }
    if (get_clock()->now() < reset_ready_after_) {
      return;
    }

    const auto arm = extractArmState();
    const auto gripper_width = extractGripperWidth();
    if (!arm || !gripper_width || !latest_object_pose_ || !left_contact_ || !right_contact_) {
      return;  // Not all inputs seen yet -- wait rather than decide on stale data.
    }

    geometry_msgs::msg::TransformStamped hand_tcp_tf;
    try {
      hand_tcp_tf = tf_buffer_->lookupTransform("world", "hand_tcp", tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_ONCE(get_logger(), "world->hand_tcp not available yet: %s", e.what());
      return;
    }

    const ObjectPose object_pose{
      latest_object_pose_->pose.position.x, latest_object_pose_->pose.position.y,
      latest_object_pose_->pose.position.z, latest_object_pose_->pose.orientation.w,
      latest_object_pose_->pose.orientation.x, latest_object_pose_->pose.orientation.y,
      latest_object_pose_->pose.orientation.z};
    if (diff_ik_source_) {
      diff_ik_source_->setSeed(arm->positions);
    }
    JointTarget target;
    try {
      if (diff_ik_source_ && latest_object_pose_->header.frame_id != "world") {
        throw std::invalid_argument("Object pose must be in world frame");
      }
      target = waypoint_source_->jointTargetFor(phase_, object_pose);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "phase %s IK_FAILED: %s", phaseName(phase_), e.what());
      manipulation_interfaces::msg::EpisodeOutcome outcome;
      outcome.success = false;
      outcome.failure_code = "IK_FAILED";
      outcome.retries = retry_count_;
      outcome.phase_names = phase_names_log_;
      outcome.phase_names.push_back(phaseName(phase_));
      outcome.phase_durations_s = phase_durations_log_;
      outcome.phase_durations_s.push_back((get_clock()->now() - phase_start_time_).seconds());
      outcome.target_tcp_x_m = target_tcp_x_log_;
      outcome.target_tcp_y_m = target_tcp_y_log_;
      outcome.target_tcp_z_m = target_tcp_z_log_;
      outcome.ik_position_error_m = ik_position_error_log_;
      outcome.joint_tracking_error_rad = joint_tracking_error_log_;
      outcome.actual_tcp_position_error_m = actual_tcp_position_error_log_;
      const double unavailable = std::numeric_limits<double>::quiet_NaN();
      outcome.target_tcp_x_m.push_back(unavailable);
      outcome.target_tcp_y_m.push_back(unavailable);
      outcome.target_tcp_z_m.push_back(unavailable);
      outcome.ik_position_error_m.push_back(unavailable);
      outcome.joint_tracking_error_rad.push_back(unavailable);
      outcome.actual_tcp_position_error_m.push_back(unavailable);
      episode_outcome_pub_->publish(outcome);
      phase_ = Phase::kFailed;
      return;
    }
    if (diff_ik_source_ && diff_ik_source_->diagnostics() &&
      logged_target_phase_ != phase_)
    {
      const auto & data = *diff_ik_source_->diagnostics();
      const Eigen::Quaterniond target_rotation(data.tcp_target.linear());
      RCLCPP_INFO(
        get_logger(),
        "phase %s target_frame=world tcp_xyz=[%.4f %.4f %.4f] "
        "tcp_qwxyz=[%.4f %.4f %.4f %.4f] "
        "ik_iterations=%zu ik_pos_err=%.6fm ik_rot_err=%.6frad sigma_min=%.6f",
        phaseName(phase_), data.tcp_target.translation().x(),
        data.tcp_target.translation().y(), data.tcp_target.translation().z(),
        target_rotation.w(), target_rotation.x(), target_rotation.y(), target_rotation.z(),
        data.ik.iterations, data.ik.position_error, data.ik.orientation_error,
        data.ik.minimum_singular_value);
      logged_target_phase_ = phase_;
    }
    publishTarget(target);

    FsmInputs in;
    in.phase = phase_;
    in.arm = *arm;
    in.gripper_width_m = *gripper_width;
    in.grasp_signals.gripper_width_m = *gripper_width;
    in.grasp_signals.box_height_m = latest_object_pose_->pose.position.z;
    in.grasp_signals.box_to_tcp_horizontal_m = std::hypot(
      latest_object_pose_->pose.position.x - hand_tcp_tf.transform.translation.x,
      latest_object_pose_->pose.position.y - hand_tcp_tf.transform.translation.y);
    in.grasp_signals.left_finger_contact = *left_contact_;
    in.grasp_signals.right_finger_contact = *right_contact_;
    in.box_x_m = latest_object_pose_->pose.position.x;
    in.box_y_m = latest_object_pose_->pose.position.y;
    in.elapsed_in_phase_s = (get_clock()->now() - phase_start_time_).seconds();
    in.retry_count = retry_count_;

    const FsmDecision decision = step(in, target, params_);
    // NOT "exit_reason == kNone" -- kRecover's retry-to-HOME transition is
    // deliberately tagged kNone (fsm.cpp: it is a redirect, not a failure outcome
    // in its own right), so that check silently ate every retry and left the node
    // stuck in kRecover forever (caught live: week2.md Stage I's first full-episode
    // run never advanced past its first RECOVER). Comparing phases is the actual
    // question this guard needs to answer: did anything change this tick.
    if (decision.next_phase == phase_) {
      return;  // Still in progress; nothing to log or transition.
    }

    double tcp_position_error_m = std::numeric_limits<double>::quiet_NaN();
    double tcp_rotation_error_rad = std::numeric_limits<double>::quiet_NaN();
    if (diff_ik_source_ && diff_ik_source_->diagnostics()) {
      const auto & desired = diff_ik_source_->diagnostics()->tcp_target;
      const Eigen::Vector3d actual(
        hand_tcp_tf.transform.translation.x, hand_tcp_tf.transform.translation.y,
        hand_tcp_tf.transform.translation.z);
      tcp_position_error_m = (desired.translation() - actual).norm();
      const Eigen::Quaterniond actual_rotation(
        hand_tcp_tf.transform.rotation.w, hand_tcp_tf.transform.rotation.x,
        hand_tcp_tf.transform.rotation.y, hand_tcp_tf.transform.rotation.z);
      tcp_rotation_error_rad =
        Eigen::Quaterniond(desired.linear()).angularDistance(actual_rotation);
    }
    RCLCPP_INFO(
      get_logger(),
      "phase %s -> %s: target=[%.3f %.3f %.3f %.3f %.3f %.3f %.3f] "
      "joint_err=%.4frad tcp_pos_err=%.4fm tcp_rot_err=%.4frad "
      "elapsed=%.2fs exit=%s",
      phaseName(phase_), phaseName(decision.next_phase), target.arm_positions[0],
      target.arm_positions[1], target.arm_positions[2], target.arm_positions[3],
      target.arm_positions[4], target.arm_positions[5], target.arm_positions[6],
      maxAbsError(arm->positions, target.arm_positions), tcp_position_error_m,
      tcp_rotation_error_rad, in.elapsed_in_phase_s,
      exitReasonName(decision.exit_reason));

    phase_names_log_.push_back(phaseName(phase_));
    phase_durations_log_.push_back(in.elapsed_in_phase_s);
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    if (diff_ik_source_ && diff_ik_source_->diagnostics()) {
      const auto & data = *diff_ik_source_->diagnostics();
      target_tcp_x_log_.push_back(data.tcp_target.translation().x());
      target_tcp_y_log_.push_back(data.tcp_target.translation().y());
      target_tcp_z_log_.push_back(data.tcp_target.translation().z());
      ik_position_error_log_.push_back(data.ik.position_error);
    } else {
      target_tcp_x_log_.push_back(unavailable);
      target_tcp_y_log_.push_back(unavailable);
      target_tcp_z_log_.push_back(unavailable);
      ik_position_error_log_.push_back(unavailable);
    }
    joint_tracking_error_log_.push_back(maxAbsError(arm->positions, target.arm_positions));
    actual_tcp_position_error_log_.push_back(tcp_position_error_m);
    if (decision.next_phase == Phase::kRecover) {
      // The four-shape design (week2.md 10.3.3): a transition INTO kRecover
      // always carries the real failure ExitReason (kTimeout/kSlipped/...), never
      // kNone -- kNone is reserved for kRecover's own kHome retry redirect, which
      // lands in the branch below instead. Kept separately from that redirect's
      // kNone so a later kFailed can report *why* it kept failing, not just that
      // retries ran out.
      last_failure_reason_ = decision.exit_reason;
    }

    if (decision.is_retry) {
      ++retry_count_;
      if (diff_ik_source_) {
        diff_ik_source_->beginEpisode();
      }
      RCLCPP_WARN(
        get_logger(), "retry %d/%d: recovering to HOME", retry_count_, params_.max_retries);
      requestReset();
    }
    if (decision.next_phase == Phase::kFailed) {
      RCLCPP_ERROR(get_logger(), "episode FAILED after %d retries", retry_count_);
      publishEpisodeOutcome(false);
    }
    if (decision.next_phase == Phase::kDone) {
      RCLCPP_INFO(get_logger(), "episode DONE");
      publishEpisodeOutcome(true);
    }
    phase_ = decision.next_phase;
    logged_target_phase_.reset();
    phase_start_time_ = get_clock()->now();
  }

  KeyframeWaypointSource keyframe_source_;
  std::unique_ptr<DiffIkWaypointSource> diff_ik_source_;
  WaypointSource * waypoint_source_ = nullptr;
  std::optional<Phase> logged_target_phase_;
  FsmParams params_;
  Phase phase_ = Phase::kHome;
  // Zero (RCL_ROS_TIME's default-constructed value) doubles as "no episode has
  // been started yet" -- onTimer()'s idle check and onStartEpisode() both rely on
  // this sentinel, see phase.hpp's comment on why there is no dedicated Phase for
  // it instead.
  rclcpp::Time phase_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reset_ready_after_{0, 0, RCL_ROS_TIME};
  bool reset_pending_ = false;
  bool reset_request_sent_ = false;
  int retry_count_ = 0;
  // Per-episode bookkeeping for ~/episode_outcome (week2.md Stage J), reset in
  // onStartEpisode(): which phases this episode visited, in order, and how long
  // elapsed_in_phase_s was when it left each one.
  std::vector<std::string> phase_names_log_;
  std::vector<double> phase_durations_log_;
  std::vector<double> target_tcp_x_log_;
  std::vector<double> target_tcp_y_log_;
  std::vector<double> target_tcp_z_log_;
  std::vector<double> ik_position_error_log_;
  std::vector<double> joint_tracking_error_log_;
  std::vector<double> actual_tcp_position_error_log_;
  // The most recent real failure classification (kTimeout/kSlipped/...), i.e. the
  // exit_reason of the most recent transition INTO kRecover -- kept separately
  // from ExitReason::kRetryLimitExceeded (kRecover's own exit reason when it gives
  // up) so a failed episode's outcome reports what actually kept failing, not
  // just that retries ran out.
  ExitReason last_failure_reason_ = ExitReason::kNone;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<control_msgs::msg::GripperCommand>::SharedPtr gripper_command_pub_;
  rclcpp::Publisher<manipulation_interfaces::msg::EpisodeOutcome>::SharedPtr
    episode_outcome_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_contact_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_contact_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_episode_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_object_pose_;
  std::optional<bool> left_contact_;
  std::optional<bool> right_contact_;
};

}  // namespace task_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<task_executor::TaskExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
