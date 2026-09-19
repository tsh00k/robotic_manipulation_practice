#include <control_msgs/msg/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "task_executor/fsm.hpp"
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
// decision-making already lives in fsm.cpp (unit tested) and
// keyframe_waypoint_source.hpp (a static lookup table).
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

    // NOT the visual place_marker geom's (0.5, 0.3) -- see
    // keyframe_waypoint_source.hpp's docstring on why kPlace's actually-reached xy
    // falls short of that by several cm, and why closing that gap is deferred to
    // week3's IK rather than hand-tuned further this week.
    params_.place_x_m = declare_parameter("verify.place_x_m", 0.43);
    params_.place_y_m = declare_parameter("verify.place_y_m", 0.31);
    params_.place_region_radius_m = declare_parameter("verify.place_region_radius_m", 0.08);

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
    if (!reset_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "~/reset not available yet, retry will re-enter HOME unreset");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    reset_client_->async_send_request(
      request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        RCLCPP_INFO(
          get_logger(), "reset for retry: success=%d message='%s'", response->success,
          response->message.c_str());
      });
  }

  void onTimer()
  {
    if (phase_ == Phase::kDone || phase_ == Phase::kFailed) {
      return;  // Terminal: stop publishing/deciding, node stays up for inspection.
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

    if (phase_start_time_.nanoseconds() == 0) {
      phase_start_time_ = get_clock()->now();
      requestReset();  // First tick of the whole episode: start from a known state.
    }

    const ObjectPose object_pose{
      latest_object_pose_->pose.position.x, latest_object_pose_->pose.position.y,
      latest_object_pose_->pose.position.z, latest_object_pose_->pose.orientation.w,
      latest_object_pose_->pose.orientation.x, latest_object_pose_->pose.orientation.y,
      latest_object_pose_->pose.orientation.z};
    const JointTarget target = waypoint_source_.jointTargetFor(phase_, object_pose);
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

    RCLCPP_INFO(
      get_logger(),
      "phase %s -> %s: target=[%.3f %.3f %.3f %.3f %.3f %.3f %.3f] pos_err=%.4frad "
      "elapsed=%.2fs exit=%s",
      phaseName(phase_), phaseName(decision.next_phase), target.arm_positions[0],
      target.arm_positions[1], target.arm_positions[2], target.arm_positions[3],
      target.arm_positions[4], target.arm_positions[5], target.arm_positions[6],
      maxAbsError(arm->positions, target.arm_positions), in.elapsed_in_phase_s,
      exitReasonName(decision.exit_reason));

    if (decision.is_retry) {
      ++retry_count_;
      RCLCPP_WARN(
        get_logger(), "retry %d/%d: recovering to HOME", retry_count_, params_.max_retries);
      requestReset();
    }
    if (decision.next_phase == Phase::kFailed) {
      RCLCPP_ERROR(get_logger(), "episode FAILED after %d retries", retry_count_);
    }
    if (decision.next_phase == Phase::kDone) {
      RCLCPP_INFO(get_logger(), "episode DONE");
    }
    phase_ = decision.next_phase;
    phase_start_time_ = get_clock()->now();
  }

  KeyframeWaypointSource waypoint_source_;
  FsmParams params_;
  Phase phase_ = Phase::kHome;
  rclcpp::Time phase_start_time_{0, 0, RCL_ROS_TIME};
  int retry_count_ = 0;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<control_msgs::msg::GripperCommand>::SharedPtr gripper_command_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_contact_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_contact_sub_;
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
