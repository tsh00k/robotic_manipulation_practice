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

#include <mujoco/mujoco.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/msg/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <manipulation_interfaces/msg/bridge_observation.hpp>
#include <manipulation_interfaces/srv/reset_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "mujoco_bridge/debug_viewer.hpp"
#include "mujoco_bridge/frame_math.hpp"
#include "mujoco_bridge/grasp_criteria.hpp"
#include "mujoco_bridge/grasp_state.hpp"
#include "mujoco_bridge/mujoco_dl.hpp"
#include "mujoco_bridge/rgbd_camera.hpp"
#include "mujoco_bridge/state_ops.hpp"
#include "scene_config.hpp"
#include "scene_ops.hpp"

namespace mujoco_bridge
{

// hand_tcp does not exist in the MJCF. It is a URDF-side frame that franka_description
// defines in end_effectors/common/franka_hand.xacro as an empty link hung off `hand`
// by hand_tcp_joint, with default tcp_xyz="0 0 0.1034", tcp_rpy="0 0 0". Since this
// node is the sim-side ground truth publisher, it synthesizes that frame so downstream
// grasp/planning code has exactly one definition of the TCP.
//
// This is a copy of an upstream number: if franka_description ever changes tcp_xyz,
// nothing here will notice. Cross-check with
//   xacro $(ros2 pkg prefix franka_description)/share/franka_description/robots/fer/fer.urdf.xacro
// and see docs/architecture.md section 1. Note the -45 deg wrist rotation is NOT part
// of this transform -- it lives in hand_joint (link8 -> hand), which the MJCF folds
// into the `hand` body's own quat.
// Default model + its reset keyframe name, as a pair: the keyframe to reset to is a
// property of *which model is loaded*, not a global constant, because different
// models can't share a keyframe name across an <include> chain. Concretely,
// panda.xml's own "home" keyframe (qpos length 9) is pulled into pick_place_scene.xml
// unchanged by its <include>, so pick_place_scene.xml cannot also define a "home" --
// MuJoCo rejects that at compile time as a duplicate name (see that file's comment
// and week2.md Stage G). Both are therefore declared ROS parameters with defaults
// that match each other.
//
// The two ways to misconfigure this pair fail very differently, and only one of them
// is safe:
//   - model_path -> panda.xml, reset_keyframe_name left at "pick_place_home": that
//     name does not exist in panda.xml at all, so name2id returns -1 and ~/reset
//     fails loudly (see the WARN below and onReset()'s error response).
//   - model_path -> pick_place_scene.xml, reset_keyframe_name left/set to "home":
//     that name DOES exist (panda.xml's own keyframe, pulled in unchanged), just with
//     the wrong (shorter) qpos length. mj_resetDataKeyframe does not validate length
//     against the current model; it silently zero-pads the missing tail and reports
//     success -- verified by actually doing this (week2.md 8.8). There is no
//     "fail loudly" here at all.
// The lesson is not "always fails safe" but "know which direction you're switching in".
constexpr const char * kDefaultModelRelativePath =
  "/mujoco/franka_emika_panda/pick_place_scene.xml";
// Loaded instead when scene.enabled is true: the same scene plus the bin. A separate
// file rather than a hidden bin in the default one, because a hidden bin was measured
// to change the rendered image (see that file's header).
constexpr const char * kBinModelRelativePath =
  "/mujoco/franka_emika_panda/pick_place_bin_scene.xml";
constexpr const char * kDefaultResetKeyframeName = "pick_place_home";

constexpr const char * kHandBodyName = "hand";
constexpr const char * kTcpFrameName = "hand_tcp";
constexpr double kHandToTcpZ = 0.1034;
// Ground-truth object body. Not every model has one (panda.xml alone does not);
// missing it only disables the oracle topic, it is never an error (see
// buildFrameIndex-style guarded lookup below).
constexpr const char * kObjectBodyName = "box";
// Same "hardcode the MJCF's own name" discipline as kHandBodyName/kObjectBodyName --
// this is the single-gripper assumption already flagged in docs/architecture.md
// section 4 (kHandBodyName's entry), not a new one.
constexpr const char * kLeftFingerBodyName = "left_finger";
constexpr const char * kRightFingerBodyName = "right_finger";
// These names intentionally match the project-owned MJCF. They are not camera
// calibration numbers: the lookup below turns them into MuJoCo IDs and verifies
// that the RGB-D camera is attached to the expected body.
constexpr const char * kCameraName = "workcell_rgbd";
constexpr const char * kCameraBodyName = "camera_link";
constexpr const char * kOpticalFrameName = "camera_optical_frame";

uint64_t makeBridgeSession()
{
  std::random_device entropy;
  return (uint64_t{entropy()} << 32) | uint64_t{entropy()};
}

class MujocoBridgeNode : public rclcpp::Node
{
public:
  MujocoBridgeNode()
  : Node("mujoco_bridge"), api_(loadMujocoApi())
  {
    // Decided first: it selects the default model, and a pose parameter given while the
    // scene is off is an error worth reporting before anything is loaded.
    const bool scene_enabled = declare_parameter("scene.enabled", false);
    std::vector<std::string> given;
    for (const auto & entry : get_node_parameters_interface()->get_parameter_overrides()) {
      given.push_back(entry.first);
    }
    rejectPoseOverridesWhenDisabled(scene_enabled, given);

    const std::string default_model_path =
      ament_index_cpp::get_package_share_directory("robot_description") +
      (scene_enabled ? kBinModelRelativePath : kDefaultModelRelativePath);
    const std::string model_path = declare_parameter("model_path", default_model_path);
    const std::string reset_keyframe_name =
      declare_parameter("reset_keyframe_name", std::string(kDefaultResetKeyframeName));

    char error[1024] = {0};
    model_ = api_.loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!model_) {
      throw std::runtime_error("mj_loadXML failed for " + model_path + ": " + error);
    }
    data_ = api_.makeData(model_);
    if (!data_) {
      throw std::runtime_error("mj_makeData failed");
    }

    const double timestep_s = model_->opt.timestep;
    // nq != nv the moment any body has a free/ball joint (a free joint alone is 7
    // qpos vs 6 qvel -- 3 translation are 1:1 but the 4-component quaternion has one
    // more entry than its 3-component angular velocity). Printing both, not just nq,
    // is what actually caught the qpos/qvel length mismatch class of bug in Stage F's
    // free_body.xml fixture; panda.xml alone never exercises it (nq==nv==9 there).
    RCLCPP_INFO(
      get_logger(), "Loaded %s (nq=%d, nv=%d, timestep=%.4fs)",
      model_path.c_str(), model_->nq, model_->nv, timestep_s);

    // Before anything is derived from the model: the box start pose and the bin pose
    // are model state, and the indices built below must see the configured layout.
    configureScene(scene_enabled);
    buildJointIndex();
    buildFrameIndex();
    buildActuatorIndex();
    resolveObjectOracle();
    resolveGripperFingers();

    // Node parameters, not code constants: the numbers below are placeholders until
    // the week2.md Stage H three-scenario measurement (empty grasp / normal grasp /
    // induced slip) fills them in, and even afterwards they are scene-specific
    // (box_width_m in particular), not physical constants.
    grasp_criteria_.box_width_m = declare_parameter("grasp.box_width_m", 0.04);
    grasp_criteria_.width_epsilon_m = declare_parameter("grasp.width_epsilon_m", 0.01);
    grasp_criteria_.lift_height_threshold_m =
      declare_parameter("grasp.lift_height_threshold_m", 0.26);
    grasp_criteria_.region_radius_m = declare_parameter("grasp.region_radius_m", 0.05);

    // Physics runs at 1/timestep; the state publishers run slower. Decimating by an
    // integer number of steps keeps every published sample aligned with an exact
    // physics step (no interpolation, no drift between sim_time and step count).
    joint_state_decimation_ =
      decimationFor("joint_state_rate_hz", "/joint_states", timestep_s);
    tf_decimation_ = decimationFor("tf_rate_hz", "/tf", timestep_s);
    camera_decimation_ = decimationFor("camera_rate_hz", "RGB-D camera", timestep_s, 10.0);
    if (joint_state_decimation_ != tf_decimation_) {
      RCLCPP_WARN(
        get_logger(), "joint_state_rate_hz and tf_rate_hz decimate differently (%d vs %d "
        "steps); /joint_states and /tf samples will not line up 1:1",
        joint_state_decimation_, tf_decimation_);
    }

    // Debug-only visualization, directly on this node's own mjData -- no topic, no
    // decimated republish. Off by default: disabled, this costs nothing (no
    // glfwInit, no window). See debug_viewer.hpp for why render() itself must stay
    // decimated rather than running every physics step.
    debug_viewer_decimation_ =
      decimationFor("debug_viewer_rate_hz", "debug viewer", timestep_s, 30.0);
    if (declare_parameter("enable_debug_viewer", false)) {
      viewer_ = std::make_unique<DebugViewer>(api_, model_);
      RCLCPP_INFO(get_logger(), "Debug viewer enabled (GLFW window)");
    }

    if (declare_parameter("enable_rgbd_camera", false)) {
      if (camera_decimation_ % tf_decimation_ != 0) {
        throw std::runtime_error(
                "camera_rate_hz must produce frames on episode_observation steps "
                "(camera decimation must be a multiple of tf decimation)");
      }
      const int camera_id = api_.name2id(model_, mjOBJ_CAMERA, kCameraName);
      const int camera_body_id = api_.name2id(model_, mjOBJ_BODY, kCameraBodyName);
      if (camera_id < 0 || camera_body_id < 0 ||
        model_->cam_bodyid[camera_id] != camera_body_id)
      {
        throw std::runtime_error(
                "enable_rgbd_camera requires workcell_rgbd attached to camera_link");
      }
      const int width = model_->vis.global.offwidth;
      const int height = model_->vis.global.offheight;
      camera_ = std::make_unique<RgbdCamera>(api_, model_, camera_id, width, height);
      const double fovy_rad = model_->cam_fovy[camera_id] * mjPI / 180.0;
      const double focal = height / (2.0 * std::tan(fovy_rad / 2.0));
      camera_info_.width = width;
      camera_info_.height = height;
      camera_info_.distortion_model = "plumb_bob";
      camera_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
      camera_info_.k = {focal, 0.0, (width - 1) / 2.0,
        0.0, focal, (height - 1) / 2.0,
        0.0, 0.0, 1.0};
      camera_info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      camera_info_.p = {focal, 0.0, (width - 1) / 2.0, 0.0,
        0.0, focal, (height - 1) / 2.0, 0.0,
        0.0, 0.0, 1.0, 0.0};
      camera_info_.header.frame_id = kOpticalFrameName;
      rgb_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "~/camera/color/image_raw",
        rclcpp::SensorDataQoS());
      depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "~/camera/depth/image_raw",
        rclcpp::SensorDataQoS());
      camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "~/camera/color/camera_info", rclcpp::SensorDataQoS());
      depth_camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "~/camera/depth/camera_info", rclcpp::SensorDataQoS());
      RCLCPP_INFO(
        get_logger(), "RGB-D %dx%d at %.1f Hz, fovy=%.1f deg, fx=fy=%.3f px, depth=32FC1 metres",
        width, height, 1.0 / (camera_decimation_ * timestep_s),
        model_->cam_fovy[camera_id], focal);
    }

    // ClockQoS: best-effort, keep-last depth 1, volatile. Deliberately NOT reliable:
    // a /clock sample that needs retransmitting is already stale by the time it
    // arrives, and at 500 Hz a reliable queue would just build backpressure.
    // Subscribers only ever want the newest sample.
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    // Global name, not ~/joint_states: robot_state_publisher and RViz both default
    // to subscribing /joint_states, and there is only ever one state source here.
    joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10));

    // Two broadcasters because /tf and /tf_static are two topics with different QoS.
    // /tf_static is transient_local (latched): a subscriber that joins late still
    // receives the one message we send below, and tf2 treats those transforms as
    // valid at any time. /tf is volatile and must be re-sent every cycle.
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    // Stamp is t=0 here; tf2 ignores the stamp on /tf_static entries and treats them
    // as valid for all time, but leaving it unset would still be misleading.
    for (auto & tf : static_transforms_) {
      tf.header.stamp = simTime();
    }
    static_tf_broadcaster_->sendTransform(static_transforms_);

    // Resolved once here, not inside the callback: a missing keyframe is a property
    // of the model, so it should be visible at startup rather than on the first
    // reset call. The service is still created (and reports the failure) so callers
    // get an explicit "no such keyframe" rather than a missing service.
    reset_keyframe_name_ = reset_keyframe_name;
    reset_keyframe_id_ = api_.name2id(model_, mjOBJ_KEY, reset_keyframe_name_.c_str());
    if (reset_keyframe_id_ < 0) {
      RCLCPP_WARN(
        get_logger(), "model has no keyframe `%s`; ~/reset will fail",
        reset_keyframe_name_.c_str());
    }
    // Private name (~/reset -> /mujoco_bridge/reset): unlike /joint_states and /clock
    // this is not a system-wide singleton -- a second sim instance in the same graph
    // must be resettable independently.
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      std::bind(
        &MujocoBridgeNode::onReset, this, std::placeholders::_1, std::placeholders::_2));
    reset_with_generation_service_ = create_service<manipulation_interfaces::srv::ResetScene>(
      "~/reset_with_generation",
      std::bind(
        &MujocoBridgeNode::onResetWithGeneration, this,
        std::placeholders::_1, std::placeholders::_2));
    observation_pub_ = create_publisher<manipulation_interfaces::msg::BridgeObservation>(
      "~/episode_observation", rclcpp::QoS(10));

    // Private name, same reasoning as ~/reset (10.4): this is a capability of *this*
    // sim instance, not a system-wide singleton, and the name must not collide with
    // whatever a real robot driver calls its command topic.
    joint_command_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_command", rclcpp::QoS(10),
      std::bind(&MujocoBridgeNode::onJointCommand, this, std::placeholders::_1));

    // Split from ~/joint_command (Stage H): control_msgs/GripperCommand is the
    // shape a real ros2_control gripper action server expects (position +
    // max_effort), so downstream code that talks to this topic today needs no
    // change when this bridge is swapped for a real driver. Still a plain topic,
    // not the actual GripperCommand *action* -- see onGripperCommand() for why.
    gripper_command_sub_ = create_subscription<control_msgs::msg::GripperCommand>(
      "~/gripper_command", rclcpp::QoS(10),
      std::bind(&MujocoBridgeNode::onGripperCommand, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timestep_s)),
      std::bind(&MujocoBridgeNode::onTimer, this));
  }

  ~MujocoBridgeNode() override
  {
    // Freed before model_/data_ (in reverse of the order they're needed) so no GL
    // teardown code runs against an already-deleted mjModel.
    viewer_.reset();
    camera_.reset();
    if (data_) {
      api_.deleteData(data_);
    }
    if (model_) {
      api_.deleteModel(model_);
    }
  }

private:
  // One entry per single-DoF (hinge/slide) joint in the MJCF, in model order.
  struct JointEntry
  {
    std::string name;
    int qpos_adr;  // index into mjData::qpos
    int dof_adr;   // index into mjData::qvel / qfrc_*
  };

  // One entry per body whose pose relative to its parent can change.
  struct DynamicFrame
  {
    int body_id;
    int parent_id;
    geometry_msgs::msg::TransformStamped msg;  // frame ids prefilled, numbers rewritten
  };

  // Derive the ROS joint list from the model itself rather than hardcoding
  // {joint1..7, finger_joint1/2}: adding a gripper or swapping the arm model then
  // needs no code change. Free/ball joints (e.g. a manipulation object dropped into
  // the scene) occupy 7/4 qpos entries and have no single joint angle, so they are
  // skipped -- they belong in TF (Stage C), not in JointState.
  void buildJointIndex()
  {
    for (int i = 0; i < model_->njnt; ++i) {
      const int type = model_->jnt_type[i];
      if (type != mjJNT_HINGE && type != mjJNT_SLIDE) {
        continue;
      }
      const char * name = api_.id2name(model_, mjOBJ_JOINT, i);
      if (!name) {
        RCLCPP_WARN(get_logger(), "joint id %d has no name, skipping", i);
        continue;
      }
      joints_.push_back({name, model_->jnt_qposadr[i], model_->jnt_dofadr[i]});
    }
    if (joints_.empty()) {
      throw std::runtime_error("no hinge/slide joints found in model");
    }

    // The name vector never changes, so fill it once and only rewrite the numbers
    // on each publish.
    joint_state_msg_.name.reserve(joints_.size());
    for (const auto & j : joints_) {
      joint_state_msg_.name.push_back(j.name);
    }
    joint_state_msg_.position.resize(joints_.size());
    joint_state_msg_.velocity.resize(joints_.size());
    joint_state_msg_.effort.resize(joints_.size());

    std::string names;
    for (const auto & j : joints_) {
      names += (names.empty() ? "" : ", ") + j.name;
    }
    RCLCPP_INFO(get_logger(), "%zu actuated joints: %s", joints_.size(), names.c_str());
  }

  // Splits the MJCF body tree into the transforms that can never change and the ones
  // that must be re-sent every cycle. The test is purely structural: a body with zero
  // joints is welded to its parent, so mjModel::body_pos/body_quat *is* its
  // parent-relative transform, forever. A body with at least one joint moves relative
  // to its parent and needs xpos/xquat every cycle.
  //
  // Deriving this from the model means a manipulation object dropped into the scene
  // (free joint, parent = world) automatically shows up as a dynamic world -> object
  // frame with no code change, and it keeps us honest about what the MJCF actually
  // contains: there is no `link8` and the base body is `link0`, not `base_link`
  // (the URDF-side names -- see docs/architecture.md section 1).
  void buildFrameIndex()
  {
    // Body 0 is MuJoCo's implicit "world" body, which is also the TF tree root, so
    // start at 1 and let the parent lookup produce "world" for us.
    for (int i = 1; i < model_->nbody; ++i) {
      const char * name = api_.id2name(model_, mjOBJ_BODY, i);
      const int parent = model_->body_parentid[i];
      const char * parent_name = api_.id2name(model_, mjOBJ_BODY, parent);
      if (!name || !parent_name) {
        // An unnamed body cannot be addressed in TF at all. Its *children* are still
        // published, which would silently reparent them -- warn loudly rather than
        // emit a broken tree.
        RCLCPP_WARN(
          get_logger(), "body id %d or its parent %d has no name, skipping frame", i, parent);
        continue;
      }

      if (model_->body_jntnum[i] == 0) {
        static_transforms_.push_back(
          makeTransform(
            parent_name, name, model_->body_pos + 3 * i, model_->body_quat + 4 * i));
      } else {
        DynamicFrame frame;
        frame.body_id = i;
        frame.parent_id = parent;
        frame.msg.header.frame_id = parent_name;
        frame.msg.child_frame_id = name;
        dynamic_frames_.push_back(frame);
      }
    }

    // Synthesize the TCP frame (see kHandToTcpZ above), but only if the model really
    // has a `hand` -- panda_nohand.xml does not.
    hand_body_id_ = api_.name2id(model_, mjOBJ_BODY, kHandBodyName);
    if (hand_body_id_ >= 0) {
      const mjtNum tcp_pos[3] = {0.0, 0.0, kHandToTcpZ};
      const mjtNum tcp_quat[4] = {1.0, 0.0, 0.0, 0.0};  // identity, (w, x, y, z)
      static_transforms_.push_back(
        makeTransform(kHandBodyName, kTcpFrameName, tcp_pos, tcp_quat));
    } else {
      RCLCPP_WARN(
        get_logger(), "no `%s` body in model, not synthesizing %s",
        kHandBodyName, kTcpFrameName);
    }

    if (api_.name2id(model_, mjOBJ_BODY, kCameraBodyName) >= 0) {
      const mjtNum optical_pos[3] = {0.0, 0.0, 0.0};
      const mjtNum optical_quat[4] = {0.0, 1.0, 0.0, 0.0};
      static_transforms_.push_back(
        makeTransform(kCameraBodyName, kOpticalFrameName, optical_pos, optical_quat));
    }

    RCLCPP_INFO(
      get_logger(), "TF: %zu static, %zu dynamic frames",
      static_transforms_.size(), dynamic_frames_.size());
  }

  // Maps joint name -> actuator id, so ~/joint_command can look up "which ctrl
  // index does joint4 drive" instead of assuming ctrl[i] and qpos[i] line up. They
  // do for this model's 7 arm joints (actuatorN drives jointN), but mjData::ctrl is
  // indexed by actuator id, not joint id -- nu=8, not nq=9, because the two fingers
  // share a single tendon-driven actuator. Deriving this from actuator_trntype
  // instead of hardcoding "actuator1..7" keeps the same "read it from the model"
  // discipline as buildJointIndex/buildFrameIndex, and doesn't break if the arm
  // actuators are ever reordered or renamed upstream.
  //
  // The gripper is handled separately below because it has no per-joint actuator at
  // all: actuator8 drives the "split" tendon, which couples finger_joint1 and
  // finger_joint2 with a fixed 0.5/0.5 split (see docs/architecture.md and
  // week1.md 4.3.4). There is exactly one control input for both fingers.
  void buildActuatorIndex()
  {
    for (int i = 0; i < model_->nu; ++i) {
      if (model_->actuator_trntype[i] == mjTRN_JOINT) {
        const int joint_id = model_->actuator_trnid[2 * i];
        const char * joint_name = api_.id2name(model_, mjOBJ_JOINT, joint_id);
        if (joint_name) {
          actuator_by_joint_[joint_name] = i;
        }
        continue;
      }
      if (model_->actuator_trntype[i] != mjTRN_TENDON) {
        continue;
      }

      // A tendon-driven actuator. Walk the tendon's wrap list to find which joints
      // it couples, rather than assuming this is "the gripper" by name -- the only
      // fact we rely on is the transmission type.
      const int tendon_id = model_->actuator_trnid[2 * i];
      const int wrap_begin = model_->tendon_adr[tendon_id];
      const int wrap_end = wrap_begin + model_->tendon_num[tendon_id];
      int representative_joint_id = -1;
      for (int w = wrap_begin; w < wrap_end; ++w) {
        if (model_->wrap_type[w] != mjWRAP_JOINT) {
          continue;
        }
        const int joint_id = model_->wrap_objid[w];
        const char * joint_name = api_.id2name(model_, mjOBJ_JOINT, joint_id);
        if (joint_name) {
          gripper_joint_names_.insert(joint_name);
          representative_joint_id = joint_id;
        }
      }
      if (representative_joint_id < 0) {
        continue;
      }

      // ctrlrange for this actuator has been remapped by the upstream MJCF (here
      // 0..255) to whatever the tendon's *own* actuators used before the remap --
      // it does not have to match the joint's own range (0..0.04 m). Rather than
      // hardcode that 255/0.04 ratio, read both ranges from the model and take
      // their ratio, so a command expressed in the joint's native units (finger
      // opening in meters) converts to the actuator's ctrl units.
      gripper_actuator_id_ = i;
      gripper_ctrl_scale_ =
        model_->actuator_ctrlrange[2 * i + 1] / model_->jnt_range[2 * representative_joint_id + 1];
    }

    RCLCPP_INFO(
      get_logger(), "actuators: %zu arm joint(s) mapped, gripper actuator %s",
      actuator_by_joint_.size(), gripper_actuator_id_ >= 0 ? "found" : "NOT found");
  }

  // Optional start layout (scene.*), off by default.
  //
  // Disabled: nothing happens. The model is the default scene file, loaded untouched
  // (no bin in it at all), so the simulator is the legacy one. Enabled: the model is the
  // bin scene; the pose parameters are declared, validated against extents read from
  // that model, and applied to it (scene_ops.hpp). A pose parameter given while the
  // scene is off was already rejected in the constructor.
  //
  // These parameters exist on this node only. The poses are simulator ground truth;
  // perception and the executor must get box/bin poses from the camera, so launch
  // passes scene.* to the bridge alone (docs/adr/015).
  void configureScene(bool enabled)
  {
    if (!enabled) {
      RCLCPP_INFO(get_logger(), "scene.enabled=false: legacy layout, no bin");
      return;
    }

    const SceneGeometry geometry = readSceneGeometry(api_, model_);
    // z defaults to the support height for the requested orientation, so omitting it
    // never penetrates; an explicit z is validated like every other value.
    const auto declarePose = [&](
      const std::string & prefix, double default_x, double default_y,
      const Eigen::Vector3d & lo, const Eigen::Vector3d & hi) {
        PoseRequest pose;
        pose.x = declare_parameter(prefix + ".x", default_x);
        pose.y = declare_parameter(prefix + ".y", default_y);
        pose.roll = declare_parameter(prefix + ".roll", 0.0);
        pose.pitch = declare_parameter(prefix + ".pitch", 0.0);
        pose.yaw = declare_parameter(prefix + ".yaw", 0.0);
        pose.z = declare_parameter(
          prefix + ".z",
          autoSupportZ(
            rotationFromRpy(pose.roll, pose.pitch, pose.yaw), lo, hi, geometry.table_top_z));
        return pose;
      };
    SceneRequest request;
    request.box = declarePose(
      "scene.box", kDefaultBoxX, kDefaultBoxY, -geometry.box_half_extents,
      geometry.box_half_extents);
    request.bin = declarePose(
      "scene.bin", kDefaultBinX, kDefaultBinY, geometry.bin_min, geometry.bin_max);

    const ScenePoses poses = resolveScene(request, geometry);
    applyScene(api_, model_, data_, poses);
    // The bin's true pose, for the oracle source's place target only (Week 4.1 Stage 8):
    // same category as ~/ground_truth/object_pose, never for perception. The bin is static
    // and reset restores the same layout, so it is published once, latched.
    bin_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/ground_truth/bin_pose", rclcpp::QoS(1).transient_local());
    geometry_msgs::msg::PoseStamped bin_pose;
    bin_pose.header.frame_id = "world";
    bin_pose.pose.position.x = poses.bin.translation().x();
    bin_pose.pose.position.y = poses.bin.translation().y();
    bin_pose.pose.position.z = poses.bin.translation().z();
    const Eigen::Quaterniond bin_rotation(poses.bin.linear());
    bin_pose.pose.orientation.w = bin_rotation.w();
    bin_pose.pose.orientation.x = bin_rotation.x();
    bin_pose.pose.orientation.y = bin_rotation.y();
    bin_pose.pose.orientation.z = bin_rotation.z();
    bin_pose_pub_->publish(bin_pose);
    RCLCPP_INFO(
      get_logger(),
      "scene.enabled=true: box xyz=[%.4f %.4f %.4f] rpy=[%.3f %.3f %.3f], "
      "bin xyz=[%.4f %.4f %.4f] rpy=[%.3f %.3f %.3f]",
      request.box.x, request.box.y, request.box.z, request.box.roll, request.box.pitch,
      request.box.yaw, request.bin.x, request.bin.y, request.bin.z, request.bin.roll,
      request.bin.pitch, request.bin.yaw);
  }

  // Looks up the ground-truth object body once at startup, same discipline as
  // reset_keyframe_id_: a missing object is a property of the model, so it should be
  // visible in the startup log rather than discovered the first time something reads
  // an empty ~/ground_truth/object_pose topic. The topic itself is only created when
  // the body exists -- there is nothing meaningful to publish for panda.xml alone.
  //
  // This is deliberately the *only* place downstream code may get the object's true
  // pose (see docs/architecture.md "oracle 必须走独立接口" requirement): the object's
  // world->box TF frame already exists for free via buildFrameIndex's structural
  // body_jntnum test, but that is incidental (every free body gets a TF frame,
  // ground-truth object or not) and TF is not gated behind "this is the oracle" the
  // way a dedicated topic is. Perception nodes are meant to publish an *estimated*
  // object pose on a different topic later (Week 4); nothing should quietly start
  // reading TF instead of this topic to get the real answer early.
  void resolveObjectOracle()
  {
    object_body_id_ = api_.name2id(model_, mjOBJ_BODY, kObjectBodyName);
    if (object_body_id_ < 0) {
      RCLCPP_INFO(
        get_logger(), "no `%s` body in model; ~/ground_truth/object_pose disabled",
        kObjectBodyName);
      return;
    }
    object_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/ground_truth/object_pose", rclcpp::QoS(
        10));
    RCLCPP_INFO(
      get_logger(), "ground-truth object `%s` found; publishing ~/ground_truth/object_pose",
      kObjectBodyName);
  }

  // Finger-to-box contact has no real-robot analogue this clean (no real gripper
  // reports "which finger is touching the object" directly) -- it belongs in the
  // same ground-truth-only category as resolveObjectOracle() above, for the same
  // "must not silently become a substitute for perception" reason. Guarded the same
  // way: missing fingers only disables these two topics, never an error.
  void resolveGripperFingers()
  {
    left_finger_body_id_ = api_.name2id(model_, mjOBJ_BODY, kLeftFingerBodyName);
    right_finger_body_id_ = api_.name2id(model_, mjOBJ_BODY, kRightFingerBodyName);
    if (left_finger_body_id_ < 0 || right_finger_body_id_ < 0) {
      RCLCPP_INFO(
        get_logger(), "no `%s`/`%s` body pair in model; grasp contact signals disabled",
        kLeftFingerBodyName, kRightFingerBodyName);
      return;
    }
    // Each finger body has exactly one slide joint (panda.xml's <default class=
    // "finger">). Reading its qpos_adr off the body -- rather than hardcoding the
    // joint names "finger_joint1"/"finger_joint2" -- keeps this in the same
    // "read it from the model" discipline as buildFrameIndex's body_jntnum test,
    // even though the body names themselves are still hardcoded above.
    left_finger_qpos_adr_ = model_->jnt_qposadr[model_->body_jntadr[left_finger_body_id_]];
    right_finger_qpos_adr_ = model_->jnt_qposadr[model_->body_jntadr[right_finger_body_id_]];

    left_finger_contact_pub_ = create_publisher<std_msgs::msg::Bool>(
      "~/ground_truth/left_finger_contact", rclcpp::QoS(10));
    right_finger_contact_pub_ = create_publisher<std_msgs::msg::Bool>(
      "~/ground_truth/right_finger_contact", rclcpp::QoS(10));
    RCLCPP_INFO(
      get_logger(), "gripper fingers `%s`/`%s` found; publishing ~/ground_truth/*_finger_contact",
      kLeftFingerBodyName, kRightFingerBodyName);
  }

  // The order hazard is on the *input* side: MuJoCo packs quaternions into a raw
  // mjtNum[4] as (w, x, y, z), so quat[0] is w, not x. geometry_msgs has named
  // fields, so assigning them by name (rather than memcpy'ing four doubles into the
  // message, which would silently reinterpret w as x) is what makes this safe.
  static geometry_msgs::msg::TransformStamped makeTransform(
    const std::string & parent, const std::string & child,
    const mjtNum * pos, const mjtNum * quat)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.transform.translation.x = pos[0];
    tf.transform.translation.y = pos[1];
    tf.transform.translation.z = pos[2];
    tf.transform.rotation.w = quat[0];
    tf.transform.rotation.x = quat[1];
    tf.transform.rotation.y = quat[2];
    tf.transform.rotation.z = quat[3];
    return tf;
  }

  // Rounds a requested publish rate down to a whole number of physics steps.
  int decimationFor(
    const std::string & param, const char * topic, double timestep_s,
    double default_rate_hz = 100.0)
  {
    const double rate_hz = declare_parameter(param, default_rate_hz);
    const int decimation =
      std::max(1, static_cast<int>(std::lround(1.0 / (rate_hz * timestep_s))));
    RCLCPP_INFO(
      get_logger(), "%s every %d steps (%.1f Hz requested, %.1f Hz actual)",
      topic, decimation, rate_hz, 1.0 / (decimation * timestep_s));
    return decimation;
  }

  // Sim time is authoritative here: it is steps * timestep, maintained by MuJoCo in
  // mjData::time. We convert that to a ROS stamp instead of calling
  // get_clock()->now() -- this node *is* the /clock source, so reading its own clock
  // would either return wall time (use_sim_time=false) or the value it just published
  // one step ago (use_sim_time=true). Neither is what a consumer expects.
  rclcpp::Time simTime() const
  {
    // llround, not a plain cast: mjData::time is a running double sum, so
    // steps*timestep lands a hair below the exact value and truncation would turn
    // 4.858s into 4.857999999s.
    return rclcpp::Time(std::llround(data_->time * 1e9), RCL_ROS_TIME);
  }

  // Snaps the simulation back to the configured keyframe. The legacy Trigger
  // service stays available; reset_with_generation returns the generation needed
  // to reject delayed observations after this reset.
  //
  // Thread safety comes for free from the default single-threaded executor: this
  // callback and onTimer() are both in the node's default (mutually exclusive)
  // callback group, so a reset can never land halfway through an mj_step. Moving
  // either one to a separate callback group, or switching to a MultiThreadedExecutor,
  // would make this a data race on mjData with no compiler or runtime complaint.
  bool resetScene(std::string & message)
  {
    // mj_resetDataKeyframe restores qpos, qvel, act, ctrl and mocap from the keyframe
    // -- ctrl included. That matters: the `home` key carries its own
    // ctrl="0 0 0 -1.57079 0 1.57079 -0.7853 255", so the position servos get targets
    // consistent with the new qpos. Resetting qpos alone would leave the old targets
    // in place and the servos would immediately drag the arm back out of home pose.
    // Sim time is kept monotonic and derived quantities (xpos/xquat, qfrc_actuator)
    // are refreshed inside resetToKeyframe (state_ops.hpp) -- see that function for
    // why both matter.
    if (!resetToKeyframe(api_, model_, data_, reset_keyframe_id_)) {
      message = "model has no keyframe `" + reset_keyframe_name_ + "`";
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
      return false;
    }

    attachment_state_ = manipulation_interfaces::msg::BridgeObservation::ATTACHMENT_NOT_ATTACHED;
    ++generation_;
    message = "reset to keyframe `" + reset_keyframe_name_ + "`";
    RCLCPP_INFO(
      get_logger(), "%s (generation=%" PRIu64 ", sim time preserved at %.3fs)",
      message.c_str(), generation_, data_->time);
    return true;
  }

  void onReset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = resetScene(response->message);
  }

  void onResetWithGeneration(
    const std::shared_ptr<manipulation_interfaces::srv::ResetScene::Request>/*request*/,
    std::shared_ptr<manipulation_interfaces::srv::ResetScene::Response> response)
  {
    response->success = resetScene(response->message);
    response->bridge_session = bridge_session_;
    response->generation = generation_;
  }

  // Writes the first trajectory point's positions into mjData::ctrl. Only the first
  // point is used -- there is no trajectory *execution* here (no interpolation
  // between points, no timing), just "set today's target and let the position
  // servos in the MJCF (gainprm/biasprm, see 4.3.4) do the rest". A real trajectory
  // follower belongs upstream of this bridge, e.g. as a ros2_control controller or
  // a node that resamples a trajectory into a fast stream of single-point commands.
  //
  // Same callback-group reasoning as onReset (10.8.3): this runs on the default
  // single-threaded executor alongside onTimer(), so a command can never be applied
  // to ctrl mid-mj_step.
  void onJointCommand(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    if (msg->points.empty()) {
      RCLCPP_WARN(get_logger(), "~/joint_command: trajectory has no points, ignoring");
      return;
    }
    if (msg->points.size() > 1) {
      // Not a bug -- this node was never a trajectory follower -- but a message with
      // N points and only the first one taking effect is exactly the kind of thing
      // that should be loud once rather than silent forever, since it produces
      // correct-looking behavior (something moves) for the wrong reason.
      RCLCPP_WARN_ONCE(
        get_logger(),
        "~/joint_command: message has %zu points; only points[0] is applied "
        "(no interpolation, no timing -- see Stage E notes)",
        msg->points.size());
    }
    const auto & point = msg->points.front();
    if (point.positions.size() != msg->joint_names.size()) {
      RCLCPP_WARN(
        get_logger(), "~/joint_command: %zu joint_names but %zu positions, ignoring",
        msg->joint_names.size(), point.positions.size());
      return;
    }

    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
      const std::string & name = msg->joint_names[i];
      const double target = point.positions[i];

      // Stage H split the gripper out to its own topic (see onGripperCommand()) so
      // this bridge's command shape matches a real ros2_control setup, where the
      // arm's JointTrajectoryController and the gripper's GripperActionController
      // are two separate interfaces. Warn once rather than silently ignoring, since
      // this used to work here and a caller that has not migrated yet would
      // otherwise see "nothing happened" with no explanation.
      if (gripper_joint_names_.count(name) > 0) {
        RCLCPP_WARN_ONCE(
          get_logger(),
          "~/joint_command: `%s` is a gripper joint; finger joints must be commanded "
          "via ~/gripper_command now (control_msgs/GripperCommand), ignoring here",
          name.c_str());
        continue;
      }

      const auto it = actuator_by_joint_.find(name);
      if (it == actuator_by_joint_.end()) {
        RCLCPP_WARN(get_logger(), "~/joint_command: unknown joint `%s`, ignoring", name.c_str());
        continue;
      }
      // it->second is an actuator id, not a joint id -- ctrl is indexed by the
      // former. Using the joint's own qpos_adr/dof_adr here would be the Stage E
      // landmine from 4.3.4: it happens to work for the 7 arm joints because
      // actuatorN drives jointN, and would silently misfire for anything else.
      data_->ctrl[it->second] = target;
    }
  }

  // control_msgs/GripperCommand rather than the action of the same name: this is
  // still a topic-based interface (see the plan doc week2.md Stage H), matching
  // ~/joint_command's own "set today's target, no execution semantics" shape rather
  // than promising the goal/feedback/cancel lifecycle a real action implies. Moving
  // to the actual action later is a bigger change than swapping message types --
  // it needs an action server loop here, not just a different subscription.
  //
  // `position` is the *total* finger-to-finger opening in meters (0 = closed, up to
  // 2x each finger joint's own 0.04m range = 0.08m open), matching the real Franka
  // gripper's convention -- not the per-finger displacement ~/joint_command used to
  // accept. Halving it here, rather than changing gripper_ctrl_scale_'s definition,
  // keeps that scale factor meaning exactly one thing: ctrl units per meter of a
  // single finger's own travel, which is also what buildActuatorIndex() derives it
  // as and what gripperWidth() (grasp_state.hpp) sums back out of qpos.
  void onGripperCommand(const control_msgs::msg::GripperCommand::SharedPtr msg)
  {
    if (gripper_actuator_id_ < 0) {
      RCLCPP_WARN(get_logger(), "~/gripper_command: no gripper actuator in this model, ignoring");
      return;
    }
    if (msg->max_effort != 0.0) {
      // Not rejected outright: max_effort=0 is also control_msgs' own "no limit"
      // sentinel in some conventions, so treating *every* nonzero value as user
      // intent and warning (rather than silently accepting all values) is the
      // closest thing to "explicit" a plain topic (no response, unlike a service or
      // action goal) allows. See week2.md Stage H for why this can't be honored:
      // the MJCF gripper actuator is a position servo (gainprm/biasprm PD gains),
      // with no separate force/torque control channel this bridge exposes.
      RCLCPP_WARN_ONCE(
        get_logger(),
        "~/gripper_command: max_effort is ignored -- the gripper actuator is a "
        "position servo with no exposed force limit, commanding one here does nothing");
    }
    data_->ctrl[gripper_actuator_id_] = (msg->position / 2.0) * gripper_ctrl_scale_;
  }

  // Logs realtime factor (sim seconds advanced / wall seconds elapsed) once a second.
  // Physics steps and publish rates are both defined in sim time; once episodes run
  // long, an RTF that has drifted from ~1 silently changes what a wall-clock timeout
  // (or a human watching RViz) actually means, with no other symptom.
  void logRtfIfDue()
  {
    const auto wall_now = std::chrono::steady_clock::now();
    const double wall_elapsed_s =
      std::chrono::duration<double>(wall_now - rtf_window_wall_start_).count();
    if (wall_elapsed_s < 1.0) {
      return;
    }
    const double sim_elapsed_s = data_->time - rtf_window_sim_start_;
    RCLCPP_INFO(get_logger(), "RTF: %.2f", sim_elapsed_s / wall_elapsed_s);
    rtf_window_wall_start_ = wall_now;
    rtf_window_sim_start_ = data_->time;
  }

  void onTimer()
  {
    api_.step(model_, data_);
    ++step_count_;
    logRtfIfDue();

    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = simTime();
    clock_pub_->publish(clock_msg);

    if (step_count_ % joint_state_decimation_ == 0) {
      publishJointState();
    }
    if (step_count_ % tf_decimation_ == 0) {
      publishTransforms();
      publishObjectPose();
      publishGripperContact();
      publishEpisodeObservation();
    }
    if (camera_ && step_count_ % camera_decimation_ == 0) {
      publishCamera();
    }

    if (viewer_) {
      viewer_->pollEvents();
      if (viewer_->shouldClose()) {
        // Closing the debug window is not a stop control: drop it and keep
        // stepping/publishing exactly as if it had never been enabled.
        viewer_.reset();
      } else if (step_count_ % debug_viewer_decimation_ == 0) {
        viewer_->render(data_);
      }
    }
  }

  // Absolute (world-frame) pose, unlike publishTransforms() which composes
  // parent-relative transforms -- the object's parent *is* world (it has a free
  // joint directly under worldbody), so xpos/xquat are already what
  // ~/ground_truth/object_pose promises. Sharing tf_decimation_ (rather than a
  // separate rate) is deliberate: this topic and the world->box TF frame are meant to
  // be cross-checked against each other (docs/architecture.md), which only means
  // something if they describe the same physics step.
  void publishObjectPose()
  {
    if (object_body_id_ < 0) {
      return;
    }
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = simTime();
    msg.header.frame_id = "world";
    msg.pose.position.x = data_->xpos[3 * object_body_id_ + 0];
    msg.pose.position.y = data_->xpos[3 * object_body_id_ + 1];
    msg.pose.position.z = data_->xpos[3 * object_body_id_ + 2];
    msg.pose.orientation.w = data_->xquat[4 * object_body_id_ + 0];
    msg.pose.orientation.x = data_->xquat[4 * object_body_id_ + 1];
    msg.pose.orientation.y = data_->xquat[4 * object_body_id_ + 2];
    msg.pose.orientation.z = data_->xquat[4 * object_body_id_ + 3];
    object_pose_pub_->publish(msg);
  }

  // Publishes the two per-finger contact booleans and, as a side effect, logs a
  // full grasp-outcome line whenever classifyGrasp()'s answer changes (Stage H).
  // The log is deliberately not itself a topic yet: task_executor (Stage I) is the
  // first real consumer, and until that node exists we do not actually know whether
  // it wants classifyGrasp's answer as a whole, just the raw signals, or something
  // else entirely -- publishing a stable topic contract now would be guessing.
  void publishGripperContact()
  {
    if (left_finger_body_id_ < 0 || object_body_id_ < 0) {
      return;
    }
    const bool left_contact = bodiesInContact(model_, data_, left_finger_body_id_, object_body_id_);
    const bool right_contact =
      bodiesInContact(model_, data_, right_finger_body_id_, object_body_id_);

    std_msgs::msg::Bool left_msg;
    left_msg.data = left_contact;
    left_finger_contact_pub_->publish(left_msg);
    std_msgs::msg::Bool right_msg;
    right_msg.data = right_contact;
    right_finger_contact_pub_->publish(right_msg);

    if (hand_body_id_ < 0) {
      return;
    }
    // hand_tcp's world position = hand's world pose composed with the fixed local
    // offset (kHandToTcpZ) -- the inverse of what relativePose() (frame_math.hpp)
    // computes, so done inline here rather than as a third frame_math function for
    // a single 3-vector rotate-and-add.
    mjtNum tcp_offset_world[3];
    const mjtNum tcp_local[3] = {0.0, 0.0, kHandToTcpZ};
    api_.rotVecQuat(tcp_offset_world, tcp_local, data_->xquat + 4 * hand_body_id_);
    const double tcp_x = data_->xpos[3 * hand_body_id_ + 0] + tcp_offset_world[0];
    const double tcp_y = data_->xpos[3 * hand_body_id_ + 1] + tcp_offset_world[1];

    const GraspSignals signals{
      gripperWidth(data_, left_finger_qpos_adr_, right_finger_qpos_adr_),
      data_->xpos[3 * object_body_id_ + 2],
      std::hypot(
        data_->xpos[3 * object_body_id_ + 0] - tcp_x, data_->xpos[3 * object_body_id_ + 1] - tcp_y),
      left_contact,
      right_contact,
    };
    const GraspOutcome outcome = classifyGrasp(signals, grasp_criteria_);
    if (!last_logged_grasp_outcome_ || *last_logged_grasp_outcome_ != outcome) {
      RCLCPP_INFO(
        get_logger(),
        "grasp outcome -> %s (width=%.4fm box_z=%.4fm box_to_tcp=%.4fm L=%d R=%d)",
        graspOutcomeName(outcome), signals.gripper_width_m, signals.box_height_m,
        signals.box_to_tcp_horizontal_m, left_contact, right_contact);
      last_logged_grasp_outcome_ = outcome;
    }
  }

  static const char * graspOutcomeName(GraspOutcome outcome)
  {
    switch (outcome) {
      case GraspOutcome::kSuccess: return "SUCCESS";
      case GraspOutcome::kNoObject: return "NO_OBJECT";
      case GraspOutcome::kGraspEmpty: return "GRASP_EMPTY";
      case GraspOutcome::kSlip: return "SLIP";
      case GraspOutcome::kTimeout: return "TIMEOUT";
      case GraspOutcome::kPlaceMissed: return "PLACE_MISSED";
      case GraspOutcome::kUnexpectedContact: return "UNEXPECTED_CONTACT";
    }
    return "UNKNOWN";
  }

  // mjData::xpos/xquat are absolute (relative to world). TF needs each transform
  // relative to the *parent* frame, so compose T_parent^-1 * T_child. Publishing the
  // absolute pose under a non-world frame_id would look right only for bodies whose
  // parent happens to be world.
  void publishTransforms()
  {
    const rclcpp::Time stamp = simTime();
    tf_batch_.clear();
    tf_batch_.reserve(dynamic_frames_.size());

    for (auto & frame : dynamic_frames_) {
      Pose child;
      std::copy_n(data_->xpos + 3 * frame.body_id, 3, child.pos);
      std::copy_n(data_->xquat + 4 * frame.body_id, 4, child.quat);
      Pose parent;
      std::copy_n(data_->xpos + 3 * frame.parent_id, 3, parent.pos);
      std::copy_n(data_->xquat + 4 * frame.parent_id, 4, parent.quat);

      const Pose rel = relativePose(api_, child, parent);

      // Only the numbers change; frame_id/child_frame_id were filled once at startup.
      frame.msg.header.stamp = stamp;
      frame.msg.transform.translation.x = rel.pos[0];
      frame.msg.transform.translation.y = rel.pos[1];
      frame.msg.transform.translation.z = rel.pos[2];
      frame.msg.transform.rotation.w = rel.quat[0];
      frame.msg.transform.rotation.x = rel.quat[1];
      frame.msg.transform.rotation.y = rel.quat[2];
      frame.msg.transform.rotation.z = rel.quat[3];
      tf_batch_.push_back(frame.msg);
    }

    // One message carrying all transforms, not one message per frame: /tf is a
    // vector-of-transforms topic precisely so a publisher can send a consistent
    // snapshot in a single sample.
    tf_broadcaster_->sendTransform(tf_batch_);
  }

  void publishJointState()
  {
    fillJointState();
    joint_state_pub_->publish(joint_state_msg_);
  }

  void publishCamera()
  {
    // Capture first, then stamp every message from the same post-mj_step sim
    // time. capture() fills CPU buffers only; this method adapts those buffers to
    // standard ROS Image/CameraInfo messages.
    camera_->capture(data_);
    const auto stamp = simTime();
    sensor_msgs::msg::Image rgb;
    rgb.header.stamp = stamp;
    rgb.header.frame_id = kOpticalFrameName;
    rgb.height = camera_info_.height;
    rgb.width = camera_info_.width;
    rgb.encoding = "rgb8";
    rgb.is_bigendian = false;
    // rgb8 is tightly packed RGB: three uint8 channels per pixel and no row
    // padding. The vector is already in ROS top-to-bottom row order.
    rgb.step = rgb.width * 3;
    rgb.data = camera_->rgb();
    rgb_pub_->publish(rgb);

    sensor_msgs::msg::Image depth;
    depth.header = rgb.header;
    depth.height = rgb.height;
    depth.width = rgb.width;
    depth.encoding = "32FC1";
    depth.is_bigendian = false;
    // 32FC1 is one float32 z-depth per pixel, in metres. Copy the byte view of
    // the float vector into the ROS message without changing NaN values.
    depth.step = depth.width * sizeof(float);
    const auto & values = camera_->depth();
    const auto * bytes = reinterpret_cast<const uint8_t *>(values.data());
    depth.data.assign(bytes, bytes + values.size() * sizeof(float));
    depth_pub_->publish(depth);

    camera_info_.header.stamp = stamp;
    camera_info_pub_->publish(camera_info_);
    depth_camera_info_pub_->publish(camera_info_);
  }

  void fillJointState()
  {
    joint_state_msg_.header.stamp = simTime();
    for (size_t i = 0; i < joints_.size(); ++i) {
      joint_state_msg_.position[i] = data_->qpos[joints_[i].qpos_adr];
      joint_state_msg_.velocity[i] = data_->qvel[joints_[i].dof_adr];
      // qfrc_actuator is the generalized force actually applied by the actuators,
      // which is the closest analogue to what a real joint torque sensor reports.
      joint_state_msg_.effort[i] = data_->qfrc_actuator[joints_[i].dof_adr];
    }
  }

  void publishEpisodeObservation()
  {
    if (object_body_id_ < 0 || hand_body_id_ < 0 || left_finger_body_id_ < 0) {
      return;
    }
    fillJointState();
    manipulation_interfaces::msg::BridgeObservation msg;
    msg.bridge_session = bridge_session_;
    msg.generation = generation_;
    msg.sample_sequence = step_count_;
    msg.joint_state = joint_state_msg_;
    msg.object_pose.header.stamp = simTime();
    msg.object_pose.header.frame_id = "world";
    msg.object_pose.pose.position.x = data_->xpos[3 * object_body_id_ + 0];
    msg.object_pose.pose.position.y = data_->xpos[3 * object_body_id_ + 1];
    msg.object_pose.pose.position.z = data_->xpos[3 * object_body_id_ + 2];
    msg.object_pose.pose.orientation.w = data_->xquat[4 * object_body_id_ + 0];
    msg.object_pose.pose.orientation.x = data_->xquat[4 * object_body_id_ + 1];
    msg.object_pose.pose.orientation.y = data_->xquat[4 * object_body_id_ + 2];
    msg.object_pose.pose.orientation.z = data_->xquat[4 * object_body_id_ + 3];
    msg.left_finger_contact = bodiesInContact(
      model_, data_, left_finger_body_id_, object_body_id_);
    msg.right_finger_contact = bodiesInContact(
      model_, data_, right_finger_body_id_, object_body_id_);

    mjtNum tcp_offset_world[3];
    const mjtNum tcp_local[3] = {0.0, 0.0, kHandToTcpZ};
    api_.rotVecQuat(tcp_offset_world, tcp_local, data_->xquat + 4 * hand_body_id_);
    auto & tcp = msg.world_to_hand_tcp;
    tcp.header.stamp = simTime();
    tcp.header.frame_id = "world";
    tcp.child_frame_id = kTcpFrameName;
    tcp.transform.translation.x = data_->xpos[3 * hand_body_id_ + 0] + tcp_offset_world[0];
    tcp.transform.translation.y = data_->xpos[3 * hand_body_id_ + 1] + tcp_offset_world[1];
    tcp.transform.translation.z = data_->xpos[3 * hand_body_id_ + 2] + tcp_offset_world[2];
    tcp.transform.rotation.w = data_->xquat[4 * hand_body_id_ + 0];
    tcp.transform.rotation.x = data_->xquat[4 * hand_body_id_ + 1];
    tcp.transform.rotation.y = data_->xquat[4 * hand_body_id_ + 2];
    tcp.transform.rotation.z = data_->xquat[4 * hand_body_id_ + 3];

    const GraspSignals signals{
      gripperWidth(data_, left_finger_qpos_adr_, right_finger_qpos_adr_),
      msg.object_pose.pose.position.z,
      std::hypot(
        msg.object_pose.pose.position.x - tcp.transform.translation.x,
        msg.object_pose.pose.position.y - tcp.transform.translation.y),
      msg.left_finger_contact,
      msg.right_finger_contact,
    };
    const auto previous_attachment_state = attachment_state_;
    const double open_threshold = grasp_criteria_.box_width_m + grasp_criteria_.width_epsilon_m;
    if (attachment_state_ == manipulation_interfaces::msg::BridgeObservation::ATTACHMENT_ATTACHED) {
      // Contact is noisy during transport. Only an intentional opening can end
      // the bridge-owned attachment lifecycle.
      if (signals.gripper_width_m > open_threshold) {
        attachment_state_ = manipulation_interfaces::msg::BridgeObservation::ATTACHMENT_RELEASED;
      }
    } else if (confirmsAttachment(signals, grasp_criteria_)) {
      attachment_state_ = manipulation_interfaces::msg::BridgeObservation::ATTACHMENT_ATTACHED;
    }
    if (attachment_state_ != previous_attachment_state) {
      RCLCPP_INFO(
        get_logger(),
        "attachment %u -> %u (width=%.4fm box_z=%.4fm box_to_tcp=%.4fm L=%d R=%d)",
        static_cast<unsigned>(previous_attachment_state), static_cast<unsigned>(attachment_state_),
        signals.gripper_width_m, signals.box_height_m, signals.box_to_tcp_horizontal_m,
        signals.left_finger_contact, signals.right_finger_contact);
    }
    msg.attachment_state = attachment_state_;
    observation_pub_->publish(msg);
  }

  MujocoApi & api_;
  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<manipulation_interfaces::srv::ResetScene>::SharedPtr
    reset_with_generation_service_;
  rclcpp::Publisher<manipulation_interfaces::msg::BridgeObservation>::SharedPtr observation_pub_;
  uint64_t generation_ = 0;
  const uint64_t bridge_session_ = makeBridgeSession();
  int reset_keyframe_id_ = -1;
  std::string reset_keyframe_name_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_sub_;
  rclcpp::Subscription<control_msgs::msg::GripperCommand>::SharedPtr gripper_command_sub_;
  std::unordered_map<std::string, int> actuator_by_joint_;  // joint name -> actuator id
  std::unordered_set<std::string> gripper_joint_names_;     // finger_joint1, finger_joint2
  int gripper_actuator_id_ = -1;
  double gripper_ctrl_scale_ = 1.0;  // ctrl units per meter of finger opening
  std::vector<JointEntry> joints_;
  sensor_msgs::msg::JointState joint_state_msg_;
  std::vector<geometry_msgs::msg::TransformStamped> static_transforms_;
  std::vector<DynamicFrame> dynamic_frames_;
  std::vector<geometry_msgs::msg::TransformStamped> tf_batch_;
  int joint_state_decimation_ = 1;
  int tf_decimation_ = 1;
  int camera_decimation_ = 1;
  std::unique_ptr<RgbdCamera> camera_;
  sensor_msgs::msg::CameraInfo camera_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_pub_;
  int object_body_id_ = -1;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr bin_pose_pub_;
  int hand_body_id_ = -1;
  int left_finger_body_id_ = -1;
  int right_finger_body_id_ = -1;
  int left_finger_qpos_adr_ = -1;
  int right_finger_qpos_adr_ = -1;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_finger_contact_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_finger_contact_pub_;
  GraspCriteria grasp_criteria_{};
  manipulation_interfaces::msg::BridgeObservation::_attachment_state_type attachment_state_ =
    manipulation_interfaces::msg::BridgeObservation::ATTACHMENT_NOT_ATTACHED;
  std::optional<GraspOutcome> last_logged_grasp_outcome_;
  uint64_t step_count_ = 0;
  std::chrono::steady_clock::time_point rtf_window_wall_start_ = std::chrono::steady_clock::now();
  double rtf_window_sim_start_ = 0.0;
  int debug_viewer_decimation_ = 1;
  std::unique_ptr<DebugViewer> viewer_;
};

}  // namespace mujoco_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<mujoco_bridge::MujocoBridgeNode> node;
  try {
    node = std::make_shared<mujoco_bridge::MujocoBridgeNode>();
  } catch (const std::exception & e) {
    // An invalid configuration (scene.*, model path) is an expected way for startup to
    // fail: say why and exit non-zero, not terminate() with a core dump.
    RCLCPP_FATAL(rclcpp::get_logger("mujoco_bridge"), "startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
