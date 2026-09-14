#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mujoco_bridge/frame_math.hpp"
#include "mujoco_bridge/mujoco_dl.hpp"
#include "mujoco_bridge/state_ops.hpp"

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
// The MJCF's only keyframe. Looked up by name at startup rather than hardcoded as
// index 0, so adding a second keyframe upstream cannot silently repoint the reset.
constexpr const char * kResetKeyframeName = "home";

constexpr const char * kHandBodyName = "hand";
constexpr const char * kTcpFrameName = "hand_tcp";
constexpr double kHandToTcpZ = 0.1034;

class MujocoBridgeNode : public rclcpp::Node
{
public:
  MujocoBridgeNode()
  : Node("mujoco_bridge"), api_(loadMujocoApi())
  {
    const std::string model_path =
      ament_index_cpp::get_package_share_directory("robot_description") +
      "/mujoco/franka_emika_panda/panda.xml";

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
    RCLCPP_INFO(
      get_logger(), "Loaded %s (nq=%d, timestep=%.4fs)",
      model_path.c_str(), model_->nq, timestep_s);

    buildJointIndex();
    buildFrameIndex();
    buildActuatorIndex();

    // Physics runs at 1/timestep; the state publishers run slower. Decimating by an
    // integer number of steps keeps every published sample aligned with an exact
    // physics step (no interpolation, no drift between sim_time and step count).
    joint_state_decimation_ =
      decimationFor("joint_state_rate_hz", "/joint_states", timestep_s);
    tf_decimation_ = decimationFor("tf_rate_hz", "/tf", timestep_s);
    if (joint_state_decimation_ != tf_decimation_) {
      RCLCPP_WARN(
        get_logger(), "joint_state_rate_hz and tf_rate_hz decimate differently (%d vs %d "
        "steps); /joint_states and /tf samples will not line up 1:1",
        joint_state_decimation_, tf_decimation_);
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
    reset_keyframe_id_ = api_.name2id(model_, mjOBJ_KEY, kResetKeyframeName);
    if (reset_keyframe_id_ < 0) {
      RCLCPP_WARN(
        get_logger(), "model has no keyframe `%s`; ~/reset will fail", kResetKeyframeName);
    }
    // Private name (~/reset -> /mujoco_bridge/reset): unlike /joint_states and /clock
    // this is not a system-wide singleton -- a second sim instance in the same graph
    // must be resettable independently.
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      std::bind(
        &MujocoBridgeNode::onReset, this, std::placeholders::_1, std::placeholders::_2));

    // Private name, same reasoning as ~/reset (10.4): this is a capability of *this*
    // sim instance, not a system-wide singleton, and the name must not collide with
    // whatever a real robot driver calls its command topic.
    joint_command_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_command", rclcpp::QoS(10),
      std::bind(&MujocoBridgeNode::onJointCommand, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timestep_s)),
      std::bind(&MujocoBridgeNode::onTimer, this));
  }

  ~MujocoBridgeNode() override
  {
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
    if (api_.name2id(model_, mjOBJ_BODY, kHandBodyName) >= 0) {
      const mjtNum tcp_pos[3] = {0.0, 0.0, kHandToTcpZ};
      const mjtNum tcp_quat[4] = {1.0, 0.0, 0.0, 0.0};  // identity, (w, x, y, z)
      static_transforms_.push_back(
        makeTransform(kHandBodyName, kTcpFrameName, tcp_pos, tcp_quat));
    } else {
      RCLCPP_WARN(
        get_logger(), "no `%s` body in model, not synthesizing %s",
        kHandBodyName, kTcpFrameName);
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
  int decimationFor(const std::string & param, const char * topic, double timestep_s)
  {
    const double rate_hz = declare_parameter(param, 100.0);
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

  // Snaps the simulation back to the `home` keyframe. Trigger (no request fields) is
  // the right service type here precisely because there is nothing to parameterise:
  // if a caller could pass a keyframe name or an arbitrary qpos, this would need a
  // custom .srv -- and that is the point at which "reset" stops being one operation.
  //
  // Thread safety comes for free from the default single-threaded executor: this
  // callback and onTimer() are both in the node's default (mutually exclusive)
  // callback group, so a reset can never land halfway through an mj_step. Moving
  // either one to a separate callback group, or switching to a MultiThreadedExecutor,
  // would make this a data race on mjData with no compiler or runtime complaint.
  void onReset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
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
      response->success = false;
      response->message =
        std::string("model has no keyframe `") + kResetKeyframeName + "`";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }

    response->success = true;
    response->message = std::string("reset to keyframe `") + kResetKeyframeName + "`";
    RCLCPP_INFO(get_logger(), "%s (sim time preserved at %.3fs)", response->message.c_str(),
      data_->time);
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

    // The two fingers share one control DOF (the "split" tendon), so a command that
    // names finger_joint1 and/or finger_joint2 is buffered here and applied once,
    // instead of writing ctrl[gripper_actuator_id_] twice from two different targets.
    double gripper_target = std::numeric_limits<double>::quiet_NaN();

    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
      const std::string & name = msg->joint_names[i];
      const double target = point.positions[i];

      if (gripper_joint_names_.count(name) > 0) {
        if (!std::isnan(gripper_target) && gripper_target != target) {
          RCLCPP_WARN(
            get_logger(),
            "~/joint_command: finger joints commanded to different positions "
            "(%.4f vs %.4f) but they share one actuator; using %.4f",
            gripper_target, target, target);
        }
        gripper_target = target;
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

    if (!std::isnan(gripper_target)) {
      if (gripper_actuator_id_ < 0) {
        RCLCPP_WARN(get_logger(), "~/joint_command: no gripper actuator in this model, ignoring");
      } else {
        // Convert from the finger joint's own units (meters of opening) to the
        // actuator's remapped ctrl range, using the ratio derived in
        // buildActuatorIndex() instead of the hardcoded 255/0.04 from the MJCF
        // comment.
        data_->ctrl[gripper_actuator_id_] = gripper_target * gripper_ctrl_scale_;
      }
    }
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
    }
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
    joint_state_msg_.header.stamp = simTime();
    for (size_t i = 0; i < joints_.size(); ++i) {
      joint_state_msg_.position[i] = data_->qpos[joints_[i].qpos_adr];
      joint_state_msg_.velocity[i] = data_->qvel[joints_[i].dof_adr];
      // qfrc_actuator is the generalized force actually applied by the actuators,
      // which is the closest analogue to what a real joint torque sensor reports.
      joint_state_msg_.effort[i] = data_->qfrc_actuator[joints_[i].dof_adr];
    }
    joint_state_pub_->publish(joint_state_msg_);
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
  int reset_keyframe_id_ = -1;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_sub_;
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
  uint64_t step_count_ = 0;
  std::chrono::steady_clock::time_point rtf_window_wall_start_ = std::chrono::steady_clock::now();
  double rtf_window_sim_start_ = 0.0;
};

}  // namespace mujoco_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mujoco_bridge::MujocoBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
