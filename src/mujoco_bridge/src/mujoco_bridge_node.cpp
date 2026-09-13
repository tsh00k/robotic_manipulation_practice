#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "mujoco_bridge/mujoco_dl.hpp"

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

    // Physics runs at 1/timestep; the state publishers run slower. Decimating by an
    // integer number of steps keeps every published sample aligned with an exact
    // physics step (no interpolation, no drift between sim_time and step count).
    joint_state_decimation_ =
      decimationFor("joint_state_rate_hz", "/joint_states", timestep_s);
    tf_decimation_ = decimationFor("tf_rate_hz", "/tf", timestep_s);

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

  void onTimer()
  {
    api_.step(model_, data_);
    ++step_count_;

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
      const mjtNum * child_pos = data_->xpos + 3 * frame.body_id;
      const mjtNum * child_quat = data_->xquat + 4 * frame.body_id;
      const mjtNum * parent_pos = data_->xpos + 3 * frame.parent_id;
      const mjtNum * parent_quat = data_->xquat + 4 * frame.parent_id;

      mjtNum parent_quat_inv[4];
      api_.negQuat(parent_quat_inv, parent_quat);

      const mjtNum delta[3] = {
        child_pos[0] - parent_pos[0],
        child_pos[1] - parent_pos[1],
        child_pos[2] - parent_pos[2],
      };
      mjtNum rel_pos[3];
      api_.rotVecQuat(rel_pos, delta, parent_quat_inv);
      mjtNum rel_quat[4];
      api_.mulQuat(rel_quat, parent_quat_inv, child_quat);

      // Only the numbers change; frame_id/child_frame_id were filled once at startup.
      frame.msg.header.stamp = stamp;
      frame.msg.transform.translation.x = rel_pos[0];
      frame.msg.transform.translation.y = rel_pos[1];
      frame.msg.transform.translation.z = rel_pos[2];
      frame.msg.transform.rotation.w = rel_quat[0];
      frame.msg.transform.rotation.x = rel_quat[1];
      frame.msg.transform.rotation.y = rel_quat[2];
      frame.msg.transform.rotation.z = rel_quat[3];
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
  std::vector<JointEntry> joints_;
  sensor_msgs::msg::JointState joint_state_msg_;
  std::vector<geometry_msgs::msg::TransformStamped> static_transforms_;
  std::vector<DynamicFrame> dynamic_frames_;
  std::vector<geometry_msgs::msg::TransformStamped> tf_batch_;
  int joint_state_decimation_ = 1;
  int tf_decimation_ = 1;
  uint64_t step_count_ = 0;
};

}  // namespace mujoco_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mujoco_bridge::MujocoBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
