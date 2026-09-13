#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "mujoco_bridge/mujoco_dl.hpp"

namespace mujoco_bridge
{

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

    // Physics runs at 1/timestep; the state publishers run slower. Decimating by an
    // integer number of steps keeps every published sample aligned with an exact
    // physics step (no interpolation, no drift between sim_time and step count).
    const double joint_state_rate_hz = declare_parameter("joint_state_rate_hz", 100.0);
    joint_state_decimation_ = std::max(
      1, static_cast<int>(std::lround(1.0 / (joint_state_rate_hz * timestep_s))));
    RCLCPP_INFO(
      get_logger(), "/joint_states every %d steps (%.1f Hz requested, %.1f Hz actual)",
      joint_state_decimation_, joint_state_rate_hz,
      1.0 / (joint_state_decimation_ * timestep_s));

    // ClockQoS: best-effort, keep-last depth 1, volatile. Deliberately NOT reliable:
    // a /clock sample that needs retransmitting is already stale by the time it
    // arrives, and at 500 Hz a reliable queue would just build backpressure.
    // Subscribers only ever want the newest sample.
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    // Global name, not ~/joint_states: robot_state_publisher and RViz both default
    // to subscribing /joint_states, and there is only ever one state source here.
    joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10));

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
  std::vector<JointEntry> joints_;
  sensor_msgs::msg::JointState joint_state_msg_;
  int joint_state_decimation_ = 1;
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
