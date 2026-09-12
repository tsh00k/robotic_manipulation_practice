#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <stdexcept>
#include <string>

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

    const auto timestep_s = model_->opt.timestep;
    const auto period = std::chrono::duration<double>(timestep_s);
    RCLCPP_INFO(
      get_logger(), "Loaded %s (nq=%d, timestep=%.4fs)",
      model_path.c_str(), model_->nq, timestep_s);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
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
  void onTimer()
  {
    api_.step(model_, data_);
    ++step_count_;
    if (step_count_ % 500 == 0) {
      RCLCPP_INFO(
        get_logger(), "step=%lu sim_time=%.3fs", step_count_, data_->time);
    }
  }

  MujocoApi & api_;
  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;
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
