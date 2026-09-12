#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mujoco_bridge");
  RCLCPP_INFO(node->get_logger(), "mujoco_bridge placeholder node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
