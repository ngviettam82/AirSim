#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "uav_control_authority/control_authority_manager_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_control_authority::ControlAuthorityManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
