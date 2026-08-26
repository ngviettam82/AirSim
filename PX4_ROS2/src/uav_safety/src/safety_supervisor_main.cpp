#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "uav_safety/safety_supervisor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_safety::SafetySupervisorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
