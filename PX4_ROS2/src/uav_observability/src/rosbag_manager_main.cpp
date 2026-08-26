#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "uav_observability/rosbag_manager_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_observability::RosbagManagerNode>();
  // Single-threaded: rosbag2_cpp::Writer is not thread-safe (R24).
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
