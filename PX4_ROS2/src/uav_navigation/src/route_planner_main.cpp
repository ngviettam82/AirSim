#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "uav_navigation/route_planner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Held in a shared_ptr: add_node keeps only a weak reference (plan P6 section 2b).
  auto node = std::make_shared<uav_navigation::RoutePlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
