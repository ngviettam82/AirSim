#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "uav_observability/diagnostics_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_observability::DiagnosticsNode>();
  // R24: single thread -- the two StalenessBoard instances are not
  // internally synchronised.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
