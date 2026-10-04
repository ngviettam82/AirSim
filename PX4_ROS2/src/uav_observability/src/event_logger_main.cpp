#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "uav_observability/event_logger_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_observability::EventLoggerNode>();
  // R24: single thread -- EventLedger and the raw fd are not synchronised.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
