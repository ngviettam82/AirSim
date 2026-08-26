#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "uav_navigation/navigator_action_server_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Multi threaded so a blocking action never starves the setpoint stream.
  rclcpp::executors::MultiThreadedExecutor executor;

  std::shared_ptr<rclcpp::Node> node;
  try {
    // Executor keeps only a weak_ptr; a temporary here would destroy the node.
    node = uav_navigation::createNavigatorActionServerNode();
  } catch (const std::exception & refusal) {
    // Unusable parameters must read as a refusal to fly, not as a crash.
    RCLCPP_FATAL(rclcpp::get_logger("navigator"), "not starting: %s", refusal.what());
    rclcpp::shutdown();
    return 1;
  }

  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
