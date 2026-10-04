#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "uav_mission/mission_executor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // R24: MultiThreadedExecutor -- tick_group_ (MutuallyExclusive) and
  // io_group_ (Reentrant) both need real worker threads to run
  // concurrently; a SingleThreadedExecutor would serialize them anyway,
  // defeating the point of a Reentrant io_group_ (uav_mission/README.md).
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<uav_mission::MissionExecutorNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
