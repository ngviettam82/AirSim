#include <rclcpp/rclcpp.hpp>

#include "uav_world_model/world_model_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(uav_world_model::createWorldModelNode());
  rclcpp::shutdown();
  return 0;
}
