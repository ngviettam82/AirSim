#include <rclcpp/rclcpp.hpp>

#include "uav_localization/localization_health_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(uav_localization::createLocalizationHealthNode());
  rclcpp::shutdown();
  return 0;
}
