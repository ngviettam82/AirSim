#include <rclcpp/rclcpp.hpp>

#include "uav_localization/localization_mux_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(uav_localization::createLocalizationMuxNode());
  rclcpp::shutdown();
  return 0;
}
