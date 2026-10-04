#include <rclcpp/rclcpp.hpp>

#include "uav_perception/object_detector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(uav_perception::createObjectDetectorNode());
  rclcpp::shutdown();
  return 0;
}
