#ifndef UAV_PERCEPTION__OBJECT_DETECTOR_NODE_HPP_
#define UAV_PERCEPTION__OBJECT_DETECTOR_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace uav_perception
{

// Lets one test process drive the whole node (mirrors world_model_node).
std::shared_ptr<rclcpp::Node> createObjectDetectorNode(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace uav_perception

#endif  // UAV_PERCEPTION__OBJECT_DETECTOR_NODE_HPP_
