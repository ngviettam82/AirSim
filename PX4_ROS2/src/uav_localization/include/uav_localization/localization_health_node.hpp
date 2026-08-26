#ifndef UAV_LOCALIZATION__LOCALIZATION_HEALTH_NODE_HPP_
#define UAV_LOCALIZATION__LOCALIZATION_HEALTH_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace uav_localization
{

// Lets one test process drive the real health node.
std::shared_ptr<rclcpp::Node> createLocalizationHealthNode(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace uav_localization

#endif  // UAV_LOCALIZATION__LOCALIZATION_HEALTH_NODE_HPP_
