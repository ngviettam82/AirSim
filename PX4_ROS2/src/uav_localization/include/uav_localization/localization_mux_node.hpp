#ifndef UAV_LOCALIZATION__LOCALIZATION_MUX_NODE_HPP_
#define UAV_LOCALIZATION__LOCALIZATION_MUX_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace uav_localization
{

// Lets one test process drive the real mux.
std::shared_ptr<rclcpp::Node> createLocalizationMuxNode(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace uav_localization

#endif  // UAV_LOCALIZATION__LOCALIZATION_MUX_NODE_HPP_
