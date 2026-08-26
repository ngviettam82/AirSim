#ifndef UAV_NAVIGATION__NAVIGATOR_ACTION_SERVER_NODE_HPP_
#define UAV_NAVIGATION__NAVIGATOR_ACTION_SERVER_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace uav_navigation
{

// Lets one test process drive the real action servers.
std::shared_ptr<rclcpp::Node> createNavigatorActionServerNode(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace uav_navigation

#endif  // UAV_NAVIGATION__NAVIGATOR_ACTION_SERVER_NODE_HPP_
