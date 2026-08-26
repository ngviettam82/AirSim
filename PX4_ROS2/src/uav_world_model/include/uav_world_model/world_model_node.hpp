#ifndef UAV_WORLD_MODEL__WORLD_MODEL_NODE_HPP_
#define UAV_WORLD_MODEL__WORLD_MODEL_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace uav_world_model
{

// Lets one test process drive the whole chain.
std::shared_ptr<rclcpp::Node> createWorldModelNode(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace uav_world_model

#endif  // UAV_WORLD_MODEL__WORLD_MODEL_NODE_HPP_
