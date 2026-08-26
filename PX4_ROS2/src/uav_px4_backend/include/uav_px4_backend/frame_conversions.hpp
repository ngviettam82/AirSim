// Only place ROS ENU/FLU and PX4 NED/FRD ever meet (R5).

#ifndef UAV_PX4_BACKEND__FRAME_CONVERSIONS_HPP_
#define UAV_PX4_BACKEND__FRAME_CONVERSIONS_HPP_

#include <array>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace uav_px4_backend
{
namespace frames
{

geometry_msgs::msg::Point nedToEnuPoint(const std::array<float, 3> & ned);
geometry_msgs::msg::Vector3 nedToEnuVector(const std::array<float, 3> & ned);

std::array<float, 3> enuToNedArray(const geometry_msgs::msg::Point & enu);
std::array<float, 3> enuToNedArray(const geometry_msgs::msg::Vector3 & enu);

geometry_msgs::msg::Vector3 frdToFluVector(const std::array<float, 3> & frd);

std::array<float, 3> fluToFrdArray(const geometry_msgs::msg::Vector3 & flu);

/// PX4 quaternion arrays are wxyz, FRD-to-NED.
geometry_msgs::msg::Quaternion px4ToRosQuaternion(const std::array<float, 4> & q_px4);
std::array<float, 4> rosToPx4Quaternion(const geometry_msgs::msg::Quaternion & q_ros);

/// Involution, so one formula serves both directions.
double yawEnuToNed(double yaw_enu);
double yawNedToEnu(double yaw_ned);

/// The frames spin opposite ways, so only the sign flips.
double yawRateEnuToNed(double yaw_rate_enu);

/// REP-105 wants twist in the child frame.
geometry_msgs::msg::Vector3 rotateEnuToBodyFlu(
  const geometry_msgs::msg::Vector3 & vector_enu,
  const geometry_msgs::msg::Quaternion & orientation_flu_enu);

}  // namespace frames
}  // namespace uav_px4_backend

#endif  // UAV_PX4_BACKEND__FRAME_CONVERSIONS_HPP_
