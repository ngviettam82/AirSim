#include "uav_px4_backend/frame_conversions.hpp"

#include <cmath>

#include <Eigen/Eigen>
#include <px4_ros_com/frame_transforms.h>

namespace uav_px4_backend
{
namespace frames
{

namespace
{

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Vector3;
using px4_ros_com::frame_transforms::StaticTF;
using px4_ros_com::frame_transforms::transform_orientation;
using px4_ros_com::frame_transforms::transform_static_frame;

Eigen::Vector3d toEigen(const std::array<float, 3> & v)
{
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

/// Wrap to [-pi, pi] so comparisons stay well defined.
double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

Point nedToEnuPoint(const std::array<float, 3> & ned)
{
  const Eigen::Vector3d enu = transform_static_frame(toEigen(ned), StaticTF::NED_TO_ENU);
  Point out;
  out.x = enu.x();
  out.y = enu.y();
  out.z = enu.z();
  return out;
}

Vector3 nedToEnuVector(const std::array<float, 3> & ned)
{
  const Eigen::Vector3d enu = transform_static_frame(toEigen(ned), StaticTF::NED_TO_ENU);
  Vector3 out;
  out.x = enu.x();
  out.y = enu.y();
  out.z = enu.z();
  return out;
}

std::array<float, 3> enuToNedArray(const Point & enu)
{
  const Eigen::Vector3d ned =
    transform_static_frame(Eigen::Vector3d(enu.x, enu.y, enu.z), StaticTF::ENU_TO_NED);
  return {static_cast<float>(ned.x()), static_cast<float>(ned.y()), static_cast<float>(ned.z())};
}

std::array<float, 3> enuToNedArray(const Vector3 & enu)
{
  const Eigen::Vector3d ned =
    transform_static_frame(Eigen::Vector3d(enu.x, enu.y, enu.z), StaticTF::ENU_TO_NED);
  return {static_cast<float>(ned.x()), static_cast<float>(ned.y()), static_cast<float>(ned.z())};
}

Vector3 frdToFluVector(const std::array<float, 3> & frd)
{
  const Eigen::Vector3d flu =
    transform_static_frame(toEigen(frd), StaticTF::AIRCRAFT_TO_BASELINK);
  Vector3 out;
  out.x = flu.x();
  out.y = flu.y();
  out.z = flu.z();
  return out;
}

std::array<float, 3> fluToFrdArray(const Vector3 & flu)
{
  const Eigen::Vector3d frd = transform_static_frame(
    Eigen::Vector3d(flu.x, flu.y, flu.z), StaticTF::BASELINK_TO_AIRCRAFT);
  return {static_cast<float>(frd.x()), static_cast<float>(frd.y()),
    static_cast<float>(frd.z())};
}

Quaternion px4ToRosQuaternion(const std::array<float, 4> & q_px4)
{
  const Eigen::Quaterniond q_frd_ned =
    px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(q_px4);

  // Reference frame first, then body frame.
  const Eigen::Quaterniond q_enu = transform_orientation(q_frd_ned, StaticTF::NED_TO_ENU);
  const Eigen::Quaterniond q_flu = transform_orientation(q_enu, StaticTF::AIRCRAFT_TO_BASELINK);

  Quaternion out;
  out.w = q_flu.w();
  out.x = q_flu.x();
  out.y = q_flu.y();
  out.z = q_flu.z();
  return out;
}

std::array<float, 4> rosToPx4Quaternion(const Quaternion & q_ros)
{
  const Eigen::Quaterniond q_flu_enu(q_ros.w, q_ros.x, q_ros.y, q_ros.z);

  const Eigen::Quaterniond q_ned = transform_orientation(q_flu_enu, StaticTF::ENU_TO_NED);
  const Eigen::Quaterniond q_frd = transform_orientation(q_ned, StaticTF::BASELINK_TO_AIRCRAFT);

  std::array<float, 4> out{};
  px4_ros_com::frame_transforms::utils::quaternion::eigen_quat_to_array(q_frd, out);
  return out;
}

double yawEnuToNed(double yaw_enu)
{
  return normalizeAngle(M_PI_2 - yaw_enu);
}

double yawNedToEnu(double yaw_ned)
{
  return normalizeAngle(M_PI_2 - yaw_ned);
}

double yawRateEnuToNed(double yaw_rate_enu)
{
  return -yaw_rate_enu;
}

Vector3 rotateEnuToBodyFlu(
  const Vector3 & vector_enu,
  const Quaternion & orientation_flu_enu)
{
  const Eigen::Quaterniond q(
    orientation_flu_enu.w, orientation_flu_enu.x,
    orientation_flu_enu.y, orientation_flu_enu.z);
  const Eigen::Vector3d body =
    q.conjugate() * Eigen::Vector3d(vector_enu.x, vector_enu.y, vector_enu.z);

  Vector3 out;
  out.x = body.x();
  out.y = body.y();
  out.z = body.z();
  return out;
}

}  // namespace frames
}  // namespace uav_px4_backend
