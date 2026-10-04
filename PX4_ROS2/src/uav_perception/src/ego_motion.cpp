#include "uav_perception/ego_motion.hpp"

#include <cmath>

namespace uav_perception
{

bool BodyPose::valid() const
{
  return orientation.valid() && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

bool isPoseFresh(double age_sec, double max_age_sec)
{
  return std::isfinite(age_sec) && age_sec >= 0.0 && age_sec <= max_age_sec;
}

bool resolveBodyPose(
  bool have_odometry, const BodyPose & odometry_pose,
  double age_sec, double max_age_sec, BodyPose & out_pose)
{
  if (!have_odometry || !isPoseFresh(age_sec, max_age_sec) || !odometry_pose.valid()) {
    return false;
  }
  out_pose = odometry_pose;
  return true;
}

namespace
{

// Defensive normalize-or-identity, independently implemented here (mirrors
// obstacle_extraction's normalizedOrIdentity but is a separate copy: this
// library must not depend on obstacle_extraction, see README).
AttitudeQuaternion normalizedOrIdentity(const AttitudeQuaternion & attitude)
{
  if (!attitude.valid()) {
    return AttitudeQuaternion{};
  }
  const double norm = std::sqrt(
    attitude.x * attitude.x + attitude.y * attitude.y +
    attitude.z * attitude.z + attitude.w * attitude.w);
  return AttitudeQuaternion{
    attitude.x / norm, attitude.y / norm, attitude.z / norm, attitude.w / norm};
}

// General active quaternion-vector rotation (v' = q*v*q^-1), textbook
// optimized form: t=2*(u x v); v'=v+w*t+(u x t), u = vector part of q.
void rotate(
  const AttitudeQuaternion & attitude, double x, double y, double z,
  double & out_x, double & out_y, double & out_z)
{
  const AttitudeQuaternion q = normalizedOrIdentity(attitude);
  const double tx = 2.0 * (q.y * z - q.z * y);
  const double ty = 2.0 * (q.z * x - q.x * z);
  const double tz = 2.0 * (q.x * y - q.y * x);
  out_x = x + q.w * tx + (q.y * tz - q.z * ty);
  out_y = y + q.w * ty + (q.z * tx - q.x * tz);
  out_z = z + q.w * tz + (q.x * ty - q.y * tx);
}

// Inverse rotation: rotate by the conjugate quaternion (-x,-y,-z,w).
void rotateInverse(
  const AttitudeQuaternion & attitude, double x, double y, double z,
  double & out_x, double & out_y, double & out_z)
{
  const AttitudeQuaternion q = normalizedOrIdentity(attitude);
  const AttitudeQuaternion conjugate{-q.x, -q.y, -q.z, q.w};
  rotate(conjugate, x, y, z, out_x, out_y, out_z);
}

}  // namespace

void opticalPointToOdom(
  double optical_x, double optical_y, double optical_z, const BodyPose & body_to_odom,
  double & odom_x, double & odom_y, double & odom_z)
{
  double body_x = 0.0;
  double body_y = 0.0;
  double body_z = 0.0;
  opticalToBody(optical_x, optical_y, optical_z, body_x, body_y, body_z);

  double rotated_x = 0.0;
  double rotated_y = 0.0;
  double rotated_z = 0.0;
  rotate(body_to_odom.orientation, body_x, body_y, body_z, rotated_x, rotated_y, rotated_z);

  odom_x = rotated_x + body_to_odom.x;
  odom_y = rotated_y + body_to_odom.y;
  odom_z = rotated_z + body_to_odom.z;
}

void odomPointToOptical(
  double odom_x, double odom_y, double odom_z, const BodyPose & body_to_odom,
  double & optical_x, double & optical_y, double & optical_z)
{
  const double relative_x = odom_x - body_to_odom.x;
  const double relative_y = odom_y - body_to_odom.y;
  const double relative_z = odom_z - body_to_odom.z;

  double body_x = 0.0;
  double body_y = 0.0;
  double body_z = 0.0;
  rotateInverse(body_to_odom.orientation, relative_x, relative_y, relative_z, body_x, body_y, body_z);

  bodyToOptical(body_x, body_y, body_z, optical_x, optical_y, optical_z);
}

void odomVectorToOptical(
  double odom_vx, double odom_vy, double odom_vz, const BodyPose & body_to_odom,
  double & optical_vx, double & optical_vy, double & optical_vz)
{
  double body_vx = 0.0;
  double body_vy = 0.0;
  double body_vz = 0.0;
  rotateInverse(body_to_odom.orientation, odom_vx, odom_vy, odom_vz, body_vx, body_vy, body_vz);

  bodyToOptical(body_vx, body_vy, body_vz, optical_vx, optical_vy, optical_vz);
}

}  // namespace uav_perception
