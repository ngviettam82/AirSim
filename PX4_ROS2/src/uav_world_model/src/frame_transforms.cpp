#include "uav_world_model/frame_transforms.hpp"

#include <cmath>

#include "uav_perception/marker_pose.hpp"

namespace uav_world_model
{

// Derived from the same axis permutation uav_perception::opticalToBody applies.
const Quat kOpticalToBody{-0.5, 0.5, -0.5, 0.5};

Quat fromRollPitchYaw(double roll, double pitch, double yaw)
{
  const double half_roll = 0.5 * roll;
  const double half_pitch = 0.5 * pitch;
  const double half_yaw = 0.5 * yaw;

  const double sin_roll = std::sin(half_roll);
  const double cos_roll = std::cos(half_roll);
  const double sin_pitch = std::sin(half_pitch);
  const double cos_pitch = std::cos(half_pitch);
  const double sin_yaw = std::sin(half_yaw);
  const double cos_yaw = std::cos(half_yaw);

  Quat result;
  result.x = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
  result.y = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
  result.z = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
  result.w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
  return result;
}

Quat multiply(const Quat & left, const Quat & right)
{
  Quat result;
  result.w = left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z;
  result.x = left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y;
  result.y = left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x;
  result.z = left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w;
  return result;
}

Quat normalized(const Quat & value)
{
  const double length = std::sqrt(
    value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w);
  if (!std::isfinite(length) || length <= 0.0) {
    return Quat{};
  }
  return Quat{value.x / length, value.y / length, value.z / length, value.w / length};
}

Quat slerp(const Quat & from, const Quat & to, double ratio)
{
  const Quat left = normalized(from);
  Quat right = normalized(to);

  double dot = left.x * right.x + left.y * right.y + left.z * right.z + left.w * right.w;
  // Opposite signs mean the same rotation, longer path.
  if (dot < 0.0) {
    right = Quat{-right.x, -right.y, -right.z, -right.w};
    dot = -dot;
  }

  if (dot > 0.9995) {
    Quat result;
    result.x = left.x + ratio * (right.x - left.x);
    result.y = left.y + ratio * (right.y - left.y);
    result.z = left.z + ratio * (right.z - left.z);
    result.w = left.w + ratio * (right.w - left.w);
    return normalized(result);
  }

  const double angle = std::acos(dot);
  const double sin_angle = std::sin(angle);
  const double weight_left = std::sin((1.0 - ratio) * angle) / sin_angle;
  const double weight_right = std::sin(ratio * angle) / sin_angle;

  Quat result;
  result.x = weight_left * left.x + weight_right * right.x;
  result.y = weight_left * left.y + weight_right * right.y;
  result.z = weight_left * left.z + weight_right * right.z;
  result.w = weight_left * left.w + weight_right * right.w;
  return normalized(result);
}

Vec3 rotate(const Quat & rotation, const Vec3 & point)
{
  const Quat unit = normalized(rotation);
  const Vec3 axis{unit.x, unit.y, unit.z};

  const Vec3 half_cross{
    axis.y * point.z - axis.z * point.y + unit.w * point.x,
    axis.z * point.x - axis.x * point.z + unit.w * point.y,
    axis.x * point.y - axis.y * point.x + unit.w * point.z};

  const Vec3 correction{
    axis.y * half_cross.z - axis.z * half_cross.y,
    axis.z * half_cross.x - axis.x * half_cross.z,
    axis.x * half_cross.y - axis.y * half_cross.x};

  return Vec3{
    point.x + 2.0 * correction.x,
    point.y + 2.0 * correction.y,
    point.z + 2.0 * correction.z};
}

Vec3 apply(const Rigid & transform, const Vec3 & point)
{
  const Vec3 rotated = rotate(transform.rotation, point);
  return Vec3{
    rotated.x + transform.translation.x,
    rotated.y + transform.translation.y,
    rotated.z + transform.translation.z};
}

Vec3 add(const Vec3 & left, const Vec3 & right)
{
  return Vec3{left.x + right.x, left.y + right.y, left.z + right.z};
}

Vec3 subtract(const Vec3 & left, const Vec3 & right)
{
  return Vec3{left.x - right.x, left.y - right.y, left.z - right.z};
}

Vec3 cross(const Vec3 & left, const Vec3 & right)
{
  return Vec3{
    left.y * right.z - left.z * right.y,
    left.z * right.x - left.x * right.z,
    left.x * right.y - left.y * right.x};
}

Vec3 scale(const Vec3 & value, double factor)
{
  return Vec3{value.x * factor, value.y * factor, value.z * factor};
}

double norm(const Vec3 & value)
{
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 opticalToBody(const Vec3 & optical)
{
  Vec3 body;
  uav_perception::opticalToBody(optical.x, optical.y, optical.z, body.x, body.y, body.z);
  return body;
}

Quat opticalToBody(const Quat & optical)
{
  return multiply(kOpticalToBody, optical);
}

namespace
{

Vec3 intoSensorBodyAxes(const Vec3 & value, const SensorMount & mount)
{
  return mount.reports_optical_axes ? opticalToBody(value) : value;
}

}  // namespace

Vec3 sensorPointToOdom(
  const Vec3 & sensor_point, const SensorMount & mount, const Rigid & odom_from_base)
{
  const Vec3 in_base = apply(mount.base_from_sensor, intoSensorBodyAxes(sensor_point, mount));
  return apply(odom_from_base, in_base);
}

Quat sensorRotationToOdom(
  const Quat & sensor_rotation, const SensorMount & mount, const Rigid & odom_from_base)
{
  const Quat in_sensor_body = mount.reports_optical_axes ?
    opticalToBody(sensor_rotation) : normalized(sensor_rotation);
  const Quat in_base = multiply(mount.base_from_sensor.rotation, in_sensor_body);
  return normalized(multiply(odom_from_base.rotation, in_base));
}

Vec3 sensorVectorToOdom(
  const Vec3 & sensor_vector, const SensorMount & mount, const Rigid & odom_from_base)
{
  const Vec3 in_base =
    rotate(mount.base_from_sensor.rotation, intoSensorBodyAxes(sensor_vector, mount));
  return rotate(odom_from_base.rotation, in_base);
}

Vec3 axisAlignedExtent(const Vec3 & extent, const Quat & rotation)
{
  const Vec3 axis_x = rotate(rotation, Vec3{1.0, 0.0, 0.0});
  const Vec3 axis_y = rotate(rotation, Vec3{0.0, 1.0, 0.0});
  const Vec3 axis_z = rotate(rotation, Vec3{0.0, 0.0, 1.0});

  return Vec3{
    std::abs(axis_x.x) * extent.x + std::abs(axis_y.x) * extent.y +
    std::abs(axis_z.x) * extent.z,
    std::abs(axis_x.y) * extent.x + std::abs(axis_y.y) * extent.y +
    std::abs(axis_z.y) * extent.z,
    std::abs(axis_x.z) * extent.x + std::abs(axis_y.z) * extent.y +
    std::abs(axis_z.z) * extent.z};
}

}  // namespace uav_world_model
