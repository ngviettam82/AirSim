#ifndef UAV_WORLD_MODEL__FRAME_TRANSFORMS_HPP_
#define UAV_WORLD_MODEL__FRAME_TRANSFORMS_HPP_

#include <string>

namespace uav_world_model
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quat
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

// Maps a child-frame point into the parent frame.
struct Rigid
{
  Quat rotation;
  Vec3 translation;
};

struct SensorMount
{
  std::string frame_id;
  Rigid base_from_sensor;
  bool reports_optical_axes{false};
};

Quat fromRollPitchYaw(double roll, double pitch, double yaw);
Quat multiply(const Quat & left, const Quat & right);
Quat normalized(const Quat & value);
Quat slerp(const Quat & from, const Quat & to, double ratio);
Vec3 rotate(const Quat & rotation, const Vec3 & point);
Vec3 apply(const Rigid & transform, const Vec3 & point);
Vec3 add(const Vec3 & left, const Vec3 & right);
Vec3 subtract(const Vec3 & left, const Vec3 & right);
Vec3 cross(const Vec3 & left, const Vec3 & right);
Vec3 scale(const Vec3 & value, double factor);
double norm(const Vec3 & value);

// optical(right,down,fwd) -> body(fwd,left,up); pinned against uav_perception.
extern const Quat kOpticalToBody;

Vec3 opticalToBody(const Vec3 & optical);
Quat opticalToBody(const Quat & optical);

Vec3 sensorPointToOdom(
  const Vec3 & sensor_point, const SensorMount & mount, const Rigid & odom_from_base);
Quat sensorRotationToOdom(
  const Quat & sensor_rotation, const SensorMount & mount, const Rigid & odom_from_base);

// No translation term; the mount offset must not leak.
Vec3 sensorVectorToOdom(
  const Vec3 & sensor_vector, const SensorMount & mount, const Rigid & odom_from_base);

// Axis-aligned box enclosing the rotated one.
Vec3 axisAlignedExtent(const Vec3 & extent, const Quat & rotation);

}  // namespace uav_world_model

#endif  // UAV_WORLD_MODEL__FRAME_TRANSFORMS_HPP_
