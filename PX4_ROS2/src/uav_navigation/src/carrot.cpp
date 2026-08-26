#include "uav_navigation/carrot.hpp"

#include <algorithm>
#include <cmath>

namespace uav_navigation
{

namespace
{
constexpr double kMinOrientationNorm = 0.1;
}

bool isFinite(const Vec3 & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double distance3(const Vec3 & from, const Vec3 & to)
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double dz = to.z - from.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double horizontalDistance(const Vec3 & from, const Vec3 & to)
{
  return std::hypot(to.x - from.x, to.y - from.y);
}

CarrotStep advanceCarrot(
  const Vec3 & carrot, const Vec3 & target, const Vec3 & position,
  const CarrotLimits & limits, double dt)
{
  CarrotStep step;
  step.setpoint = carrot;
  if (!(dt > 0.0) || !std::isfinite(dt) || !isFinite(target) || !isFinite(carrot)) {
    return step;
  }

  Vec3 next = carrot;

  const double dx = target.x - carrot.x;
  const double dy = target.y - carrot.y;
  const double horizontal = std::hypot(dx, dy);
  const double horizontal_step = std::max(0.0, limits.max_speed) * dt;
  if (horizontal <= horizontal_step || horizontal <= 0.0) {
    next.x = target.x;
    next.y = target.y;
  } else {
    next.x = carrot.x + dx / horizontal * horizontal_step;
    next.y = carrot.y + dy / horizontal * horizontal_step;
  }

  const double dz = target.z - carrot.z;
  const double vertical_step = std::max(0.0, limits.max_vertical_speed) * dt;
  if (std::abs(dz) <= vertical_step) {
    next.z = target.z;
  } else {
    next.z = carrot.z + std::copysign(vertical_step, dz);
  }

  // A setpoint that outruns the aircraft turns a lag into a lurch. Each axis is
  // held back on its own budget, so a stuck altitude cannot freeze the flight path.
  if (isFinite(position)) {
    if (limits.max_lead_horizontal > 0.0) {
      const double lead_now = horizontalDistance(position, carrot);
      const double lead_next = horizontalDistance(position, next);
      if (lead_next > limits.max_lead_horizontal && lead_next > lead_now) {
        next.x = carrot.x;
        next.y = carrot.y;
        step.clamped_horizontal = true;
      }
    }
    if (limits.max_lead_vertical > 0.0) {
      const double lead_now = std::abs(carrot.z - position.z);
      const double lead_next = std::abs(next.z - position.z);
      if (lead_next > limits.max_lead_vertical && lead_next > lead_now) {
        next.z = carrot.z;
        step.clamped_vertical = true;
      }
    }
  }

  step.setpoint = next;
  return step;
}

double wrapAngle(double radians)
{
  if (!std::isfinite(radians)) {
    return 0.0;
  }
  // A subtract loop never ends: 1e17 - 2*pi == 1e17 in double.
  const double wrapped = std::remainder(radians, 2.0 * M_PI);
  // remainder() ties to even; keep the caller's side of pi.
  return std::abs(wrapped) == M_PI ? std::copysign(M_PI, radians) : wrapped;
}

double stepYaw(double current, double target, double max_rate, double dt)
{
  if (!std::isfinite(current)) {
    current = 0.0;
  }
  if (!std::isfinite(target) || !(dt > 0.0) || !std::isfinite(dt)) {
    return wrapAngle(current);
  }
  const double error = wrapAngle(target - current);
  const double step = std::max(0.0, max_rate) * dt;
  if (std::abs(error) <= step) {
    return wrapAngle(target);
  }
  return wrapAngle(current + std::copysign(step, error));
}

double yawFromQuaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

bool hasUsableOrientation(double x, double y, double z, double w)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(w)) {
    return false;
  }
  return std::sqrt(x * x + y * y + z * z + w * w) >= kMinOrientationNorm;
}

}  // namespace uav_navigation
