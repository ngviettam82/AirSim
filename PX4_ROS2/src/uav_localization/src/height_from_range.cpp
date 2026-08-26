#include "uav_localization/height_from_range.hpp"

#include <cmath>

namespace uav_localization
{

using geometry_msgs::msg::Quaternion;

double tiltCosine(const Quaternion & q)
{
  // Rotation matrix (2,2): world-Z component of body Z.
  const double norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (!std::isfinite(norm_sq) || norm_sq < 1e-9) {
    return 0.0;
  }
  const double r22 = (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) / norm_sq;
  return r22;
}

HeightEstimate heightFromRange(
  double measured_range_m,
  const Quaternion & orientation_flu_enu,
  double min_range_m,
  double max_range_m,
  double max_tilt_rad)
{
  HeightEstimate estimate;

  if (!std::isfinite(measured_range_m)) {
    estimate.reason = "no return";
    return estimate;
  }
  if (measured_range_m < min_range_m) {
    estimate.reason = "below minimum range";
    return estimate;
  }
  if (measured_range_m > max_range_m) {
    estimate.reason = "beyond maximum range";
    return estimate;
  }

  const double cos_tilt = tiltCosine(orientation_flu_enu);
  if (cos_tilt <= 0.0) {
    estimate.reason = "beam not pointing downwards";
    return estimate;
  }

  estimate.tilt_rad = std::acos(std::min(1.0, cos_tilt));
  if (estimate.tilt_rad > max_tilt_rad) {
    estimate.reason = "tilted too far to describe the ground below";
    return estimate;
  }

  estimate.height_m = measured_range_m * cos_tilt;
  estimate.valid = true;
  estimate.reason = "ok";
  return estimate;
}

}  // namespace uav_localization
