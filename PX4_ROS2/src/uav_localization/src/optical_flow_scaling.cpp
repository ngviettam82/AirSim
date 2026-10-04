#include "uav_localization/optical_flow_scaling.hpp"

#include <cmath>

namespace uav_localization
{

FlowVelocity velocityFromFlow(
  const FlowSample & sample, const FlowGeometry & geometry,
  const BodyRates & rates, const FlowLimits & limits)
{
  FlowVelocity velocity;

  if (sample.tracked_features < limits.min_features) {
    velocity.reason = "too few tracked features";
    return velocity;
  }
  if (!(sample.dt_sec > 0.0) || !std::isfinite(sample.dt_sec)) {
    velocity.reason = "no usable time base";
    return velocity;
  }
  if (!(geometry.fx > 0.0) || !(geometry.fy > 0.0)) {
    velocity.reason = "camera intrinsics missing";
    return velocity;
  }
  if (!std::isfinite(geometry.height_m) ||
    geometry.height_m < limits.min_height_m ||
    geometry.height_m > limits.max_height_m)
  {
    velocity.reason = "height above ground unusable";
    return velocity;
  }

  // Rotation sweeps f*w*dt pixels regardless of ground distance.
  const double rotation_u_px =
    limits.roll_rate_sign * geometry.fx * rates.roll_rate * sample.dt_sec;
  const double rotation_v_px =
    limits.pitch_rate_sign * geometry.fy * rates.pitch_rate * sample.dt_sec;

  const double translation_u_px = sample.median_u_px - rotation_u_px;
  const double translation_v_px = sample.median_v_px - rotation_v_px;

  velocity.left_mps =
    translation_u_px * geometry.height_m / (geometry.fx * sample.dt_sec);
  velocity.forward_mps =
    translation_v_px * geometry.height_m / (geometry.fy * sample.dt_sec);

  const double speed =
    std::hypot(velocity.forward_mps, velocity.left_mps);
  if (speed > limits.max_speed_mps) {
    velocity.forward_mps = 0.0;
    velocity.left_mps = 0.0;
    velocity.reason = "implausible speed, tracking probably wrong";
    return velocity;
  }

  // One pixel of error grows with height, shrinks with dt.
  velocity.stddev_mps =
    limits.pixel_noise_px * geometry.height_m / (geometry.fx * sample.dt_sec);
  velocity.valid = true;
  velocity.reason = "";
  return velocity;
}

}  // namespace uav_localization
