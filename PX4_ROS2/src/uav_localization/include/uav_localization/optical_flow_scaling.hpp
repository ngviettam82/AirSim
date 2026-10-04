// Pixel motion -> metric velocity, with rotation removed. See README.

#ifndef UAV_LOCALIZATION__OPTICAL_FLOW_SCALING_HPP_
#define UAV_LOCALIZATION__OPTICAL_FLOW_SCALING_HPP_

#include <cstddef>

namespace uav_localization
{

struct FlowSample
{
  double median_u_px{0.0};      // image right
  double median_v_px{0.0};      // image down
  double dt_sec{0.0};
  size_t tracked_features{0};
};

struct FlowGeometry
{
  double fx{0.0};
  double fy{0.0};
  double height_m{0.0};         // above ground, not above the estimator origin
};

/// FLU, as published on the raw odometry twist.
struct BodyRates
{
  double roll_rate{0.0};
  double pitch_rate{0.0};
};

struct FlowLimits
{
  size_t min_features{20};
  double min_height_m{0.3};
  double max_height_m{25.0};
  double max_speed_mps{20.0};
  double pixel_noise_px{1.0};
  // Measured by experiment, not derived: the mounting decides them.
  double roll_rate_sign{1.0};
  double pitch_rate_sign{1.0};
};

struct FlowVelocity
{
  bool valid{false};
  double forward_mps{0.0};      // body +x
  double left_mps{0.0};         // body +y
  double stddev_mps{-1.0};
  const char * reason{""};
};

FlowVelocity velocityFromFlow(
  const FlowSample & sample, const FlowGeometry & geometry,
  const BodyRates & rates, const FlowLimits & limits);

}  // namespace uav_localization

#endif  // UAV_LOCALIZATION__OPTICAL_FLOW_SCALING_HPP_
