// Slant beam distance -> tilt-corrected height. See README.

#ifndef UAV_LOCALIZATION__HEIGHT_FROM_RANGE_HPP_
#define UAV_LOCALIZATION__HEIGHT_FROM_RANGE_HPP_

#include <geometry_msgs/msg/quaternion.hpp>

namespace uav_localization
{

struct HeightEstimate
{
  bool valid{false};
  double height_m{0.0};
  double tilt_rad{0.0};
  const char * reason{""};
};

/// 1.0 level; 0.0 beam pointing at the horizon.
double tiltCosine(const geometry_msgs::msg::Quaternion & orientation_flu_enu);

/// Past max_tilt_rad the beam is refused, not corrected.
HeightEstimate heightFromRange(
  double measured_range_m,
  const geometry_msgs::msg::Quaternion & orientation_flu_enu,
  double min_range_m,
  double max_range_m,
  double max_tilt_rad);

}  // namespace uav_localization

#endif  // UAV_LOCALIZATION__HEIGHT_FROM_RANGE_HPP_
