#include "uav_localization/pose_degrader.hpp"

namespace uav_localization
{

// Golden-ratio mix keeps noise and drift independent from one seed.
PoseDegrader::PoseDegrader(uint32_t seed)
: drift_(seed ^ 0x9e3779b9U), generator_(seed)
{
}

void PoseDegrader::configure(const DegradeParameters & parameters)
{
  parameters_ = parameters;

  DriftParameters drift = parameters_.drift;
  drift.enabled = parameters_.enabled && drift.enabled;
  drift_.configure(drift);

  if (!parameters_.enabled) {
    reset();
  }
}

void PoseDegrader::reset()
{
  drift_.reset();
}

std::array<double, 3> PoseDegrader::apply(
  const std::array<double, 3> & position, double dt_sec)
{
  if (!parameters_.enabled) {
    return position;
  }

  const auto drift_offset = drift_.advance(dt_sec);

  std::array<double, 3> degraded = position;
  for (size_t axis = 0; axis < degraded.size(); ++axis) {
    degraded[axis] += drift_offset[axis];
    if (parameters_.white_noise_stddev_m > 0.0) {
      degraded[axis] += parameters_.white_noise_stddev_m * standard_normal_(generator_);
    }
  }
  return degraded;
}

}  // namespace uav_localization
