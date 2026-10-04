#include "uav_localization/gauss_markov_drift.hpp"

#include <algorithm>
#include <cmath>

namespace uav_localization
{

GaussMarkovDrift::GaussMarkovDrift(uint32_t seed)
: generator_(seed)
{
}

void GaussMarkovDrift::configure(const DriftParameters & parameters)
{
  const bool seed_changed = parameters.seed != parameters_.seed;
  parameters_ = parameters;
  if (seed_changed) {
    generator_.seed(parameters_.seed);
  }
  if (!parameters_.enabled) {
    reset();
  }
}

void GaussMarkovDrift::reset()
{
  offset_ = {0.0, 0.0, 0.0};
}

std::array<double, 3> GaussMarkovDrift::advance(double dt_sec)
{
  if (!parameters_.enabled || dt_sec <= 0.0 || parameters_.sigma_m <= 0.0) {
    return offset_;
  }

  // No correlation time means no memory: ordinary white noise.
  const double decay = parameters_.correlation_time_sec > 0.0
    ? std::exp(-dt_sec / parameters_.correlation_time_sec)
    : 0.0;
  const double innovation_scale =
    parameters_.sigma_m * std::sqrt(std::max(0.0, 1.0 - decay * decay));

  for (double & axis : offset_) {
    axis = decay * axis + innovation_scale * standard_normal_(generator_);
  }
  return offset_;
}

}  // namespace uav_localization
