// Simulation-only degradation, off by default. See README.

#ifndef UAV_LOCALIZATION__POSE_DEGRADER_HPP_
#define UAV_LOCALIZATION__POSE_DEGRADER_HPP_

#include <array>
#include <cstdint>
#include <random>

#include "uav_localization/gauss_markov_drift.hpp"

namespace uav_localization
{

struct DegradeParameters
{
  bool enabled{false};
  double white_noise_stddev_m{0.0};   // uncorrelated, per axis
  DriftParameters drift{};            // correlated wander
};

/// Latency and loss of lock stay in the node.
class PoseDegrader
{
public:
  explicit PoseDegrader(uint32_t seed = 0);

  void configure(const DegradeParameters & parameters);
  const DegradeParameters & parameters() const {return parameters_;}

  std::array<double, 3> apply(const std::array<double, 3> & position, double dt_sec);

  void reset();

private:
  DegradeParameters parameters_{};
  GaussMarkovDrift drift_;
  std::mt19937 generator_;
  std::normal_distribution<double> standard_normal_{0.0, 1.0};
};

}  // namespace uav_localization

#endif  // UAV_LOCALIZATION__POSE_DEGRADER_HPP_
