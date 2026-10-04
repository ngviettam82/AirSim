// Simulation-only GNSS drift injection, off by default. See README.

#ifndef UAV_LOCALIZATION__GAUSS_MARKOV_DRIFT_HPP_
#define UAV_LOCALIZATION__GAUSS_MARKOV_DRIFT_HPP_

#include <array>
#include <cstdint>
#include <random>

namespace uav_localization
{

struct DriftParameters
{
  bool enabled{false};
  double correlation_time_sec{60.0};
  double sigma_m{5.0};
  /// Sim-only: lets a gate show a PASS is not one lucky draw (G4).
  uint32_t seed{0};
};

/// One independent first-order process per axis.
class GaussMarkovDrift
{
public:
  explicit GaussMarkovDrift(uint32_t seed = 0);

  void configure(const DriftParameters & parameters);
  const DriftParameters & parameters() const {return parameters_;}

  /// Offset in metres; starts at zero so runs stay comparable.
  std::array<double, 3> advance(double dt_sec);

  std::array<double, 3> offset() const {return offset_;}
  void reset();

private:
  DriftParameters parameters_{};
  std::array<double, 3> offset_{{0.0, 0.0, 0.0}};
  std::mt19937 generator_;
  std::normal_distribution<double> standard_normal_{0.0, 1.0};
};

}  // namespace uav_localization

#endif  // UAV_LOCALIZATION__GAUSS_MARKOV_DRIFT_HPP_
