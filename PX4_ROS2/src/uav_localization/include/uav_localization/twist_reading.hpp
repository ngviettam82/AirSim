// Reads the twist tri-state. Zero is a gap, not a perfect measurement.

#ifndef UAV_LOCALIZATION__TWIST_READING_HPP_
#define UAV_LOCALIZATION__TWIST_READING_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace uav_localization
{

/// What a twist covariance actually claims. Contract: docs 2.17.
enum class TwistTrust
{
  Unknown,     ///< no usable twist at all
  Stated,      ///< twist plus a declared 1-sigma
  Unstated,    ///< twist present, its error never declared
};

/// Row-major 6x6; element 0 carries the whole claim, as the mux writes it.
constexpr std::size_t kTwistLinearXVariance = 0;

inline TwistTrust readTwistTrust(const std::array<double, 36> & twist_covariance)
{
  const double variance = twist_covariance[kTwistLinearXVariance];

  // Anything unreadable stays unknown; repairing it would be a guess.
  if (!std::isfinite(variance) || variance < 0.0) {
    return TwistTrust::Unknown;
  }
  return variance > 0.0 ? TwistTrust::Stated : TwistTrust::Unstated;
}

/// Empty unless the error was declared. Covariance holds variance, not sigma.
inline std::optional<double> statedTwistStddev(const std::array<double, 36> & twist_covariance)
{
  if (readTwistTrust(twist_covariance) != TwistTrust::Stated) {
    return std::nullopt;
  }
  return std::sqrt(twist_covariance[kTwistLinearXVariance]);
}

}  // namespace uav_localization

#endif  // UAV_LOCALIZATION__TWIST_READING_HPP_
