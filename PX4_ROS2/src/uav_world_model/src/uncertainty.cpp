#include "uav_world_model/uncertainty.hpp"

#include <algorithm>
#include <cmath>

namespace uav_world_model
{

namespace
{

// Row-major 6x6 pose covariance diagonal for x, y, z.
constexpr std::size_t kCovarianceX = 0;
constexpr std::size_t kCovarianceY = 7;
constexpr std::size_t kCovarianceZ = 14;

}  // namespace

bool isUsableUncertainty(double value)
{
  return std::isfinite(value) && value > 0.0;
}

double floorValue(const UncertaintyFloor & floor, double range_m)
{
  const double range = (std::isfinite(range_m) && range_m > 0.0) ? range_m : 0.0;
  const double value = floor.base_m + floor.per_metre * range;
  return std::isfinite(value) && value > 0.0 ? value : 0.0;
}

double perceptionUncertainty(double reported_m, double range_m, const UncertaintyFloor & floor)
{
  const double lower_bound = floorValue(floor, range_m);
  if (!isUsableUncertainty(reported_m)) {
    return lower_bound;
  }
  return std::max(reported_m, lower_bound);
}

double driftGrowth(double age_sec, const DriftModel & model)
{
  if (!std::isfinite(age_sec) || age_sec <= 0.0) {
    return 0.0;
  }
  if (!std::isfinite(model.sigma_m) || model.sigma_m <= 0.0) {
    return 0.0;
  }
  if (!std::isfinite(model.correlation_time_sec) || model.correlation_time_sec <= 0.0) {
    return model.sigma_m * std::sqrt(2.0);
  }
  const double decay = std::exp(-age_sec / model.correlation_time_sec);
  return model.sigma_m * std::sqrt(2.0 * (1.0 - decay));
}

double combineQuadrature(std::initializer_list<double> terms)
{
  double sum_of_squares = 0.0;
  for (const double term : terms) {
    if (std::isfinite(term) && term > 0.0) {
      sum_of_squares += term * term;
    }
  }
  return std::sqrt(sum_of_squares);
}

double worstAxisStddev(const std::array<double, 36> & pose_covariance)
{
  const double variances[] = {
    pose_covariance[kCovarianceX],
    pose_covariance[kCovarianceY],
    pose_covariance[kCovarianceZ]};

  double worst = 0.0;
  for (const double variance : variances) {
    if (!std::isfinite(variance) || variance < 0.0) {
      return -1.0;
    }
    worst = std::max(worst, variance);
  }
  if (worst <= 0.0) {
    return -1.0;
  }
  return std::sqrt(worst);
}

}  // namespace uav_world_model
