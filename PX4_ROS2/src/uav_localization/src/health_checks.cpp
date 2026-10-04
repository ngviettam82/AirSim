#include "uav_localization/health_checks.hpp"

#include <algorithm>
#include <cmath>

namespace uav_localization
{

bool isImplausibleJump(double step_m, double dt_sec, const JumpCheck & check)
{
  if (!std::isfinite(step_m) || !std::isfinite(dt_sec) || dt_sec <= 0.0) {
    return false;
  }
  return step_m > check.max_speed_mps * dt_sec + check.tolerance_m;
}

JumpVerdict classifyJump(
  double step_m, double dt_sec, double declared_sigma_m, const JumpCheck & check)
{
  if (!std::isfinite(step_m) || !std::isfinite(dt_sec) || dt_sec <= 0.0) {
    return JumpVerdict::CannotJudge;
  }
  if (!std::isfinite(declared_sigma_m) || declared_sigma_m < 0.0 ||
    declared_sigma_m > check.max_sigma_m)
  {
    return JumpVerdict::CannotJudge;
  }
  const double allowance = check.max_speed_mps * dt_sec + check.tolerance_m +
    check.noise_sigmas * declared_sigma_m;
  return step_m > allowance ? JumpVerdict::Jumped : JumpVerdict::Continuous;
}

HealthLevel classifyDisagreement(
  double gap_m, double stddev_a, double stddev_b, const DisagreementCheck & check)
{
  if (!std::isfinite(gap_m) || stddev_a < 0.0 || stddev_b < 0.0) {
    return HealthLevel::Ok;
  }

  const double combined = std::max(
    check.floor_m, std::sqrt(stddev_a * stddev_a + stddev_b * stddev_b));
  const double ratio = gap_m / combined;

  if (ratio > check.error_sigma) {
    return HealthLevel::Error;
  }
  if (ratio > check.warn_sigma) {
    return HealthLevel::Warn;
  }
  return HealthLevel::Ok;
}

HealthLevel classifyRate(double measured_hz, const RateCheck & check)
{
  if (!std::isfinite(measured_hz) || check.expected_hz <= 0.0) {
    return HealthLevel::Error;
  }
  const double fraction = measured_hz / check.expected_hz;

  if (fraction < check.error_fraction) {
    return HealthLevel::Error;
  }
  if (fraction < check.warn_fraction) {
    return HealthLevel::Warn;
  }
  return HealthLevel::Ok;
}

HealthLevel classifyCrossCheck(std::size_t sources_compared)
{
  return sources_compared >= 2 ? HealthLevel::Ok : HealthLevel::Warn;
}

}  // namespace uav_localization
