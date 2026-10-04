// How localization fails, stated as machine-checkable tests. See README.

#ifndef UAV_LOCALIZATION__HEALTH_CHECKS_HPP_
#define UAV_LOCALIZATION__HEALTH_CHECKS_HPP_

#include <cstddef>

namespace uav_localization
{

enum class HealthLevel
{
  Ok = 0,
  Warn = 1,
  Error = 2
};

struct JumpCheck
{
  double max_speed_mps{20.0};
  double tolerance_m{0.2};
  double noise_sigmas{2.0};     // see README: the source declares its own sigma
  double max_sigma_m{5.0};      // past this the test cannot judge, and must not pass
};

/// A step no aircraft could fly is an estimator jump.
bool isImplausibleJump(double step_m, double dt_sec, const JumpCheck & check);

enum class JumpVerdict
{
  Continuous = 0,
  Jumped = 1,
  CannotJudge = 2
};

/// Same test, widened by the uncertainty the source itself declared.
JumpVerdict classifyJump(
  double step_m, double dt_sec, double declared_sigma_m, const JumpCheck & check);

struct DisagreementCheck
{
  double warn_sigma{3.0};
  double error_sigma{6.0};
  double floor_m{0.2};      // stops an over-confident source dividing by zero
};

HealthLevel classifyDisagreement(
  double gap_m, double stddev_a, double stddev_b, const DisagreementCheck & check);

struct RateCheck
{
  double expected_hz{10.0};
  double warn_fraction{0.8};
  double error_fraction{0.5};
};

HealthLevel classifyRate(double measured_hz, const RateCheck & check);

/// Cannot cross-check is not the same as agrees: one source alone is a warning.
HealthLevel classifyCrossCheck(std::size_t sources_compared);

}  // namespace uav_localization

#endif  // UAV_LOCALIZATION__HEALTH_CHECKS_HPP_
