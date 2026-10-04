#ifndef UAV_WORLD_MODEL__UNCERTAINTY_HPP_
#define UAV_WORLD_MODEL__UNCERTAINTY_HPP_

#include <array>
#include <initializer_list>

namespace uav_world_model
{

// Measured detector error, not a statistical sigma; see README.
struct UncertaintyFloor
{
  double base_m{0.0};
  double per_metre{0.0};
};

// Relative drift of the odom frame between two instants.
struct DriftModel
{
  double sigma_m{0.5};
  double correlation_time_sec{120.0};
};

double floorValue(const UncertaintyFloor & floor, double range_m);

double perceptionUncertainty(double reported_m, double range_m, const UncertaintyFloor & floor);

double driftGrowth(double age_sec, const DriftModel & model);

double combineQuadrature(std::initializer_list<double> terms);

// -1 when no usable error is stated; zero counts.
double worstAxisStddev(const std::array<double, 36> & pose_covariance);

bool isUsableUncertainty(double value);

}  // namespace uav_world_model

#endif  // UAV_WORLD_MODEL__UNCERTAINTY_HPP_
