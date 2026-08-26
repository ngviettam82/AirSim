#ifndef UAV_NAVIGATION__TRAJECTORY_HPP_
#define UAV_NAVIGATION__TRAJECTORY_HPP_

#include <string>
#include <vector>

#include "uav_navigation/carrot.hpp"

namespace uav_navigation
{

struct TrajectoryLimits
{
  double max_speed = 0.55;
  double max_vertical_speed = 0.45;
  double max_acceleration = 1.0;
  double max_yaw_rate = 0.5;

  // Below this speed the heading of travel is meaningless, so yaw holds.
  double yaw_deadband_speed = 0.15;

  double sample_hz = 50.0;
  double max_duration = 300.0;
};

struct TrajectorySample
{
  Vec3 position;
  Vec3 velocity;
  Vec3 acceleration;
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

/// Clamped uniform cubic B-spline through waypoints, parameterized by time.
///
/// Approximating, not interpolating: interior waypoints are smoothed, so every
/// sample stays inside the convex hull of the waypoints. That hull is NOT the
/// corridor around the polyline -- a corner is cut by |W0 - 2*W1 + W2| / 6, which
/// the planner above must carry as inflation (plan P6 section 2c).
/// Endpoints are exact and both ends start and finish at rest.
class Trajectory
{
public:
  Trajectory() = default;

  static Trajectory build(
    const std::vector<Vec3> & waypoints, double initial_yaw,
    const TrajectoryLimits & limits, std::string & reason);

  bool valid() const {return valid_;}
  double duration() const {return duration_;}
  double gridPeriod() const {return grid_period_;}
  double peakSpeed() const {return peak_speed_;}
  double peakVerticalSpeed() const {return peak_vertical_speed_;}
  double peakAcceleration() const {return peak_acceleration_;}

  /// Worst heading error while moving: the nose slews, the path does not wait.
  double peakYawLag() const {return peak_yaw_lag_;}
  const std::vector<TrajectorySample> & grid() const {return grid_;}

  /// Clamped outside [0, duration]; before the start and after the end it holds still.
  TrajectorySample sample(double time) const;

  Vec3 finalPosition() const;

private:
  bool valid_ = false;
  double duration_ = 0.0;
  double grid_period_ = 0.02;
  double peak_speed_ = 0.0;
  double peak_vertical_speed_ = 0.0;
  double peak_acceleration_ = 0.0;
  double peak_yaw_lag_ = 0.0;
  std::vector<TrajectorySample> grid_;
};

}  // namespace uav_navigation

#endif  // UAV_NAVIGATION__TRAJECTORY_HPP_
