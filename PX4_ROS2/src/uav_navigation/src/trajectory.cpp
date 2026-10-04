#include "uav_navigation/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace uav_navigation
{

namespace
{

constexpr double kDuplicateWaypointDistance = 1e-3;
constexpr int kPeakScanSteps = 64;
constexpr std::size_t kMaxGridPoints = 20000;
constexpr double kMinSampleHz = 40.0;
constexpr double kMaxSampleHz = 1000.0;

struct CubicSegment
{
  Vec3 a0;
  Vec3 a1;
  Vec3 a2;
  Vec3 a3;
};

Vec3 add(const Vec3 & left, const Vec3 & right)
{
  return Vec3{left.x + right.x, left.y + right.y, left.z + right.z};
}

Vec3 scale(const Vec3 & point, double factor)
{
  return Vec3{point.x * factor, point.y * factor, point.z * factor};
}

double norm(const Vec3 & point)
{
  return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

Vec3 lerp(const Vec3 & from, const Vec3 & to, double fraction)
{
  return Vec3{
    from.x + (to.x - from.x) * fraction,
    from.y + (to.y - from.y) * fraction,
    from.z + (to.z - from.z) * fraction};
}

/// Uniform cubic B-spline basis over four control points.
CubicSegment makeSegment(const Vec3 & q0, const Vec3 & q1, const Vec3 & q2, const Vec3 & q3)
{
  CubicSegment segment;
  segment.a0 = scale(add(add(q0, scale(q1, 4.0)), q2), 1.0 / 6.0);
  segment.a1 = scale(add(scale(q0, -1.0), q2), 0.5);
  segment.a2 = scale(add(add(q0, scale(q1, -2.0)), q2), 0.5);
  segment.a3 = scale(
    add(add(scale(q0, -1.0), scale(q1, 3.0)), add(scale(q2, -3.0), q3)), 1.0 / 6.0);
  return segment;
}

Vec3 positionAt(const CubicSegment & segment, double s)
{
  return add(
    add(segment.a0, scale(segment.a1, s)),
    add(scale(segment.a2, s * s), scale(segment.a3, s * s * s)));
}

Vec3 derivativeAt(const CubicSegment & segment, double s)
{
  return add(segment.a1, add(scale(segment.a2, 2.0 * s), scale(segment.a3, 3.0 * s * s)));
}

Vec3 secondDerivativeAt(const CubicSegment & segment, double s)
{
  return add(scale(segment.a2, 2.0), scale(segment.a3, 6.0 * s));
}

bool isPositiveFinite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool validateLimits(const TrajectoryLimits & limits, std::string & reason)
{
  if (!isPositiveFinite(limits.max_speed)) {
    reason = "max_speed must be positive and finite";
    return false;
  }
  if (!isPositiveFinite(limits.max_vertical_speed)) {
    reason = "max_vertical_speed must be positive and finite";
    return false;
  }
  if (!isPositiveFinite(limits.max_acceleration)) {
    reason = "max_acceleration must be positive and finite";
    return false;
  }
  if (!isPositiveFinite(limits.max_yaw_rate)) {
    reason = "max_yaw_rate must be positive and finite";
    return false;
  }
  if (!std::isfinite(limits.yaw_deadband_speed) || limits.yaw_deadband_speed < 0.0) {
    reason = "yaw_deadband_speed must be finite and non-negative";
    return false;
  }
  if (!isPositiveFinite(limits.max_duration)) {
    reason = "max_duration must be positive and finite";
    return false;
  }
  // Below twice the 20 Hz stream the consumer interpolates across a real peak.
  if (!isPositiveFinite(limits.sample_hz) ||
    limits.sample_hz < kMinSampleHz || limits.sample_hz > kMaxSampleHz)
  {
    reason = "sample_hz must be between 40 and 1000";
    return false;
  }
  return true;
}

std::vector<Vec3> sanitize(const std::vector<Vec3> & waypoints, std::string & reason)
{
  std::vector<Vec3> clean;
  for (const Vec3 & waypoint : waypoints) {
    if (!isFinite(waypoint)) {
      reason = "a waypoint is not finite";
      return {};
    }
    if (!clean.empty() && distance3(clean.back(), waypoint) < kDuplicateWaypointDistance) {
      continue;
    }
    clean.push_back(waypoint);
  }
  if (clean.empty()) {
    reason = "no waypoints";
  }
  return clean;
}

/// Triple endpoints: the curve then starts and ends exactly on them, at rest.
std::vector<Vec3> clampedControlPoints(const std::vector<Vec3> & waypoints)
{
  std::vector<Vec3> control;
  control.reserve(waypoints.size() + 4);
  control.push_back(waypoints.front());
  control.push_back(waypoints.front());
  for (const Vec3 & waypoint : waypoints) {
    control.push_back(waypoint);
  }
  control.push_back(waypoints.back());
  control.push_back(waypoints.back());
  return control;
}

struct ParametricPeaks
{
  double speed = 0.0;
  double vertical_speed = 0.0;
  double acceleration = 0.0;
};

ParametricPeaks scanPeaks(const std::vector<CubicSegment> & segments)
{
  ParametricPeaks peaks;
  for (const CubicSegment & segment : segments) {
    for (int step = 0; step <= kPeakScanSteps; ++step) {
      const double s = static_cast<double>(step) / kPeakScanSteps;
      const Vec3 first = derivativeAt(segment, s);
      peaks.speed = std::max(peaks.speed, norm(first));
      peaks.vertical_speed = std::max(peaks.vertical_speed, std::abs(first.z));
      peaks.acceleration = std::max(peaks.acceleration, norm(secondDerivativeAt(segment, s)));
    }
  }
  return peaks;
}

/// Every segment carries the same duration, so time has ONE scale across the
/// whole spline. Per-segment durations make dP/dt jump at every interior join.
std::vector<TrajectorySample> sampleGrid(
  const std::vector<CubicSegment> & segments, double duration, double period,
  double initial_yaw, const TrajectoryLimits & limits)
{
  const std::size_t segment_count = segments.size();
  const double segment_duration = duration / static_cast<double>(segment_count);
  const std::size_t node_count =
    static_cast<std::size_t>(std::llround(duration / period)) + 1;

  std::vector<TrajectorySample> grid;
  grid.reserve(node_count);

  double yaw = wrapAngle(initial_yaw);
  for (std::size_t node = 0; node < node_count; ++node) {
    const double time = std::min(static_cast<double>(node) * period, duration);
    const double position_in_spline = time / segment_duration;
    const std::size_t index =
      std::min(segment_count - 1, static_cast<std::size_t>(position_in_spline));
    const double s = std::min(1.0, position_in_spline - static_cast<double>(index));

    TrajectorySample sample;
    sample.position = positionAt(segments[index], s);
    sample.velocity = scale(derivativeAt(segments[index], s), 1.0 / segment_duration);
    sample.acceleration =
      scale(secondDerivativeAt(segments[index], s), 1.0 / (segment_duration * segment_duration));

    const double travel_speed = std::hypot(sample.velocity.x, sample.velocity.y);
    if (travel_speed >= limits.yaw_deadband_speed) {
      const double heading = std::atan2(sample.velocity.y, sample.velocity.x);
      yaw = stepYaw(yaw, heading, limits.max_yaw_rate, period);
    }
    sample.yaw = yaw;
    sample.yaw_rate =
      grid.empty() ? 0.0 : wrapAngle(yaw - grid.back().yaw) / period;
    grid.push_back(sample);
  }
  return grid;
}

/// How far the nose falls behind the path while the rate limit slews it round.
/// Reported rather than designed away: slowing the whole flight to keep the nose
/// on the path makes an ordinary GotoPose outrun its timeout.
double worstYawLag(const std::vector<TrajectorySample> & grid, double deadband_speed)
{
  double worst = 0.0;
  for (const TrajectorySample & sample : grid) {
    const double travel_speed = std::hypot(sample.velocity.x, sample.velocity.y);
    if (travel_speed < std::max(deadband_speed, 1e-6)) {
      continue;
    }
    const double heading = std::atan2(sample.velocity.y, sample.velocity.x);
    worst = std::max(worst, std::abs(wrapAngle(heading - sample.yaw)));
  }
  return worst;
}

}  // namespace

Trajectory Trajectory::build(
  const std::vector<Vec3> & waypoints, double initial_yaw, const TrajectoryLimits & limits,
  std::string & reason)
{
  Trajectory trajectory;
  reason.clear();

  if (!validateLimits(limits, reason)) {
    return trajectory;
  }
  trajectory.grid_period_ = 1.0 / limits.sample_hz;

  const std::vector<Vec3> clean = sanitize(waypoints, reason);
  if (clean.empty()) {
    return trajectory;
  }

  if (clean.size() == 1) {
    TrajectorySample hold;
    hold.position = clean.front();
    hold.yaw = wrapAngle(initial_yaw);
    trajectory.grid_.push_back(hold);
    trajectory.valid_ = true;
    return trajectory;
  }

  const std::vector<Vec3> control = clampedControlPoints(clean);
  std::vector<CubicSegment> segments;
  segments.reserve(control.size() - 3);
  for (std::size_t index = 0; index + 3 < control.size(); ++index) {
    segments.push_back(
      makeSegment(control[index], control[index + 1], control[index + 2], control[index + 3]));
  }

  const ParametricPeaks peaks = scanPeaks(segments);
  if (!(peaks.speed > 0.0)) {
    reason = "waypoints collapse to a single point";
    return trajectory;
  }

  // Stretching by k divides speed by k and acceleration by k*k, so the duration
  // that satisfies every limit is a closed form, not a search.
  const double count = static_cast<double>(segments.size());
  double duration = count * peaks.speed / limits.max_speed;
  duration = std::max(duration, count * peaks.vertical_speed / limits.max_vertical_speed);
  duration = std::max(duration, count * std::sqrt(peaks.acceleration / limits.max_acceleration));

  const double period = trajectory.grid_period_;
  if (!std::isfinite(duration) || duration > limits.max_duration) {
    reason = "trajectory would take longer than max_duration";
    return trajectory;
  }
  const std::size_t node_count =
    static_cast<std::size_t>(std::llround(std::ceil(duration / period))) + 1;
  if (node_count + 1 > kMaxGridPoints) {
    reason = "trajectory needs more samples than the grid allows";
    return trajectory;
  }
  // sample() walks an evenly spaced grid, so the duration must be a whole number
  // of periods; rounding up only ever makes the flight slower.
  const double snapped = static_cast<double>(node_count - 1) * period;
  std::vector<TrajectorySample> grid = sampleGrid(segments, snapped, period, initial_yaw, limits);

  if (grid.size() < 2) {
    reason = "trajectory collapsed to a single sample";
    return trajectory;
  }
  for (const TrajectorySample & sample : grid) {
    if (!isFinite(sample.position) || !isFinite(sample.velocity) ||
      !isFinite(sample.acceleration) || !std::isfinite(sample.yaw) ||
      !std::isfinite(sample.yaw_rate))
    {
      reason = "waypoints are too large to be represented";
      return trajectory;
    }
  }

  const double segment_duration = snapped / count;
  trajectory.peak_yaw_lag_ = worstYawLag(grid, limits.yaw_deadband_speed);
  trajectory.grid_ = std::move(grid);
  trajectory.duration_ = snapped;
  trajectory.peak_speed_ = peaks.speed / segment_duration;
  trajectory.peak_vertical_speed_ = peaks.vertical_speed / segment_duration;
  trajectory.peak_acceleration_ = peaks.acceleration / (segment_duration * segment_duration);
  trajectory.valid_ = true;
  return trajectory;
}

TrajectorySample Trajectory::sample(double time) const
{
  if (grid_.empty()) {
    return TrajectorySample{};
  }
  if (grid_.size() < 2 || !std::isfinite(time) || time <= 0.0) {
    TrajectorySample start = grid_.front();
    start.velocity = Vec3{};
    start.acceleration = Vec3{};
    start.yaw_rate = 0.0;
    return start;
  }
  if (time >= duration_) {
    TrajectorySample end = grid_.back();
    end.velocity = Vec3{};
    end.acceleration = Vec3{};
    end.yaw_rate = 0.0;
    return end;
  }

  const double position = time / grid_period_;
  const std::size_t index =
    std::min(grid_.size() - 2, static_cast<std::size_t>(position));
  const double fraction = std::min(1.0, std::max(0.0, position - static_cast<double>(index)));
  const TrajectorySample & before = grid_[index];
  const TrajectorySample & after = grid_[index + 1];

  TrajectorySample blended;
  blended.position = lerp(before.position, after.position, fraction);
  blended.velocity = lerp(before.velocity, after.velocity, fraction);
  blended.acceleration = lerp(before.acceleration, after.acceleration, fraction);
  blended.yaw = wrapAngle(before.yaw + wrapAngle(after.yaw - before.yaw) * fraction);
  blended.yaw_rate = before.yaw_rate + (after.yaw_rate - before.yaw_rate) * fraction;
  return blended;
}

Vec3 Trajectory::finalPosition() const
{
  return grid_.empty() ? Vec3{} : grid_.back().position;
}

}  // namespace uav_navigation
