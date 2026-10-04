#include "uav_navigation/costmap.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace uav_navigation
{

namespace
{
constexpr unsigned kMaxCellsPerSide = 4096u;

/// Highest soft cost, kept below kCostInscribed so the ordering never collapses.
constexpr double kSoftCeiling = 252.0;

/// Beyond this the exponential falls under 1 and would round to FREE anyway.
constexpr double kSoftDecayNumerator = 253.0;

double clampToZero(double value)
{
  return value > 0.0 ? value : 0.0;
}

/// Distance from a point to the obstacle footprint; 0 when the point is inside it.
double footprintDistance(const CostmapObstacle & obstacle, double x, double y)
{
  const double outside_x = clampToZero(std::abs(x - obstacle.center.x) - 0.5 * obstacle.size.x);
  const double outside_y = clampToZero(std::abs(y - obstacle.center.y) - 0.5 * obstacle.size.y);
  return std::hypot(outside_x, outside_y);
}

bool nonNegativeFinite(double value)
{
  return std::isfinite(value) && value >= 0.0;
}
}  // namespace

Costmap Costmap::build(const CostmapLimits & limits, std::string & reason)
{
  Costmap map;
  reason.clear();

  if (!std::isfinite(limits.resolution_m) || !(limits.resolution_m > 0.0)) {
    reason = "resolution_m must be positive and finite";
    return map;
  }
  if (!std::isfinite(limits.width_m) || !(limits.width_m > 0.0)) {
    reason = "width_m must be positive and finite";
    return map;
  }
  if (!std::isfinite(limits.cost_scaling) || !(limits.cost_scaling > 0.0)) {
    reason = "cost_scaling must be positive and finite";
    return map;
  }
  if (!nonNegativeFinite(limits.drone_radius_m)) {
    reason = "drone_radius_m must be non-negative and finite";
    return map;
  }
  if (!nonNegativeFinite(limits.corner_cut_m)) {
    reason = "corner_cut_m must be non-negative and finite";
    return map;
  }
  if (!nonNegativeFinite(limits.obstacle_timeout_sec)) {
    reason = "obstacle_timeout_sec must be non-negative and finite";
    return map;
  }
  if (!nonNegativeFinite(limits.flight_band_m)) {
    reason = "flight_band_m must be non-negative and finite";
    return map;
  }

  const double cells = std::round(limits.width_m / limits.resolution_m);
  if (!(cells >= 1.0) || cells > static_cast<double>(kMaxCellsPerSide)) {
    reason = "width_m / resolution_m yields an unusable cell count";
    return map;
  }

  map.limits_ = limits;
  map.cells_per_side_ = static_cast<unsigned>(cells);
  const std::size_t side = static_cast<std::size_t>(map.cells_per_side_);
  map.cells_.assign(side * side, kCostFree);
  map.valid_ = true;
  return map;
}

double Costmap::inflationRadius(const CostmapObstacle & obstacle) const
{
  const double uncertainty = clampToZero(obstacle.position_uncertainty);
  // Corner cut is spent here because the spline flies inside the planned corner.
  return limits_.drone_radius_m + uncertainty + limits_.corner_cut_m;
}

void Costmap::update(
  const std::vector<CostmapObstacle> & obstacles, const Vec3 & center,
  double now_seconds, double feed_stamp_seconds, std::string & reason)
{
  reason.clear();
  written_ = 0;
  rejected_ = 0;

  if (!valid_) {
    reason = "costmap was never built";
    return;
  }
  if (!isFinite(center) || !std::isfinite(now_seconds)) {
    reason = "update needs a finite center and time";
    rejected_ = static_cast<unsigned>(obstacles.size());
    last_input_age_ = std::numeric_limits<double>::infinity();
    return;
  }

  std::fill(cells_.begin(), cells_.end(), kCostFree);

  const double resolution = limits_.resolution_m;
  const double span = static_cast<double>(cells_per_side_) * resolution;
  // Snapped to the grid: an unsnapped window resamples its own cells as it slides.
  origin_.x = std::floor((center.x - 0.5 * span) / resolution) * resolution;
  origin_.y = std::floor((center.y - 0.5 * span) / resolution) * resolution;
  origin_.z = center.z;

  const double band_low = center.z - limits_.flight_band_m;
  const double band_high = center.z + limits_.flight_band_m;
  const double soft_span = std::log(kSoftDecayNumerator) / limits_.cost_scaling;
  const int last_cell = static_cast<int>(cells_per_side_) - 1;

  // A report the map could not trust must not count as freshness: saying "fresh" over
  // an empty map reads as clear sky. Only the flight-band filter leaves trust intact --
  // it drops obstacles that are real but irrelevant, which is the map working, not failing.
  bool report_trustworthy = true;

  for (const CostmapObstacle & obstacle : obstacles) {
    if (!isFinite(obstacle.center) || !isFinite(obstacle.size) ||
      !std::isfinite(obstacle.position_uncertainty) || !std::isfinite(obstacle.stamp_seconds))
    {
      ++rejected_;
      report_trustworthy = false;
      if (reason.empty()) {
        reason = "obstacle carries a non-finite field";
      }
      continue;
    }
    if (obstacle.position_uncertainty < 0.0) {
      ++rejected_;
      report_trustworthy = false;
      if (reason.empty()) {
        reason = "obstacle reports no measured uncertainty";
      }
      continue;
    }
    if (obstacle.size.x < 0.0 || obstacle.size.y < 0.0 || obstacle.size.z < 0.0) {
      ++rejected_;
      report_trustworthy = false;
      if (reason.empty()) {
        reason = "obstacle reports a negative extent";
      }
      continue;
    }
    if (now_seconds - obstacle.stamp_seconds > limits_.obstacle_timeout_sec) {
      ++rejected_;
      report_trustworthy = false;
      if (reason.empty()) {
        reason = "obstacle is older than obstacle_timeout_sec";
      }
      continue;
    }

    // Uncertainty widens the slab too: a doubtful obstacle may still reach the band.
    const double reach_z = 0.5 * obstacle.size.z + obstacle.position_uncertainty;
    if (obstacle.center.z + reach_z < band_low || obstacle.center.z - reach_z > band_high) {
      ++rejected_;
      if (reason.empty()) {
        reason = "obstacle sits outside the flight band";
      }
      continue;
    }

    ++written_;

    const double inflation = inflationRadius(obstacle);
    const double reach = inflation + soft_span;
    const double min_x = obstacle.center.x - 0.5 * obstacle.size.x - reach;
    const double max_x = obstacle.center.x + 0.5 * obstacle.size.x + reach;
    const double min_y = obstacle.center.y - 0.5 * obstacle.size.y - reach;
    const double max_y = obstacle.center.y + 0.5 * obstacle.size.y + reach;

    const double low_x = std::floor((min_x - origin_.x) / resolution);
    const double high_x = std::floor((max_x - origin_.x) / resolution);
    const double low_y = std::floor((min_y - origin_.y) / resolution);
    const double high_y = std::floor((max_y - origin_.y) / resolution);
    if (high_x < 0.0 || high_y < 0.0 ||
      low_x > static_cast<double>(last_cell) || low_y > static_cast<double>(last_cell))
    {
      continue;
    }

    const int first_x = low_x < 0.0 ? 0 : static_cast<int>(low_x);
    const int first_y = low_y < 0.0 ? 0 : static_cast<int>(low_y);
    const int final_x = high_x > static_cast<double>(last_cell) ? last_cell :
      static_cast<int>(high_x);
    const int final_y = high_y > static_cast<double>(last_cell) ? last_cell :
      static_cast<int>(high_y);

    for (int cell_y = first_y; cell_y <= final_y; ++cell_y) {
      for (int cell_x = first_x; cell_x <= final_x; ++cell_x) {
        const Vec3 point = cellToWorld(cell_x, cell_y);
        const double range = footprintDistance(obstacle, point.x, point.y);

        uint8_t value = kCostFree;
        if (range <= 0.0) {
          value = kCostLethal;
        } else if (range <= inflation) {
          value = kCostInscribed;
        } else {
          const double soft =
            1.0 + kSoftDecayNumerator * std::exp(-limits_.cost_scaling * (range - inflation));
          if (soft < 1.0) {
            continue;
          }
          value = static_cast<uint8_t>(soft > kSoftCeiling ? kSoftCeiling : soft);
        }

        uint8_t & cell = cells_[
          static_cast<std::size_t>(cell_y) * cells_per_side_ + static_cast<std::size_t>(cell_x)];
        if (value > cell) {
          cell = value;
        }
      }
    }
  }

  // An empty report is still a report, and so is one whose obstacles all sat outside
  // the flight band. Before 2026-08-26 freshness came from the newest obstacle WRITTEN,
  // so both of those read exactly like a dead perception: age inf, "no obstacle input
  // has ever arrived", while the sensor was healthy and saying "clear".
  if (report_trustworthy && std::isfinite(feed_stamp_seconds) &&
    feed_stamp_seconds > last_input_stamp_)
  {
    last_input_stamp_ = feed_stamp_seconds;
  }

  last_input_age_ = now_seconds - last_input_stamp_;
  // A stamp from the future is a broken clock, not freshness.
  if (!(last_input_age_ >= 0.0)) {
    last_input_age_ = std::numeric_limits<double>::infinity();
  }
}

bool Costmap::inside(int cell_x, int cell_y) const
{
  if (!valid_) {
    return false;
  }
  const int side = static_cast<int>(cells_per_side_);
  return cell_x >= 0 && cell_y >= 0 && cell_x < side && cell_y < side;
}

uint8_t Costmap::cost(int cell_x, int cell_y) const
{
  if (!inside(cell_x, cell_y)) {
    return kCostUnknown;
  }
  return cells_[
    static_cast<std::size_t>(cell_y) * cells_per_side_ + static_cast<std::size_t>(cell_x)];
}

uint8_t Costmap::costAt(const Vec3 & point) const
{
  int cell_x = 0;
  int cell_y = 0;
  if (!worldToCell(point, cell_x, cell_y)) {
    return kCostUnknown;
  }
  return cost(cell_x, cell_y);
}

bool Costmap::worldToCell(const Vec3 & point, int & cell_x, int & cell_y) const
{
  if (!valid_ || !isFinite(point)) {
    return false;
  }
  const double column = std::floor((point.x - origin_.x) / limits_.resolution_m);
  const double row = std::floor((point.y - origin_.y) / limits_.resolution_m);
  const double side = static_cast<double>(cells_per_side_);
  if (column < 0.0 || row < 0.0 || column >= side || row >= side) {
    return false;
  }
  cell_x = static_cast<int>(column);
  cell_y = static_cast<int>(row);
  return true;
}

Vec3 Costmap::cellToWorld(int cell_x, int cell_y) const
{
  Vec3 point = origin_;
  point.x = origin_.x + (static_cast<double>(cell_x) + 0.5) * limits_.resolution_m;
  point.y = origin_.y + (static_cast<double>(cell_y) + 0.5) * limits_.resolution_m;
  return point;
}

}  // namespace uav_navigation
