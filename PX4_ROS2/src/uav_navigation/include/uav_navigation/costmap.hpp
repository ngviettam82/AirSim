#ifndef UAV_NAVIGATION__COSTMAP_HPP_
#define UAV_NAVIGATION__COSTMAP_HPP_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "uav_navigation/carrot.hpp"

namespace uav_navigation
{

/// Nav2 cost semantics carried by value, so no Nav2 dependency comes with them.
constexpr uint8_t kCostFree = 0;
constexpr uint8_t kCostInscribed = 253;
constexpr uint8_t kCostLethal = 254;
constexpr uint8_t kCostUnknown = 255;

struct CostmapLimits
{
  double width_m = 25.0;
  double resolution_m = 0.25;

  /// Radius of the circle that circumscribes the airframe.
  double drone_radius_m = 0.35;

  /// Exponential falloff of the soft cost, in 1/m.
  double cost_scaling = 1.0;

  /// Optional extra keep-out for the corner the spline cuts. Measured 2026-08-18:
  /// the cut is 0.44-0.59 m, but in open ground the soft cost already leaves a 2 m
  /// cushion, and a fixed 0.6 m here closes every gap under ~1.9 m. Left at zero on
  /// purpose -- tightenForSpline() buys the same guarantee without the corridor.
  double corner_cut_m = 0.0;

  /// Past this age an obstacle is no longer written. Silence is not free space:
  /// ask lastInputAge() before reading an empty window as open air.
  double obstacle_timeout_sec = 3.0;

  /// Half-height of the horizontal slice we plan in (decision 3: 2.5D).
  double flight_band_m = 1.0;
};

struct CostmapObstacle
{
  Vec3 center;

  /// FULL extent, per interface contract v0.1: a 1x1x2 m box reports (1,1,2).
  Vec3 size;

  /// 1-sigma metres. Negative means "not measured", which /world/* forbids;
  /// the map refuses such an obstacle instead of reading it as certain.
  double position_uncertainty = 0.0;

  double stamp_seconds = 0.0;
};

/// Rolling 2D window of obstacle cost around the aircraft.
///
/// Two invariants carry the safety weight, and both exist because a planner that
/// reads this map decides where the aircraft flies:
///   1. An empty window and a missing window are DIFFERENT. This class never
///      reports the difference as free space -- lastInputAge() is the only
///      honest answer, and a consumer that ignores it will fly into a stale map.
///   2. Every number that enters is checked finite. std::max swallows NaN, and
///      one NaN in an inflation radius silently deletes an obstacle.
class Costmap
{
public:
  Costmap() = default;

  static Costmap build(const CostmapLimits & limits, std::string & reason);

  bool valid() const {return valid_;}
  unsigned cellsPerSide() const {return cells_per_side_;}
  double resolution() const {return limits_.resolution_m;}

  /// Half-height of the slice these cells describe. A consumer that moves the
  /// aircraft further than this in z has left the ground this map can vouch for.
  double flightBand() const {return limits_.flight_band_m;}

  /// World position of the low corner of cell (0, 0).
  Vec3 origin() const {return origin_;}

  /// Rewrites the whole window around center. Obstacles that are stale, not
  /// finite, outside the flight band, or missing an uncertainty are skipped and
  /// counted in rejected(); reason names the first one so a gate can quote it.
  ///
  /// feed_stamp_seconds is WHEN THE REPORT ARRIVED, and it is deliberately a
  /// separate argument from the obstacles it carried. Freshness used to be
  /// inferred from the newest obstacle actually WRITTEN, which made a healthy
  /// perception saying "clear sky" read exactly like a dead one: an empty array,
  /// or two perfectly fresh obstacles that both sat outside the flight band, both
  /// left the map reporting age inf and "no obstacle input has ever arrived"
  /// (measured 2026-08-26). Pass a non-finite value only when no report has ever
  /// arrived.
  void update(
    const std::vector<CostmapObstacle> & obstacles, const Vec3 & center,
    double now_seconds, double feed_stamp_seconds, std::string & reason);

  uint8_t cost(int cell_x, int cell_y) const;
  uint8_t costAt(const Vec3 & point) const;

  bool inside(int cell_x, int cell_y) const;
  bool worldToCell(const Vec3 & point, int & cell_x, int & cell_y) const;

  /// Center of the cell, which is what a planner should route through.
  Vec3 cellToWorld(int cell_x, int cell_y) const;

  /// Distance from the obstacle surface inside which the aircraft cannot fit.
  double inflationRadius(const CostmapObstacle & obstacle) const;

  /// Seconds between the newest obstacle REPORT and the time of that update.
  /// Infinite while no report has ever arrived. An empty report is still a
  /// report: it says the sensor looked and saw nothing.
  double lastInputAge() const {return last_input_age_;}

  /// When the newest report was stamped. A caller that skips update() must age
  /// the map from this against its own clock; lastInputAge() would stand still.
  double lastInputStamp() const {return last_input_stamp_;}

  unsigned written() const {return written_;}
  unsigned rejected() const {return rejected_;}

private:
  bool valid_ = false;
  CostmapLimits limits_;
  unsigned cells_per_side_ = 0;
  Vec3 origin_;
  /// -inf until the first report arrives, so an untouched map reports age inf.
  double last_input_stamp_ = -std::numeric_limits<double>::infinity();
  double last_input_age_ = std::numeric_limits<double>::infinity();
  unsigned written_ = 0;
  unsigned rejected_ = 0;
  std::vector<uint8_t> cells_;
};

}  // namespace uav_navigation

#endif  // UAV_NAVIGATION__COSTMAP_HPP_
