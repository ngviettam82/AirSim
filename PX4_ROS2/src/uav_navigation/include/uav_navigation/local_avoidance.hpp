#ifndef UAV_NAVIGATION__LOCAL_AVOIDANCE_HPP_
#define UAV_NAVIGATION__LOCAL_AVOIDANCE_HPP_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "uav_navigation/costmap.hpp"

namespace uav_navigation
{

struct AvoidanceLimits
{
  /// How far along the route ahead to look. Reported back as checked_horizon_m.
  double horizon_m = 6.0;

  /// Spacing of the collision samples. Below one cell so a corner is not stepped over.
  double sample_step_m = 0.1;

  /// Clearance search radius. The reported clearance SATURATES here: a value equal
  /// to this means "at least this much", never "exactly this much".
  double clearance_scan_m = 2.0;

  /// Archimedes growth term; a = V/2 in the source algorithm.
  double spiral_growth_m = 0.5;

  /// 2.5D guard (decision 3): an escape point further than this from the obstacle
  /// in z is refused outright.
  double max_escape_climb_m = 2.0;

  /// Refuses an escape that ducks BELOW the obstacle being avoided: low flight
  /// meets more obstacles, not fewer. Measured against the obstacle rather than
  /// the aircraft because in a 2.5D slice there is always climb room beside it,
  /// so an aircraft-relative test would never fire.
  bool allow_descent = false;

  /// Past this age the map cannot be advised on at all.
  double map_timeout_sec = 3.0;

  int max_spiral_steps = 24;
};

enum class Advice : uint8_t
{
  Clear = 0,
  Escape = 1,
  Hold = 2,
};

struct AvoidanceResult
{
  Advice advice = Advice::Hold;

  /// NaN unless advice is Escape. Never (0,0,0): in odom that is the launch point.
  Vec3 escape_point{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  double clearance_m = 0.0;

  /// How far the check actually reached before the horizon or the map window ended.
  /// A clearance without this number cannot be read.
  double checked_horizon_m = 0.0;

  bool map_fresh = false;
  double map_age_sec = std::numeric_limits<double>::infinity();

  /// Always filled, including on Clear.
  std::string reason;
};

/// Reactive layer that ADVISES; it never commands.
///
/// Three things carry the safety weight here:
///   1. It answers Hold when it cannot see, and never dresses "no map" up as Clear.
///      The permissive reading belongs to the node that owns require_obstacle_feed,
///      in one place, out loud -- not to this class.
///   2. Cost 255 means "outside the rolling window", NOT "blocked". Reading it as an
///      obstacle would make every long route look walled off; it bounds
///      checked_horizon_m instead.
///   3. An escape point is refused unless BOTH legs around it are clear as far as
///      the map can see, it never ducks under the obstacle, and its climb stays
///      inside the flight band -- past that band the cells describe nothing.
class LocalAvoidance
{
public:
  LocalAvoidance() = default;

  static LocalAvoidance build(const AvoidanceLimits & limits, std::string & reason);

  bool valid() const {return valid_;}
  const AvoidanceLimits & limits() const {return limits_;}

  /// route_ahead starts at or near the aircraft and runs toward the goal; the caller
  /// owns the trimming. position is the aircraft now.
  AvoidanceResult advise(
    const Costmap & map, const std::vector<Vec3> & route_ahead, const Vec3 & position) const;

private:
  bool blocked(const Costmap & map, const Vec3 & point) const;

  /// Clear as far as the map can see: samples outside the window end the walk
  /// rather than condemning the segment.
  bool segmentClear(const Costmap & map, const Vec3 & from, const Vec3 & to) const;

  double clearanceAt(const Costmap & map, const Vec3 & point) const;

  bool findEscape(
    const Costmap & map, const Vec3 & obstacle, const Vec3 & position,
    const Vec3 & goal, Vec3 & escape, std::string & reason) const;

  bool valid_ = false;
  AvoidanceLimits limits_;
};

}  // namespace uav_navigation

#endif  // UAV_NAVIGATION__LOCAL_AVOIDANCE_HPP_
