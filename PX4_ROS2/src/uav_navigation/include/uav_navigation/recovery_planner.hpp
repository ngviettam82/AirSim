#ifndef UAV_NAVIGATION__RECOVERY_PLANNER_HPP_
#define UAV_NAVIGATION__RECOVERY_PLANNER_HPP_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "uav_navigation/carrot.hpp"

namespace uav_navigation
{

/// Three types only, and the CALLER picks one.
///
/// There is deliberately no LAND. With localisation lost the aircraft does not know
/// where it is, so an "automatic landing" lands on an unknown spot -- see
/// docs/interface-contract-v0.1.md section 2.5. Deciding WHEN to recover, and which
/// type, belongs to the safety layer; this library only executes what it is told.
///
/// The values happen to line up with the action constants, but nothing here relies
/// on that: the node must switch on the goal explicitly so LAND and
/// HANDOVER_TO_PILOT are refused where they are read, not silently cast.
enum class RecoveryType : uint8_t
{
  Unknown = 0,
  Hover = 1,
  ClimbToSafe = 2,
  ReturnHome = 3,
};

const char * recoveryTypeName(RecoveryType type);

struct RecoveryLimits
{
  /// Smallest move worth flying. Below this the request sits inside position noise,
  /// so it is refused rather than turned into a hover the caller did not ask for.
  double min_travel_m = 0.3;

  /// Cap on ONE recovery climb, measured from the aircraft.
  double max_climb_m = 15.0;

  /// Absolute ceiling any recovery may reach.
  double max_safe_altitude_m = 30.0;

  /// Vertical room kept above home. A return that ends at or below the home point is
  /// a landing without authority, which this planner never issues.
  double min_home_clearance_m = 1.0;

  /// Horizontal range a return may cover. Further than this is a cross-country leg,
  /// not a recovery, and nothing here has checked the ground in between.
  /// The default is bounded by the trajectory core: 30 m with a climb leg already
  /// costs 164 s of the 192 s the node allows. Raising it without raising that
  /// budget only produces plans the core refuses.
  double max_home_distance_m = 25.0;
};

struct RecoveryRequest
{
  RecoveryType type = RecoveryType::Unknown;

  /// Read by ClimbToSafe (the apex) and by ReturnHome (the cruise altitude).
  /// Ignored by Hover. Never defaulted: the caller states it or the plan is refused.
  double safe_altitude_m = std::numeric_limits<double>::quiet_NaN();
};

struct RecoveryState
{
  /// Where the aircraft is being commanded from now -- the motion anchor, so the
  /// first waypoint continues the stream instead of stepping it.
  Vec3 position;

  /// Only read when home_known is true.
  Vec3 home;

  /// False until a home has actually arrived. In odom, (0,0,0) is the launch point:
  /// a plausible-looking coordinate, which is exactly why it must not be assumed.
  bool home_known = false;
};

struct RecoveryPlan
{
  bool accepted = false;
  RecoveryType type = RecoveryType::Unknown;

  /// Empty for Hover, and empty on every refusal. Otherwise waypoints.front() is the
  /// aircraft, to be handed to the SAME Trajectory core that flies GotoPose.
  std::vector<Vec3> waypoints;

  /// Where the aircraft ends up and holds. NaN unless accepted; never (0,0,0),
  /// which in odom is the launch point.
  Vec3 hold_point{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  double climb_m = 0.0;
  double home_distance_m = 0.0;

  /// Always filled, including on accept.
  std::string reason;
};

/// Turns a recovery request into waypoints. ROS-free, and it produces geometry only.
///
/// Three things carry the safety weight here:
///   1. It never invents a home. No home received means the return is refused by
///      name, because (0,0,0) in odom is the launch point and would look fine.
///   2. It never descends and never lands. ReturnHome ends HOVERING above home;
///      coming down is a separate, authorised act.
///   3. It refuses instead of clamping, and it never substitutes a type. A request
///      that cannot be flown as asked comes back refused with a reason; asking for
///      Hover instead is the caller's decision, not this class's.
class RecoveryPlanner
{
public:
  RecoveryPlanner() = default;

  static RecoveryPlanner build(const RecoveryLimits & limits, std::string & reason);

  bool valid() const {return valid_;}
  const RecoveryLimits & limits() const {return limits_;}

  RecoveryPlan plan(const RecoveryRequest & request, const RecoveryState & state) const;

private:
  RecoveryPlan planHover(const RecoveryState & state) const;
  RecoveryPlan planClimb(const RecoveryRequest & request, const RecoveryState & state) const;
  RecoveryPlan planReturnHome(const RecoveryRequest & request, const RecoveryState & state) const;

  bool valid_ = false;
  RecoveryLimits limits_;
};

}  // namespace uav_navigation

#endif  // UAV_NAVIGATION__RECOVERY_PLANNER_HPP_
