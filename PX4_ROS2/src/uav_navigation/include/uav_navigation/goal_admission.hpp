#ifndef UAV_NAVIGATION__GOAL_ADMISSION_HPP_
#define UAV_NAVIGATION__GOAL_ADMISSION_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "uav_navigation/carrot.hpp"

namespace uav_navigation
{

enum class NavigatorState
{
  IDLE,             // on the ground, streaming nothing
  TAKING_OFF,
  GOING_TO_POSE,
  HOLD_TASK,        // a HoldPosition goal is running
  FOLLOWING_PATH,
  TRACKING_TARGET,
  RECOVERING,       // a Recover goal took the aircraft over
  HOLDING,          // hovering with no task, ready for the next goal
  LANDING,
};

enum class TaskType
{
  TAKEOFF,
  GOTO_POSE,
  HOLD_POSITION,
  LAND,
  FOLLOW_PATH,
  TRACK_TARGET,
  RECOVER,
};

enum class RejectReason
{
  NONE,
  BUSY,
  NOT_AIRBORNE,
  ALREADY_AIRBORNE,
  INVALID_FRAME,
  NOT_FINITE,
  OUT_OF_RANGE,
  UNSUPPORTED,
};

const char * toString(NavigatorState state);
const char * toString(TaskType task);
const char * toString(RejectReason reason);

/// Mutual exclusion: one task at a time, and only from a settled state.
bool acceptsNewGoal(NavigatorState state);

struct MotionLimits
{
  double max_speed = 0.55;
  double max_vertical_speed = 0.45;
  double acceptance_radius = 0.3;
  double max_takeoff_altitude = 10.0;
  double min_altitude = 0.5;
  double max_altitude = 20.0;
};

struct Admission
{
  bool accepted = false;
  RejectReason reason = RejectReason::NONE;
  std::string detail;
};

struct TakeoffRequest
{
  double target_altitude = 0.0;
};

struct GotoRequest
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  std::string frame_id;
  double acceptance_radius = 0.0;
  double max_speed = 0.0;
};

struct HoldRequest
{
  double duration_seconds = 0.0;
};

struct LandRequest
{
  bool use_precision_landing = false;
};

struct FollowPathRequest
{
  std::vector<Vec3> waypoints;
  std::string frame_id;
  bool path_is_valid = false;
  double max_speed = 0.0;
  double acceptance_radius = 0.0;
};

struct TrackRequest
{
  double standoff_distance = 0.0;
  double duration_seconds = 0.0;
  double target_lost_timeout = 0.0;
};

struct RecoverRequest
{
  /// False for LAND, HANDOVER_TO_PILOT and UNKNOWN. Deciding to land or to give the
  /// aircraft back is the safety layer's call; this node only flies motion.
  bool type_executable = false;
  std::string refusal;
  bool reads_altitude = false;
  double safe_altitude = 0.0;
};

Admission admitTakeoff(
  NavigatorState state, const TakeoffRequest & request, const MotionLimits & limits);

Admission admitGoto(
  NavigatorState state, const GotoRequest & request, const MotionLimits & limits,
  const std::string & odom_frame);

Admission admitHold(NavigatorState state, const HoldRequest & request);

Admission admitLand(NavigatorState state, const LandRequest & request);

Admission admitFollowPath(
  NavigatorState state, const FollowPathRequest & request, const MotionLimits & limits,
  const std::string & odom_frame);

Admission admitTrackTarget(
  NavigatorState state, const TrackRequest & request, const MotionLimits & limits);

/// The one goal that PREEMPTS: a recovery arriving mid-flight takes the aircraft
/// over instead of being refused as busy. Only landing is left alone, because the
/// autopilot already owns the aircraft by then.
Admission admitRecover(NavigatorState state, const RecoverRequest & request);

/// Waypoint count a single spline is allowed to carry; beyond this a caller should
/// split the mission instead of handing the core an unplannable path.
constexpr std::size_t kMaxPathWaypoints = 200;

struct GotoLimits
{
  double acceptance_radius = 0.3;
  double max_speed = 0.55;
};

/// Per-goal overrides after defaults fill the blanks and the config caps the rest.
GotoLimits resolveGotoLimits(const GotoRequest & request, const MotionLimits & limits);

}  // namespace uav_navigation

#endif  // UAV_NAVIGATION__GOAL_ADMISSION_HPP_
