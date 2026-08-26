#ifndef UAV_MISSION__CONTINUITY_TRACKER_HPP_
#define UAV_MISSION__CONTINUITY_TRACKER_HPP_

#include <optional>

namespace uav_mission
{

/// Tracks how long a boolean predicate has held TRUE continuously -- the
/// caller-owned clock companion to mission_policy's pure evaluate() (R21:
/// the policy library never reads a clock). One instance per
/// "*_elapsed_sec" field of MissionWorldView (mission_policy.hpp), same
/// technique as uav_safety's FailsafePolicy internal ContinuityTracker, but
/// owned here by mission_executor_node instead (P9.2's documented split,
/// see mission_policy.hpp's MissionWorldView comment).
class ContinuityTracker
{
public:
  /// Call once per tick with the CURRENT truth value of the predicate this
  /// tracker watches. Returns 0.0 the instant the predicate is false, and
  /// the seconds since it FIRST became true (this episode) while it stays
  /// true.
  double update(bool predicate_true, double now_sec)
  {
    if (!predicate_true) {
      began_at_sec_.reset();
      return 0.0;
    }
    if (!began_at_sec_.has_value()) {
      began_at_sec_ = now_sec;
    }
    return now_sec - *began_at_sec_;
  }

  void reset() {began_at_sec_.reset();}

private:
  std::optional<double> began_at_sec_;
};

}  // namespace uav_mission

#endif  // UAV_MISSION__CONTINUITY_TRACKER_HPP_
