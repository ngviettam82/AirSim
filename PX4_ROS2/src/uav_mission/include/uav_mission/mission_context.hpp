#ifndef UAV_MISSION__MISSION_CONTEXT_HPP_
#define UAV_MISSION__MISSION_CONTEXT_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "uav_mission/mission_policy.hpp"
#include "uav_mission/nav_goal_broker.hpp"

namespace uav_mission
{

/// A landmark/target sighting, as the BT leaf nodes need it -- plain
/// structs, not the generated uav_interfaces messages, so bt_nodes.hpp
/// (included by both mission_executor_node.hpp and the custom BT node
/// implementations) never has to pull in message headers just to declare
/// this interface.
struct SightedLandmark
{
  double x = 0.0;
  double y = 0.0;
};

/// A target sighting -- like SightedLandmark, plus the track_id it was
/// actually seen under. G-M4.4 (2026-08-23): TargetSeen (bt_nodes.cpp)
/// writes this id to the blackboard so the NEXT NavAction(TrackTarget) can
/// anchor its goal to it, instead of mission always sending target_id=-1
/// ("any"), which defeated both the tracker's own M/N ghost filter AND
/// navigator's targetSample(wanted_id) id-pinning (navigator_action_server_
/// node.cpp:2597) -- see follow_target.xml / README.md for the full
/// rebind-on-reacquire semantics this exists to support.
struct SightedTarget
{
  double x = 0.0;
  double y = 0.0;
  int32_t track_id = -1;
};

/// Pure interface the 6 custom BT leaf nodes (bt_nodes.hpp) call into.
/// mission_executor_node.hpp implements this; bt_nodes.hpp only needs this
/// header, never mission_executor_node.hpp itself -- breaks what would
/// otherwise be a circular include (the node registers the leaf node
/// builders, the leaves call back into the node).
class MissionContext
{
public:
  virtual ~MissionContext() = default;

  virtual double rosNowSeconds() const = 0;
  virtual double retryBackoffSec() const = 0;
  /// Y1 (2026-08-23, review round 1): mission_policy's step_timeout_sec
  /// (validated >= 1.2 x navigator_goto_timeout_copy_sec) was declared but
  /// never enforced anywhere -- dead on paper. NavAction::onRunning() reads
  /// this to cancel+fail a goal stuck kInFlight past this many seconds.
  virtual double stepTimeoutSec() const = 0;

  // ---- NavAction's two supported nav_type values (plan P9 S:3: the only
  // two used by the 3 shipped mission bodies). Each returns whether the
  // broker actually dispatched (false means "still busy, try again next
  // tick" -- NavAction retries the SAME send, never treats this as a
  // failure). ----
  virtual bool sendGotoPose(
    double x, double y, double z, const std::string & frame_id,
    double acceptance_radius, double max_speed) = 0;
  virtual bool sendTrackTarget(
    int32_t target_id, double standoff_distance, double duration_seconds,
    double target_lost_timeout) = 0;

  /// Pops the broker's terminal outcome for the goal NavAction most
  /// recently dispatched (nullopt while still in flight).
  virtual std::optional<GoalOutcome> pollNavActionOutcome() = 0;
  /// Best-effort cancel of whatever goal NavAction currently owns --
  /// called from NavAction::onHalted() (e.g. a Timeout decorator halting
  /// this branch). A no-op if the broker is not kActive.
  virtual void cancelNavActionGoal() = 0;

  /// Step-boundary reset (mission_policy.hpp: "Reset at the STEP boundary
  /// by the caller") -- called once from NavAction::onStart().
  virtual void resetStepRetryState() = 0;
  /// Priority-5 (step/body failure) decision: reuses mission_policy_ with
  /// priorities 1-4 forced inert (see mission_executor_node.hpp for why
  /// that split is safe) plus this leaf's own retry_count_/reason
  /// bookkeeping. A kAbortHold/kAbortLand verdict is ALSO recorded
  /// node-side as the pending terminal verdict for the tick loop to act on
  /// once the tree returns FAILURE.
  virtual GuardVerdict evaluateStepFailure(uint8_t result_code, const std::string & reason) = 0;

  virtual void reportStepStarted(const std::string & step_name) = 0;
  virtual void reportStepCompleted(const std::string & step_name, bool succeeded) = 0;

  virtual std::optional<SightedLandmark> latestLandmark(int32_t marker_id) const = 0;
  virtual std::optional<SightedTarget> latestTarget(int32_t target_id) const = 0;

  // ---- G-M4.4b (2026-08-23): follow_target's loss-episode search-attempts
  // budget. A momentary TargetSeen flicker (one tick true) must NOT reset
  // it -- only sustained re-tracking does -- so this is deliberately kept
  // SEPARATE from resetStepRetryState() (which onStart() calls on every
  // fresh dispatch, flicker included). See bt_nodes.cpp NavAction and
  // SearchBudgetAvailable, and mission_executor_node.cpp tickBody(). ----

  /// Called every RUNNING tick a TrackTarget leg has a goal genuinely
  /// in-flight (no outcome yet), with the seconds elapsed since THAT
  /// dispatch attempt started (reset on every fresh send, retries
  /// included -- any interruption restarts it, so this can only reach
  /// track_progress_reset_sec via truly UNINTERRUPTED tracking). Resets
  /// the search-attempts budget to 0 once past that threshold.
  virtual void noteTrackTargetProgress(double elapsed_this_attempt_sec) = 0;
  /// bug #12 (2026-08-23): called by SearchBudgetAvailable::tick() on
  /// EVERY fresh entry into search_when_lost, UNCONDITIONALLY -- moved
  /// here (off NavAction's TrackTarget-terminal-failure branch, G-M4.4b's
  /// original hook) because a fresh entry can ALSO happen with NO prior
  /// TrackTarget attempt at all (TargetSeen fails from the very first
  /// tick, e.g. "arena hoan toan sach" -- target never existed), which
  /// that original hook could never observe.
  virtual void noteSearchEpisodeBegins() = 0;
  /// Whether the search-attempts budget still has room for ANOTHER search
  /// episode -- read by SearchBudgetAvailable (bt_nodes.hpp) to gate
  /// search_when_lost; once false, the whole follow_target tree returns
  /// FAILURE instead of dispatching another search, letting
  /// mission_executor_node route to Recover(CLIMB) -> ABORTED_LOST_TARGET.
  virtual bool searchBudgetAvailable() const = 0;
  /// bug #12 (2026-08-23): called by SearchBudgetAvailable::tick() exactly
  /// once, the SAME tick it is about to return FAILURE (budget just
  /// exhausted) -- records the terminal ABORTED_LOST_TARGET verdict
  /// directly, mirroring what evaluateStepFailure() does for a TrackTarget-
  /// side terminal failure. Needed because the "arena hoan toan sach" path
  /// never goes through evaluateStepFailure() at all (TrackTarget is never
  /// even dispatched), so nothing else would ever record this verdict.
  virtual void noteSearchBudgetExhausted() = 0;
};

}  // namespace uav_mission

#endif  // UAV_MISSION__MISSION_CONTEXT_HPP_
