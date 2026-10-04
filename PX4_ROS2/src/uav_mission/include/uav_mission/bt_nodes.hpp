#ifndef UAV_MISSION__BT_NODES_HPP_
#define UAV_MISSION__BT_NODES_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include "uav_mission/mission_context.hpp"
#include "uav_mission/mission_registry.hpp"

namespace uav_mission
{

/// Registers the 6 custom leaf node types the 3 shipped mission XML bodies
/// use (mission_registry.hpp's allowedXmlTags() whitelist) against
/// factory, each bound to context. Standard BT.CPP controls/decorators
/// (Sequence, Fallback, ReactiveFallback, Repeat, Timeout, Inverter,
/// RetryUntilSuccessful) need no registration -- BehaviorTreeFactory's own
/// constructor already provides them.
void registerMissionBtNodes(BT::BehaviorTreeFactory & factory, MissionContext * context);

/// The only leaf that talks to the navigator (plan P9 S:1 "Guard trong
/// CODE, than mission trong XML" -- this class dispatches through
/// MissionContext, never rclcpp_action directly, so it stays testable
/// against a fake MissionContext with no ROS at all).
///
/// nav_type is read once in onStart() and supports exactly the two values
/// the 3 shipped missions use: "GotoPose" (via target={Waypoint} OR plain
/// x/y/z ports) and "TrackTarget". Any other value is a hard FAILURE with
/// an ABORTED_INVALID_GOAL terminal verdict recorded via the context --
/// scope decision, see uav_mission/README.md ("P9.4 lech plan").
class NavAction : public BT::StatefulActionNode
{
public:
  NavAction(const std::string & name, const BT::NodeConfiguration & config, MissionContext * context);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class LegState : uint8_t { kAwaitingDispatch, kInFlight, kBackoffWait };

  bool readPorts();
  void tryDispatch();

  MissionContext * context_;

  std::string nav_type_;
  // GotoPose ports.
  double x_ = 0.0;
  double y_ = 0.0;
  double z_ = 0.0;
  std::string frame_id_ = "odom";
  double acceptance_radius_ = 0.3;
  double max_speed_ = 0.55;
  // TrackTarget ports.
  int32_t target_id_ = -1;
  double standoff_distance_ = 2.0;
  double duration_seconds_ = 0.0;
  double target_lost_timeout_ = 4.0;

  LegState leg_state_ = LegState::kAwaitingDispatch;
  double backoff_until_sec_ = 0.0;
  // G-M4.4b (2026-08-23): rosNowSeconds() at the most recent successful
  // tryDispatch() (fresh send OR a backoff-retry re-send) -- feeds
  // MissionContext::noteTrackTargetProgress() while a TrackTarget leg is
  // kInFlight, so ANY interruption (retry, re-dispatch) restarts the
  // "sustained progress" clock rather than accumulating across gaps. Also
  // Y1's step-timeout clock (SAME reset points -- a step timeout is
  // measured per DISPATCH ATTEMPT, matching leg_start_sec_'s existing
  // scope exactly, not accumulated across retries).
  double leg_start_sec_ = 0.0;
  // Y1 (2026-08-23, review round 1): set true the tick onRunning() detects
  // now - leg_start_sec_ >= stepTimeoutSec() and issues the cancel; the
  // NEXT outcome this leg receives (whatever its own canceled/succeeded/
  // failed flavor) is then routed to evaluateStepFailure(ABORTED_TIMEOUT)
  // instead of the normal per-outcome handling -- without this flag, a
  // timeout-triggered cancel would fall into the SAME "outcome.canceled ->
  // not this step's fault, resend forever" branch a pause/abort-triggered
  // cancel uses, and step_timeout_sec would still enforce nothing.
  bool step_timeout_triggered_ = false;
};

/// Cycles through an input `waypoints` list, one step per tick it is
/// ticked -- persistent index_ survives across ticks because BT.CPP tree
/// nodes are constructed once and re-ticked, not rebuilt (SyncActionNode:
/// synchronous, no RUNNING state, matches a single cheap list-index
/// advance).
class WaypointCursor : public BT::SyncActionNode
{
public:
  WaypointCursor(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::size_t index_ = 0;
};

/// Waits duration_sec, ticked on the SAME rosNowSeconds() clock the rest of
/// the node uses (not steady_clock) -- keeps Dwell consistent with
/// use_sim_time across the whole stack.
class Dwell : public BT::StatefulActionNode
{
public:
  Dwell(const std::string & name, const BT::NodeConfiguration & config, MissionContext * context);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  MissionContext * context_;
  double duration_sec_ = 0.0;
  double start_sec_ = 0.0;
};

/// Condition: is marker_id (-1 = any) currently visible per
/// /world/semantic_landmarks? On SUCCESS also writes marker_x/marker_y
/// (inspect_point.xml's goto_over_marker step reads them back).
class MarkerSeen : public BT::ConditionNode
{
public:
  MarkerSeen(const std::string & name, const BT::NodeConfiguration & config, MissionContext * context);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  MissionContext * context_;
};

/// Condition: is target_id (-1 = any) currently visible per
/// /world/target_state? On SUCCESS also writes track_id -- the id ACTUALLY
/// seen (never -1 on success), which follow_target.xml feeds straight back
/// into NavAction(TrackTarget)'s own target_id port so the dispatched goal
/// is ANCHORED to one identity for its whole lifetime (G-M4.4, 2026-08-23).
class TargetSeen : public BT::ConditionNode
{
public:
  TargetSeen(const std::string & name, const BT::NodeConfiguration & config, MissionContext * context);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  MissionContext * context_;
};

/// Condition: has follow_target's search-attempts budget NOT been
/// exhausted yet (G-M4.4b, 2026-08-23)? Gates search_when_lost so a
/// persistent (across-flicker) loss-episode counter -- kept by
/// MissionContext, NOT by this stateless leaf -- can eventually make the
/// WHOLE tree return FAILURE once exhausted, instead of an endless
/// track<->search flicker loop that only ever ends via the outer Timeout
/// with the wrong result code. No ports: reads/writes nothing on the
/// blackboard.
/// bug #12 (2026-08-23): NOT purely a read-only gate anymore -- tick()
/// ALSO spends one budget unit on every fresh entry (moved here from
/// NavAction's TrackTarget-terminal-failure branch, see
/// MissionContext::noteSearchEpisodeBegins()) because this is the ONLY
/// place reliably reached on EVERY entry into search_when_lost, including
/// the case where TrackTarget was never even dispatched (TargetSeen never
/// true from mission start -- "arena hoan toan sach").
class SearchBudgetAvailable : public BT::ConditionNode
{
public:
  SearchBudgetAvailable(
    const std::string & name, const BT::NodeConfiguration & config, MissionContext * context);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  MissionContext * context_;
};

}  // namespace uav_mission

#endif  // UAV_MISSION__BT_NODES_HPP_
