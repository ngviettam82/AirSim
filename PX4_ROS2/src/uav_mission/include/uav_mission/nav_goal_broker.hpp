#ifndef UAV_MISSION__NAV_GOAL_BROKER_HPP_
#define UAV_MISSION__NAV_GOAL_BROKER_HPP_

#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

namespace uav_mission
{

enum class BrokerState : uint8_t
{
  kIdle,
  kSending,
  kActive,
  kCanceling,
  kCancelRefused,
};

const char * brokerStateName(BrokerState state);

/// Terminal outcome of a navigator goal, as handed back by takeOutcome().
/// Deliberately NOT a uav_interfaces message: this header stays ROS-free so
/// the FSM below is unit-testable without rclcpp_action (plan P9 S:5 P9.4:
/// "uu tien tach de test khong can ROS").
struct GoalOutcome
{
  bool succeeded = false;
  bool canceled = false;
  uint8_t result_code = 0;   // mission_policy::kResultUnknown; set iff !succeeded && !canceled
  std::string message;
};

/// At-most-one-goal-navigator-forever FSM (plan P9 S:1). Every actual
/// rclcpp_action call is injected by the caller (mission_executor_node) as
/// a std::function -- this class never includes rclcpp_action, so P9.4's
/// gate criteria (a) no >1 concurrent goal, (b) cancel-before-new-send,
/// (f) cancel-refused-at-LANDING-does-not-hang are pinnable directly
/// against this FSM with scripted fake send/cancel callbacks, with or
/// without a real fake navigator process behind them.
///
/// Three things to know before changing this file:
///   1. ONE mutex_ guards state_ AND pending_outcome_. request()/cancel()
///      are called from mission_executor_node's tick_group_ (BT tick
///      thread); onGoalAccepted()/onGoalRejected()/onCancelAccepted()/
///      onCancelRefused()/onResult() are called from whatever thread the
///      caller's rclcpp_action client callbacks land on (io_group_) -- R24.
///   2. request() only ever dispatches send_fn from kIdle, and returns
///      false (no-op) otherwise -- no queueing, matching navigator's own
///      "REJECT immediately, do not queue" contract
///      (uav_navigation/README.md S:2). send_fn is invoked OUTSIDE the
///      lock (it does real I/O) after state_ is already kSending, so a
///      synchronous/mocked send_fn that calls back into onGoalAccepted()
///      immediately can never deadlock on this non-recursive mutex.
///   3. cancel() only ever dispatches cancel_fn from kActive (a goal that
///      has been fully accepted) -- a cancel requested while still
///      kSending simply waits: the caller's next tick will see kActive (or
///      kIdle if it was rejected, in which case there is nothing left to
///      cancel) and try again. This costs at most one tick (100 ms at
///      10 Hz) and is strictly more conservative than racing a cancel
///      against an in-flight accept.
class NavGoalBroker
{
public:
  BrokerState state() const;

  /// state() != kIdle -- named for readability at call sites that only
  /// care whether a new request() would be accepted right now.
  bool busy() const;

  /// Dispatches send_fn() iff state() == kIdle. send_fn must eventually
  /// call onGoalAccepted() or onGoalRejected() exactly once. Returns false
  /// without calling send_fn if not idle.
  bool request(const std::function<void()> & send_fn);

  /// Navigator accepted the goal -- kSending -> kActive. A late call
  /// arriving after a cancel() already moved state_ to kCanceling is
  /// harmless: state_ is left at kCanceling (the goal handle the cancel
  /// targets is now valid and in flight, exactly as intended).
  void onGoalAccepted();
  /// Navigator rejected the goal at its own goal-callback gate -- kSending
  /// -> kIdle. The rejection becomes an ordinary terminal outcome
  /// (ABORTED_INVALID_GOAL-shaped) via takeOutcome(), so callers handle a
  /// REJECT exactly like any other finished goal.
  void onGoalRejected(uint8_t result_code, const std::string & message);

  /// Dispatches cancel_fn() iff state() == kActive. Returns false, no-op,
  /// if kIdle/kSending/kCanceling/kCancelRefused already (kSending: see
  /// point 3 above; the rest: a cancel is already outstanding or there is
  /// nothing to cancel).
  bool cancel(const std::function<void()> & cancel_fn);
  /// Navigator's cancel service accepted the request for THIS goal --
  /// cancellation is in flight. R21: this is NOT itself the terminal
  /// event; onResult() still ends the episode. Kept as a named entry point
  /// (rather than a silent no-op inline) for callers that want to log the
  /// acknowledgement.
  void onCancelAccepted();
  /// Navigator's cancel service REFUSED this specific goal (uav_navigation
  /// README S:2: "Cancel khi dang LANDING bi TU CHOI") -- kCanceling ->
  /// kCancelRefused. The goal keeps running; onResult() still ends it, and
  /// request() stays blocked until it does (gate criterion (f)).
  void onCancelRefused();

  /// The navigator action's own result callback -- from ANY of
  /// kSending/kActive/kCanceling/kCancelRefused (a result can race a
  /// cancel response in either order). Always -> kIdle. A stray call while
  /// already kIdle (a late/duplicate callback) is ignored.
  void onResult(const GoalOutcome & outcome);

  /// Pops the most recent terminal outcome if one has not yet been
  /// consumed. std::nullopt if state() is not kIdle, or if this kIdle
  /// episode's outcome was already taken.
  std::optional<GoalOutcome> takeOutcome();

private:
  mutable std::mutex mutex_;
  BrokerState state_ = BrokerState::kIdle;
  std::optional<GoalOutcome> pending_outcome_;
};

}  // namespace uav_mission

#endif  // UAV_MISSION__NAV_GOAL_BROKER_HPP_
