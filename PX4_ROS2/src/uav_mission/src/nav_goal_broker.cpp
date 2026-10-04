#include "uav_mission/nav_goal_broker.hpp"

namespace uav_mission
{

const char * brokerStateName(BrokerState state)
{
  switch (state) {
    case BrokerState::kIdle: return "IDLE";
    case BrokerState::kSending: return "SENDING";
    case BrokerState::kActive: return "ACTIVE";
    case BrokerState::kCanceling: return "CANCELING";
    case BrokerState::kCancelRefused: return "CANCEL_REFUSED";
    default: return "UNKNOWN";
  }
}

BrokerState NavGoalBroker::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool NavGoalBroker::busy() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_ != BrokerState::kIdle;
}

bool NavGoalBroker::request(const std::function<void()> & send_fn)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != BrokerState::kIdle) {
      return false;
    }
    state_ = BrokerState::kSending;
    pending_outcome_.reset();
  }
  // Outside the lock (point 2): send_fn does real I/O and may synchronously
  // call back into onGoalAccepted()/onGoalRejected() in a mocked/unit-test
  // caller -- mutex_ is non-recursive, so holding it here would deadlock.
  send_fn();
  return true;
}

void NavGoalBroker::onGoalAccepted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == BrokerState::kSending) {
    state_ = BrokerState::kActive;
  }
}

void NavGoalBroker::onGoalRejected(uint8_t result_code, const std::string & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != BrokerState::kSending) {
    return;   // stray/late callback for a goal already moved past kSending
  }
  state_ = BrokerState::kIdle;
  pending_outcome_ = GoalOutcome{false, false, result_code, message};
}

bool NavGoalBroker::cancel(const std::function<void()> & cancel_fn)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != BrokerState::kActive) {
      return false;
    }
    state_ = BrokerState::kCanceling;
  }
  cancel_fn();
  return true;
}

void NavGoalBroker::onCancelAccepted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  // R21: deliberately a no-op besides holding kCanceling -- the terminal
  // event this FSM acts on is onResult(), never this acknowledgement.
}

void NavGoalBroker::onCancelRefused()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == BrokerState::kCanceling) {
    state_ = BrokerState::kCancelRefused;
  }
}

void NavGoalBroker::onResult(const GoalOutcome & outcome)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == BrokerState::kIdle) {
    return;   // stray/late callback, already handled
  }
  state_ = BrokerState::kIdle;
  pending_outcome_ = outcome;
}

std::optional<GoalOutcome> NavGoalBroker::takeOutcome()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != BrokerState::kIdle || !pending_outcome_.has_value()) {
    return std::nullopt;
  }
  std::optional<GoalOutcome> result = pending_outcome_;
  pending_outcome_.reset();
  return result;
}

}  // namespace uav_mission
