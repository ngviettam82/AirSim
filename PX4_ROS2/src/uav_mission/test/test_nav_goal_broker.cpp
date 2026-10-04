// The at-most-one-goal-forever FSM, exercised directly.
//
// WHY THIS FILE EXISTS (S3 coverage, 2026-08-26). NavGoalBroker had no unit test
// of its own: everything reached it through mission_executor_node, so the arms
// that exist ONLY to survive a racing callback -- a rejection arriving after the
// goal already moved past kSending, a result arriving twice -- were never run by
// ctest at all. Those arms are the whole point of the class. The header is
// deliberately ROS-free (plan P9 S:5) precisely so they can be pinned here.
//
// Every case drives the FSM through its public entry points only; no test reaches
// into state_, so a refactor that keeps the contract keeps these tests.

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "uav_mission/nav_goal_broker.hpp"

using uav_mission::BrokerState;
using uav_mission::GoalOutcome;
using uav_mission::NavGoalBroker;
using uav_mission::brokerStateName;

namespace
{

GoalOutcome succeeded()
{
  GoalOutcome outcome;
  outcome.succeeded = true;
  return outcome;
}

/// Drives a broker to kActive, which is where most of the interesting arms start.
/// Takes a reference: the broker owns a mutex and is deliberately non-copyable.
void driveToActive(NavGoalBroker & broker)
{
  broker.request([]() {});
  broker.onGoalAccepted();
}

}  // namespace

TEST(NavGoalBroker, StartsIdleAndNotBusy)
{
  const NavGoalBroker broker;
  EXPECT_EQ(broker.state(), BrokerState::kIdle);
  EXPECT_FALSE(broker.busy());
}

TEST(NavGoalBroker, ASecondRequestIsRefusedWhileOneIsStillInFlight)
{
  int sends = 0;
  NavGoalBroker broker;

  EXPECT_TRUE(broker.request([&sends]() {++sends;}));
  EXPECT_EQ(broker.state(), BrokerState::kSending);
  EXPECT_TRUE(broker.busy());

  EXPECT_FALSE(broker.request([&sends]() {++sends;}));
  broker.onGoalAccepted();
  EXPECT_FALSE(broker.request([&sends]() {++sends;}));

  EXPECT_EQ(sends, 1) << "a second goal reached the navigator";
}

// The rejection path: the navigator's own goal gate said no.
TEST(NavGoalBroker, ARejectionEndsTheEpisodeAsAnOrdinaryOutcome)
{
  NavGoalBroker broker;
  broker.request([]() {});
  broker.onGoalRejected(7, "refused: already airborne");

  EXPECT_EQ(broker.state(), BrokerState::kIdle);
  const std::optional<GoalOutcome> outcome = broker.takeOutcome();
  ASSERT_TRUE(outcome.has_value());
  EXPECT_FALSE(outcome->succeeded);
  EXPECT_FALSE(outcome->canceled);
  EXPECT_EQ(outcome->result_code, 7);
  EXPECT_EQ(outcome->message, "refused: already airborne");
}

// The arm this file was written for: a rejection that arrives late, after the
// goal already became active. It must not tear down a live goal.
TEST(NavGoalBroker, ALateRejectionCannotUnseatAGoalThatIsAlreadyActive)
{
  NavGoalBroker broker;
  driveToActive(broker);
  ASSERT_EQ(broker.state(), BrokerState::kActive);

  broker.onGoalRejected(7, "stray");

  EXPECT_EQ(broker.state(), BrokerState::kActive) << "a stray rejection ended a live goal";
  EXPECT_FALSE(broker.takeOutcome().has_value()) << "a stray rejection invented an outcome";
}

TEST(NavGoalBroker, ARejectionArrivingWhileIdleIsIgnored)
{
  NavGoalBroker broker;
  broker.onGoalRejected(7, "stray");

  EXPECT_EQ(broker.state(), BrokerState::kIdle);
  EXPECT_FALSE(broker.takeOutcome().has_value());
}

TEST(NavGoalBroker, CancelOnlyDispatchesFromActive)
{
  int cancels = 0;
  const auto count = [&cancels]() {++cancels;};

  NavGoalBroker idle;
  EXPECT_FALSE(idle.cancel(count));

  NavGoalBroker sending;
  sending.request([]() {});
  EXPECT_FALSE(sending.cancel(count)) << "cancel raced an accept that had not landed";

  NavGoalBroker active;
  driveToActive(active);
  EXPECT_TRUE(active.cancel(count));
  EXPECT_EQ(active.state(), BrokerState::kCanceling);
  EXPECT_FALSE(active.cancel(count)) << "a second cancel went out for one goal";

  EXPECT_EQ(cancels, 1);
}

// Gate criterion (f): a refused cancel must not hang the broker, and must not
// free it either -- the goal is still running.
TEST(NavGoalBroker, ARefusedCancelKeepsTheGoalAndStaysBlockedUntilTheResult)
{
  NavGoalBroker broker;
  driveToActive(broker);
  ASSERT_TRUE(broker.cancel([]() {}));
  broker.onCancelRefused();

  EXPECT_EQ(broker.state(), BrokerState::kCancelRefused);
  EXPECT_TRUE(broker.busy());
  EXPECT_FALSE(broker.request([]() {})) << "a new goal went out while one was still running";

  broker.onResult(succeeded());
  EXPECT_EQ(broker.state(), BrokerState::kIdle);
  EXPECT_TRUE(broker.request([]() {})) << "the broker stayed wedged after the goal ended";
}

TEST(NavGoalBroker, AnAcknowledgedCancelIsNotItselfTerminal)
{
  NavGoalBroker broker;
  driveToActive(broker);
  ASSERT_TRUE(broker.cancel([]() {}));
  broker.onCancelAccepted();

  EXPECT_EQ(broker.state(), BrokerState::kCanceling) << "the ack was treated as the end";
  EXPECT_FALSE(broker.takeOutcome().has_value());
}

// A result can arrive from any non-idle state, in either order against a cancel.
TEST(NavGoalBroker, AResultEndsTheEpisodeFromEveryNonIdleState)
{
  {
    NavGoalBroker broker;
    broker.request([]() {});
    broker.onResult(succeeded());
    EXPECT_EQ(broker.state(), BrokerState::kIdle);
  }
  {
    NavGoalBroker broker;
    driveToActive(broker);
    broker.onResult(succeeded());
    EXPECT_EQ(broker.state(), BrokerState::kIdle);
  }
  {
    NavGoalBroker broker;
    driveToActive(broker);
    broker.cancel([]() {});
    broker.onResult(succeeded());
    EXPECT_EQ(broker.state(), BrokerState::kIdle);
  }
  {
    NavGoalBroker broker;
    driveToActive(broker);
    broker.cancel([]() {});
    broker.onCancelRefused();
    broker.onResult(succeeded());
    EXPECT_EQ(broker.state(), BrokerState::kIdle);
  }
}

// The second arm this file was written for: a duplicate result callback.
TEST(NavGoalBroker, ADuplicateResultCannotOverwriteTheOutcomeAlreadyTaken)
{
  NavGoalBroker broker;
  driveToActive(broker);

  GoalOutcome real;
  real.succeeded = false;
  real.result_code = 11;
  real.message = "the outcome that happened";
  broker.onResult(real);

  const std::optional<GoalOutcome> taken = broker.takeOutcome();
  ASSERT_TRUE(taken.has_value());
  EXPECT_EQ(taken->result_code, 11);

  GoalOutcome stray;
  stray.succeeded = true;
  stray.message = "late duplicate";
  broker.onResult(stray);

  EXPECT_EQ(broker.state(), BrokerState::kIdle);
  EXPECT_FALSE(broker.takeOutcome().has_value())
    << "a late duplicate result reopened an episode that was already accounted for";
}

TEST(NavGoalBroker, AnOutcomeIsHandedOutExactlyOnce)
{
  NavGoalBroker broker;
  driveToActive(broker);
  broker.onResult(succeeded());

  EXPECT_TRUE(broker.takeOutcome().has_value());
  EXPECT_FALSE(broker.takeOutcome().has_value()) << "the same outcome was consumed twice";
}

TEST(NavGoalBroker, NoOutcomeIsOfferedWhileAGoalIsStillRunning)
{
  NavGoalBroker broker;
  driveToActive(broker);
  EXPECT_FALSE(broker.takeOutcome().has_value());
}

// Diagnostics only, but a wrong name here sends a reader to the wrong state.
TEST(NavGoalBroker, EveryStateHasItsOwnName)
{
  const std::vector<BrokerState> states{
    BrokerState::kIdle, BrokerState::kSending, BrokerState::kActive,
    BrokerState::kCanceling, BrokerState::kCancelRefused};

  std::vector<std::string> names;
  for (const BrokerState state : states) {
    const std::string name = brokerStateName(state);
    EXPECT_FALSE(name.empty());
    EXPECT_EQ(std::count(names.begin(), names.end(), name), 0) << "duplicate name: " << name;
    names.push_back(name);
  }
  EXPECT_EQ(std::string(brokerStateName(static_cast<BrokerState>(200))), "UNKNOWN");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
