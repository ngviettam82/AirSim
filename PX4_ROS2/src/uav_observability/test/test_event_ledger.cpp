// ROS-free (no domain, no rclcpp::init): the edge-detection/formatting logic
// is pinned by a test instead of a real bag/log, same rule uav_safety's
// failsafe_policy tests follow.
#include <map>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "uav_observability/event_ledger.hpp"

namespace uav_observability
{
namespace
{

bool contains(const std::string & haystack, const std::string & needle)
{
  return haystack.find(needle) != std::string::npos;
}

}  // namespace

// ===========================================================================
// escapeJson.
// ===========================================================================

TEST(EscapeJson, QuoteAndBackslashAreEscaped)
{
  EXPECT_EQ(escapeJson("a\"b\\c"), "a\\\"b\\\\c");
}

TEST(EscapeJson, NewlineAndControlCharsAreEscaped)
{
  EXPECT_EQ(escapeJson("a\nb"), "a\\nb");
  EXPECT_EQ(escapeJson("a\tb"), "a\\tb");
  EXPECT_EQ(escapeJson("a\rb"), "a\\rb");
}

TEST(EscapeJson, PlainAsciiIsUnchanged)
{
  EXPECT_EQ(escapeJson("HOLDING"), "HOLDING");
}

// ===========================================================================
// Invariant 1: repeated value -> nullopt (no line, no seq spent); changed
// value -> one line.
// ===========================================================================

TEST(EventLedger, RepeatedValueYieldsNullopt)
{
  EventLedger ledger(2.0);
  const auto first = ledger.observe("safety", "state", "NORMAL", EventLevel::kInfo, 0.0, 0.0);
  ASSERT_TRUE(first.has_value());
  const auto second = ledger.observe("safety", "state", "NORMAL", EventLevel::kInfo, 1.0, 1.0);
  EXPECT_FALSE(second.has_value());
}

TEST(EventLedger, ChangedValueEmitsALine)
{
  EventLedger ledger(2.0);
  ledger.observe("safety", "state", "NORMAL", EventLevel::kInfo, 0.0, 0.0);
  const auto changed = ledger.observe("safety", "state", "HOLDING", EventLevel::kWarn, 1.0, 1.0);
  ASSERT_TRUE(changed.has_value());
  EXPECT_TRUE(contains(*changed, "\"from\":\"NORMAL\""));
  EXPECT_TRUE(contains(*changed, "\"to\":\"HOLDING\""));
}

// Mutation counter: repeats must never consume a sequence number -- the
// NEXT genuine change must follow directly from the last emitted seq, with
// no gap for the swallowed repeats in between.
TEST(EventLedger, RepeatsNeverConsumeASequenceNumber)
{
  EventLedger ledger(2.0);
  const auto first = ledger.observe("safety", "state", "NORMAL", EventLevel::kInfo, 0.0, 0.0);
  ledger.observe("safety", "state", "NORMAL", EventLevel::kInfo, 1.0, 1.0);   // repeat, swallowed
  ledger.observe("safety", "state", "NORMAL", EventLevel::kInfo, 2.0, 2.0);   // repeat, swallowed
  const auto second = ledger.observe("safety", "state", "HOLDING", EventLevel::kWarn, 3.0, 3.0);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_TRUE(contains(*first, "\"seq\":0"));
  EXPECT_TRUE(contains(*second, "\"seq\":1"));
}

// ===========================================================================
// Invariant 2: first sighting of (src, field) always counts as a change,
// reported with from="".
// ===========================================================================

TEST(EventLedger, FirstSightingEmitsWithEmptyFrom)
{
  EventLedger ledger(2.0);
  const auto line = ledger.observe("mission", "step", "TAKEOFF", EventLevel::kInfo, 0.0, 0.0);
  ASSERT_TRUE(line.has_value());
  EXPECT_TRUE(contains(*line, "\"from\":\"\""));
  EXPECT_TRUE(contains(*line, "\"to\":\"TAKEOFF\""));
}

// ===========================================================================
// Invariant 3: seq is a single counter, continuous, spanning every src.
// ===========================================================================

TEST(EventLedger, SeqIsContinuousAcrossDifferentSources)
{
  EventLedger ledger(2.0);
  const auto a = ledger.observe("safety", "state", "NORMAL", EventLevel::kInfo, 0.0, 0.0);
  const auto b = ledger.observe("authority", "active_source", "MISSION", EventLevel::kInfo, 0.0, 0.0);
  const auto c = ledger.observe("mission", "step", "TAKEOFF", EventLevel::kInfo, 0.0, 0.0);
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  ASSERT_TRUE(c.has_value());
  EXPECT_TRUE(contains(*a, "\"seq\":0"));
  EXPECT_TRUE(contains(*b, "\"seq\":1"));
  EXPECT_TRUE(contains(*c, "\"seq\":2"));
  EXPECT_EQ(ledger.sequenceCount(), 3u);
}

TEST(EventLedger, DifferentFieldsOfTheSameSrcAreIndependentEdges)
{
  EventLedger ledger(2.0);
  const auto state_line = ledger.observe(
    "authority", "active_source", "MISSION", EventLevel::kInfo, 0.0, 0.0);
  const auto latch_line = ledger.observe("authority", "latch", "false", EventLevel::kInfo, 0.0, 0.0);
  ASSERT_TRUE(state_line.has_value());
  ASSERT_TRUE(latch_line.has_value());
  EXPECT_TRUE(contains(*state_line, "\"seq\":0"));
  EXPECT_TRUE(contains(*latch_line, "\"seq\":1"));
}

// ===========================================================================
// Invariant 4: fixed key set/order, correct JSON escaping.
// ===========================================================================

TEST(EventLedger, KeyOrderIsFixed)
{
  EventLedger ledger(2.0);
  std::map<std::string, std::string> detail{{"reason", "watchdog"}};
  const auto line = ledger.observe(
    "safety", "state", "HOLDING", EventLevel::kError, 12.5, 100.25, detail);
  ASSERT_TRUE(line.has_value());
  const std::vector<std::string> keys = {
    "\"t_sim\"", "\"t_wall\"", "\"seq\"", "\"src\"", "\"field\"", "\"from\"", "\"to\"", "\"level\"",
    "\"detail\""};
  size_t last_pos = 0;
  for (const auto & key : keys) {
    const size_t pos = line->find(key);
    ASSERT_NE(pos, std::string::npos) << "missing key " << key;
    EXPECT_GE(pos, last_pos) << "key " << key << " out of fixed order";
    last_pos = pos;
  }
}

TEST(EventLedger, AllValuesArePresentWithCorrectContent)
{
  EventLedger ledger(2.0);
  const auto line = ledger.observe(
    "safety", "state", "HOLDING", EventLevel::kError, 12.5, 100.25);
  ASSERT_TRUE(line.has_value());
  EXPECT_TRUE(contains(*line, "\"t_sim\":12.5"));
  EXPECT_TRUE(contains(*line, "\"t_wall\":100.25"));
  EXPECT_TRUE(contains(*line, "\"src\":\"safety\""));
  EXPECT_TRUE(contains(*line, "\"field\":\"state\""));
  EXPECT_TRUE(contains(*line, "\"to\":\"HOLDING\""));
  EXPECT_TRUE(contains(*line, "\"level\":\"ERROR\""));
}

TEST(EventLedger, ValueContainingQuoteAndNewlineIsEscapedCorrectly)
{
  EventLedger ledger(2.0);
  const auto line = ledger.observe(
    "mission", "events", "text with \"quotes\" and\nnewline", EventLevel::kInfo, 0.0, 0.0);
  ASSERT_TRUE(line.has_value());
  EXPECT_TRUE(contains(*line, "text with \\\"quotes\\\" and\\nnewline"));
  // Must not contain a raw, unescaped newline inside the JSONL line itself
  // -- that would break "one line == one event".
  EXPECT_FALSE(contains(*line, "\n"));
}

TEST(EventLedger, DetailMapIsEmittedAsNestedObject)
{
  EventLedger ledger(2.0);
  std::map<std::string, std::string> detail{{"reason", "watchdog"}, {"code", "OBSTACLE_TOO_CLOSE"}};
  const auto line = ledger.observe(
    "safety", "state", "HOLDING", EventLevel::kWarn, 0.0, 0.0, detail);
  ASSERT_TRUE(line.has_value());
  EXPECT_TRUE(contains(*line, "\"reason\":\"watchdog\""));
  EXPECT_TRUE(contains(*line, "\"code\":\"OBSTACLE_TOO_CLOSE\""));
}

TEST(EventLedger, EmptyDetailMapIsEmptyJsonObject)
{
  EventLedger ledger(2.0);
  const auto line = ledger.observe("safety", "state", "NORMAL", EventLevel::kInfo, 0.0, 0.0);
  ASSERT_TRUE(line.has_value());
  EXPECT_TRUE(contains(*line, "\"detail\":{}"));
}

// ===========================================================================
// Invariant 5: needsFsync -- level >= ERROR always true; otherwise true iff
// fsync_period_sec has elapsed since the last markFsynced().
// ===========================================================================

TEST(NeedsFsync, ErrorLevelAlwaysNeedsFsyncRegardlessOfElapsedTime)
{
  EventLedger ledger(1000.0);   // huge period -- time alone would say "no"
  ledger.markFsynced(0.0);
  EXPECT_TRUE(ledger.needsFsync(EventLevel::kError, 0.001));
}

TEST(NeedsFsync, InfoJustBeforePeriodElapsedDoesNotNeedFsync)
{
  EventLedger ledger(2.0);
  ledger.markFsynced(10.0);
  EXPECT_FALSE(ledger.needsFsync(EventLevel::kInfo, 11.999));
}

TEST(NeedsFsync, InfoAtOrPastPeriodElapsedNeedsFsync)
{
  EventLedger ledger(2.0);
  ledger.markFsynced(10.0);
  EXPECT_TRUE(ledger.needsFsync(EventLevel::kInfo, 12.0));
}

// Mutation counter: markFsynced() must actually reset the clock -- a second
// check right after marking, at the same t_wall, must read false again
// (proves the period is measured from markFsynced, not from construction).
TEST(NeedsFsync, MarkFsyncedResetsThePeriodClock)
{
  EventLedger ledger(2.0);
  ledger.markFsynced(10.0);
  ASSERT_TRUE(ledger.needsFsync(EventLevel::kInfo, 12.0));
  ledger.markFsynced(12.0);
  EXPECT_FALSE(ledger.needsFsync(EventLevel::kInfo, 12.0));
  EXPECT_TRUE(ledger.needsFsync(EventLevel::kInfo, 14.0));
}

TEST(NeedsFsync, WarnLevelUsesTheSameTimeRuleAsInfo)
{
  EventLedger ledger(2.0);
  ledger.markFsynced(0.0);
  EXPECT_FALSE(ledger.needsFsync(EventLevel::kWarn, 1.0));
  EXPECT_TRUE(ledger.needsFsync(EventLevel::kWarn, 2.0));
}

TEST(NeedsFsync, NeverExplicitlyFsyncedYetReadsAsOverdueForARealisticWallClock)
{
  // Safe-by-default (R0): before the first markFsynced(), any realistic
  // wall-clock t_wall (large epoch seconds) already reads as overdue rather
  // than silently waiting a full period from an undefined start.
  EventLedger ledger(5.0);
  EXPECT_TRUE(ledger.needsFsync(EventLevel::kInfo, 1'700'000'000.0));
}

}  // namespace uav_observability
