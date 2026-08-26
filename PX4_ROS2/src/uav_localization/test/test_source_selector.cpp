#include <vector>

#include <gtest/gtest.h>
#include <uav_interfaces/msg/localization_status.hpp>

#include "uav_localization/source_selector.hpp"

using uav_interfaces::msg::LocalizationStatus;
using uav_localization::SelectionPolicy;
using uav_localization::SourceSelector;
using uav_localization::SourceSnapshot;

namespace
{

constexpr uint8_t kGps = LocalizationStatus::SOURCE_GPS;
constexpr uint8_t kVio = LocalizationStatus::SOURCE_VIO;
constexpr uint8_t kNone = LocalizationStatus::SOURCE_NONE;

SourceSnapshot snapshot(uint8_t id, bool eligible, double stddev)
{
  SourceSnapshot value;
  value.source_id = id;
  value.eligible = eligible;
  value.position_stddev = stddev;
  return value;
}

SourceSelector defaultSelector()
{
  SourceSelector selector;
  selector.configure(SelectionPolicy{0.7, 1.0});
  return selector;
}

TEST(SourceSelector, PicksNothingWhenNothingIsUsable)
{
  auto selector = defaultSelector();

  const uint8_t chosen = selector.select(
    {snapshot(kGps, false, 0.9), snapshot(kVio, false, 0.1)}, 0.0);

  EXPECT_EQ(chosen, kNone);
}

TEST(SourceSelector, StartsOnTheMoreCertainSource)
{
  auto selector = defaultSelector();

  const uint8_t chosen = selector.select(
    {snapshot(kGps, true, 0.9), snapshot(kVio, true, 0.1)}, 0.0);

  EXPECT_EQ(chosen, kVio);
  EXPECT_TRUE(selector.justSwitched());
}

TEST(SourceSelector, RejectsASourceThatCannotStateItsUncertainty)
{
  auto selector = defaultSelector();

  const uint8_t chosen = selector.select(
    {snapshot(kGps, true, -1.0), snapshot(kVio, true, 0.4)}, 0.0);

  EXPECT_EQ(chosen, kVio);
}

TEST(SourceSelector, DoesNotSwitchForAMarginalImprovement)
{
  auto selector = defaultSelector();
  selector.select({snapshot(kGps, true, 0.5), snapshot(kVio, true, 9.0)}, 0.0);

  double time_sec = 0.0;
  for (int step = 0; step < 100; ++step) {
    time_sec += 0.1;
    selector.select({snapshot(kGps, true, 0.5), snapshot(kVio, true, 0.4)}, time_sec);
  }

  EXPECT_EQ(selector.active(), kGps) << "0.4 vs 0.5 is not worth a discontinuity";
}

TEST(SourceSelector, SwitchesWhenTheRivalIsClearlyAndPersistentlyBetter)
{
  auto selector = defaultSelector();
  selector.select({snapshot(kGps, true, 0.5), snapshot(kVio, true, 9.0)}, 0.0);

  selector.select({snapshot(kGps, true, 0.5), snapshot(kVio, true, 0.1)}, 1.0);
  EXPECT_EQ(selector.active(), kGps) << "must stay put until the rival proves itself";

  const uint8_t chosen =
    selector.select({snapshot(kGps, true, 0.5), snapshot(kVio, true, 0.1)}, 2.1);

  EXPECT_EQ(chosen, kVio);
  EXPECT_TRUE(selector.justSwitched());
}

TEST(SourceSelector, ABriefAdvantageDoesNotEarnASwitch)
{
  auto selector = defaultSelector();
  selector.select({snapshot(kGps, true, 0.5), snapshot(kVio, true, 9.0)}, 0.0);

  selector.select({snapshot(kGps, true, 0.5), snapshot(kVio, true, 0.1)}, 1.0);
  selector.select({snapshot(kGps, true, 0.5), snapshot(kVio, true, 9.0)}, 1.4);
  selector.select({snapshot(kGps, true, 0.5), snapshot(kVio, true, 0.1)}, 1.8);
  const uint8_t chosen =
    selector.select({snapshot(kGps, true, 0.5), snapshot(kVio, true, 0.1)}, 2.5);

  EXPECT_EQ(chosen, kGps) << "the hold time restarts when the advantage lapses";
}

TEST(SourceSelector, LeavesAFailedSourceImmediately)
{
  auto selector = defaultSelector();
  selector.select({snapshot(kGps, true, 0.2), snapshot(kVio, true, 9.0)}, 0.0);
  ASSERT_EQ(selector.active(), kGps);

  const uint8_t chosen =
    selector.select({snapshot(kGps, false, 0.2), snapshot(kVio, true, 9.0)}, 0.1);

  EXPECT_EQ(chosen, kVio);
  EXPECT_TRUE(selector.justSwitched());
}

TEST(SourceSelector, ReportsNoSourceWhenTheLastOneDies)
{
  auto selector = defaultSelector();
  selector.select({snapshot(kGps, true, 0.2)}, 0.0);

  const uint8_t chosen = selector.select({snapshot(kGps, false, 0.2)}, 0.1);

  EXPECT_EQ(chosen, kNone);
  EXPECT_FALSE(selector.justSwitched()) << "losing everything is not a switch";
}

TEST(SourceSelector, StayingPutIsNotReportedAsASwitch)
{
  auto selector = defaultSelector();
  selector.select({snapshot(kGps, true, 0.2), snapshot(kVio, true, 0.9)}, 0.0);

  selector.select({snapshot(kGps, true, 0.2), snapshot(kVio, true, 0.9)}, 1.0);

  EXPECT_FALSE(selector.justSwitched());
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
