// Pins PX4 numbers that the compiler cannot check.

#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>

#include "uav_px4_backend/px4_interop.hpp"

using px4_msgs::msg::BatteryStatus;
using px4_msgs::msg::SensorGps;
using px4_msgs::msg::VehicleStatus;
using sensor_msgs::msg::NavSatStatus;
using uav_interfaces::msg::VehicleState;
namespace px4 = uav_px4_backend::px4;

TEST(Px4Modes, ManualStyleModesCarryNoSubMode)
{
  for (const uint8_t mode : {VehicleState::FLIGHT_MODE_MANUAL,
      VehicleState::FLIGHT_MODE_ALTITUDE,
      VehicleState::FLIGHT_MODE_POSITION,
      VehicleState::FLIGHT_MODE_OFFBOARD})
  {
    const auto encoded = px4::toPx4Mode(mode);
    ASSERT_TRUE(encoded.supported) << "mode " << static_cast<int>(mode);
    EXPECT_FLOAT_EQ(encoded.sub_mode, px4::kSubNone) << "mode " << static_cast<int>(mode);
  }
}

TEST(Px4Modes, EveryAutoModeSendsMainAutoPlusItsSubMode)
{
  const std::vector<std::pair<uint8_t, float>> expected{
    {VehicleState::FLIGHT_MODE_HOLD, px4::kSubAutoLoiter},
    {VehicleState::FLIGHT_MODE_MISSION, px4::kSubAutoMission},
    {VehicleState::FLIGHT_MODE_RETURN, px4::kSubAutoRtl},
    {VehicleState::FLIGHT_MODE_LAND, px4::kSubAutoLand},
    {VehicleState::FLIGHT_MODE_TAKEOFF, px4::kSubAutoTakeoff},
  };

  for (const auto & [internal_mode, sub_mode] : expected) {
    const auto encoded = px4::toPx4Mode(internal_mode);
    ASSERT_TRUE(encoded.supported) << "mode " << static_cast<int>(internal_mode);
    EXPECT_FLOAT_EQ(encoded.main_mode, px4::kMainAuto)
      << "mode " << static_cast<int>(internal_mode);
    EXPECT_FLOAT_EQ(encoded.sub_mode, sub_mode)
      << "mode " << static_cast<int>(internal_mode);
  }
}

// RETURN was once encoded as main mode 5: ACRO.
TEST(Px4Modes, ReturnIsNotAcro)
{
  const auto encoded = px4::toPx4Mode(VehicleState::FLIGHT_MODE_RETURN);
  constexpr float kMainAcro = 5.0F;
  EXPECT_NE(encoded.main_mode, kMainAcro);
  EXPECT_FLOAT_EQ(encoded.main_mode, px4::kMainAuto);
  EXPECT_FLOAT_EQ(encoded.sub_mode, px4::kSubAutoRtl);
}

TEST(Px4Modes, SupportedModesAllEncodeDifferently)
{
  const std::vector<uint8_t> modes{
    VehicleState::FLIGHT_MODE_MANUAL, VehicleState::FLIGHT_MODE_ALTITUDE,
    VehicleState::FLIGHT_MODE_POSITION, VehicleState::FLIGHT_MODE_OFFBOARD,
    VehicleState::FLIGHT_MODE_HOLD, VehicleState::FLIGHT_MODE_MISSION,
    VehicleState::FLIGHT_MODE_RETURN, VehicleState::FLIGHT_MODE_LAND,
    VehicleState::FLIGHT_MODE_TAKEOFF};

  std::set<std::pair<float, float>> seen;
  for (const uint8_t mode : modes) {
    const auto encoded = px4::toPx4Mode(mode);
    ASSERT_TRUE(encoded.supported);
    EXPECT_TRUE(seen.insert({encoded.main_mode, encoded.sub_mode}).second)
      << "duplicate encoding for mode " << static_cast<int>(mode);
  }
}

TEST(Px4Modes, UnsupportedModeIsRejectedRatherThanGuessed)
{
  EXPECT_FALSE(px4::toPx4Mode(VehicleState::FLIGHT_MODE_UNKNOWN).supported);
  EXPECT_FALSE(px4::toPx4Mode(200).supported);
}

// Disagreement here reports one mode while flying another.
TEST(Px4Modes, CommandAndReportAgreeOnEveryMode)
{
  const std::vector<std::pair<uint8_t, uint8_t>> mode_for_nav_state{
    {VehicleStatus::NAVIGATION_STATE_MANUAL, VehicleState::FLIGHT_MODE_MANUAL},
    {VehicleStatus::NAVIGATION_STATE_ALTCTL, VehicleState::FLIGHT_MODE_ALTITUDE},
    {VehicleStatus::NAVIGATION_STATE_POSCTL, VehicleState::FLIGHT_MODE_POSITION},
    {VehicleStatus::NAVIGATION_STATE_OFFBOARD, VehicleState::FLIGHT_MODE_OFFBOARD},
    {VehicleStatus::NAVIGATION_STATE_AUTO_LOITER, VehicleState::FLIGHT_MODE_HOLD},
    {VehicleStatus::NAVIGATION_STATE_AUTO_MISSION, VehicleState::FLIGHT_MODE_MISSION},
    {VehicleStatus::NAVIGATION_STATE_AUTO_RTL, VehicleState::FLIGHT_MODE_RETURN},
    {VehicleStatus::NAVIGATION_STATE_AUTO_LAND, VehicleState::FLIGHT_MODE_LAND},
    {VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF, VehicleState::FLIGHT_MODE_TAKEOFF},
  };

  for (const auto & [nav_state, internal_mode] : mode_for_nav_state) {
    EXPECT_EQ(px4::toInternalFlightMode(nav_state), internal_mode)
      << "nav_state " << static_cast<int>(nav_state);
    EXPECT_TRUE(px4::toPx4Mode(internal_mode).supported)
      << "internal mode " << static_cast<int>(internal_mode);
  }
}

TEST(Px4Modes, UnmappedNavStateReportsUnknownRatherThanAWrongMode)
{
  EXPECT_EQ(
    px4::toInternalFlightMode(VehicleStatus::NAVIGATION_STATE_ACRO),
    VehicleState::FLIGHT_MODE_UNKNOWN);
  EXPECT_EQ(px4::toInternalFlightMode(250), VehicleState::FLIGHT_MODE_UNKNOWN);
}

TEST(AutopilotAuthority, FailsafeAlwaysOutranksTheNavState)
{
  EXPECT_TRUE(
    px4::autopilotOwnsTheFlight(VehicleStatus::NAVIGATION_STATE_POSCTL, true));
}

TEST(AutopilotAuthority, AutonomousManoeuvresAreNotInterrupted)
{
  for (const uint8_t nav_state : {VehicleStatus::NAVIGATION_STATE_AUTO_LAND,
      VehicleStatus::NAVIGATION_STATE_AUTO_RTL,
      VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF,
      VehicleStatus::NAVIGATION_STATE_AUTO_MISSION,
      VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND,
      VehicleStatus::NAVIGATION_STATE_DESCEND,
      VehicleStatus::NAVIGATION_STATE_TERMINATION})
  {
    EXPECT_TRUE(px4::autopilotOwnsTheFlight(nav_state, false))
      << "nav_state " << static_cast<int>(nav_state);
  }
}

TEST(AutopilotAuthority, PilotModesLeaveControlAvailable)
{
  for (const uint8_t nav_state : {VehicleStatus::NAVIGATION_STATE_MANUAL,
      VehicleStatus::NAVIGATION_STATE_POSCTL,
      VehicleStatus::NAVIGATION_STATE_ALTCTL,
      VehicleStatus::NAVIGATION_STATE_AUTO_LOITER,
      VehicleStatus::NAVIGATION_STATE_OFFBOARD})
  {
    EXPECT_FALSE(px4::autopilotOwnsTheFlight(nav_state, false))
      << "nav_state " << static_cast<int>(nav_state);
  }
}

// P11.6 (RC override). The test above pins autopilotOwnsTheFlight() FALSE for pilot
// modes, and that is right: it answers "may we engage offboard from here?", and the
// stack engages from a POSCTL/MANUAL ground state on every single flight. But "control
// is available" is a different question from "is a human flying right now", and reading
// the first as an answer to the second is what let the offboard session manager plan to
// re-take an aircraft a pilot had just grabbed with the sticks.
TEST(PilotIsFlying, TrueForEveryStickFlownMode)
{
  for (const uint8_t nav_state : {VehicleStatus::NAVIGATION_STATE_MANUAL,
      VehicleStatus::NAVIGATION_STATE_ALTCTL,
      VehicleStatus::NAVIGATION_STATE_POSCTL,
      VehicleStatus::NAVIGATION_STATE_POSITION_SLOW,
      VehicleStatus::NAVIGATION_STATE_ACRO,
      VehicleStatus::NAVIGATION_STATE_STAB})
  {
    EXPECT_TRUE(px4::pilotIsFlying(nav_state))
      << "nav_state " << static_cast<int>(nav_state);
  }
}

TEST(PilotIsFlying, FalseWhenTheAutopilotOrWeAreFlying)
{
  for (const uint8_t nav_state : {VehicleStatus::NAVIGATION_STATE_OFFBOARD,
      VehicleStatus::NAVIGATION_STATE_AUTO_LOITER,
      VehicleStatus::NAVIGATION_STATE_AUTO_RTL,
      VehicleStatus::NAVIGATION_STATE_AUTO_LAND,
      VehicleStatus::NAVIGATION_STATE_AUTO_MISSION,
      VehicleStatus::NAVIGATION_STATE_TERMINATION})
  {
    EXPECT_FALSE(px4::pilotIsFlying(nav_state))
      << "nav_state " << static_cast<int>(nav_state);
  }
}

// The two predicates answer different questions but must never both say yes: one means
// "stand off, the autopilot is mid-manoeuvre", the other "stand off, a human has it".
// A nav_state satisfying both would mean one of the two classifications is wrong, and
// the node reads them in sequence, so the second would silently never be reached.
TEST(PilotIsFlying, NeverAgreesWithAutopilotOwnership)
{
  for (uint8_t nav_state = 0; nav_state < 25; ++nav_state) {
    EXPECT_FALSE(px4::pilotIsFlying(nav_state) && px4::autopilotOwnsTheFlight(nav_state, false))
      << "nav_state " << static_cast<int>(nav_state);
  }
}

// The latch that stops the stack taking an aircraft back from a pilot. Every flight
// starts on the ground in POSCTL or MANUAL, so the disarm branch is what keeps this
// from bricking autonomy entirely -- these tests exist mainly to pin that.
TEST(PilotOverrideLatch, GroundStateNeverLatchesEvenInAPilotMode)
{
  EXPECT_FALSE(
    px4::updatePilotOverride(
      false, VehicleStatus::NAVIGATION_STATE_POSCTL, VehicleStatus::ARMING_STATE_DISARMED));
  EXPECT_FALSE(
    px4::updatePilotOverride(
      false, VehicleStatus::NAVIGATION_STATE_MANUAL, VehicleStatus::ARMING_STATE_DISARMED));
}

TEST(PilotOverrideLatch, DisarmClearsAnExistingLatch)
{
  EXPECT_FALSE(
    px4::updatePilotOverride(
      true, VehicleStatus::NAVIGATION_STATE_POSCTL, VehicleStatus::ARMING_STATE_DISARMED));
}

TEST(PilotOverrideLatch, ArmedAndStickFlownLatches)
{
  EXPECT_TRUE(
    px4::updatePilotOverride(
      false, VehicleStatus::NAVIGATION_STATE_POSCTL, VehicleStatus::ARMING_STATE_ARMED));
}

TEST(PilotOverrideLatch, StaysLatchedWhileStillArmed)
{
  // The decisive case. After the takeover PX4 may report any mode at all -- what must
  // NOT happen is the latch dropping the moment the nav_state stops being a pilot mode,
  // because that is precisely when auto_engage would grab the aircraft back.
  EXPECT_TRUE(
    px4::updatePilotOverride(
      true, VehicleStatus::NAVIGATION_STATE_AUTO_LOITER, VehicleStatus::ARMING_STATE_ARMED));
  EXPECT_TRUE(
    px4::updatePilotOverride(
      true, VehicleStatus::NAVIGATION_STATE_OFFBOARD, VehicleStatus::ARMING_STATE_ARMED));
}

TEST(PilotOverrideLatch, ArmedAutonomousFlightDoesNotLatch)
{
  EXPECT_FALSE(
    px4::updatePilotOverride(
      false, VehicleStatus::NAVIGATION_STATE_OFFBOARD, VehicleStatus::ARMING_STATE_ARMED));
  EXPECT_FALSE(
    px4::updatePilotOverride(
      false, VehicleStatus::NAVIGATION_STATE_AUTO_LAND, VehicleStatus::ARMING_STATE_ARMED));
}

TEST(AllFinite, RejectsAnyNonFiniteComponent)
{
  constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
  constexpr float kInf = std::numeric_limits<float>::infinity();

  EXPECT_TRUE(px4::allFinite({0.0F, -1.5F, 42.0F}));
  EXPECT_FALSE(px4::allFinite({kNan, 0.0F, 0.0F}));
  EXPECT_FALSE(px4::allFinite({0.0F, kInf, 0.0F}));
  EXPECT_FALSE(px4::allFinite({0.0F, 0.0F, -kInf}));
}

// A published 0.0 once read as BATTERY_CRITICAL at boot.
TEST(BatteryHealth, ReportsNothingBeforeTheFirstAutopilotMessage)
{
  const auto health = px4::toBatteryHealth(std::nullopt);

  EXPECT_TRUE(std::isnan(health.remaining)) << "zero would read as an empty battery";
  EXPECT_TRUE(std::isnan(health.voltage));
  EXPECT_TRUE(std::isnan(health.current));
  EXPECT_TRUE(std::isnan(health.temperature));
}

TEST(BatteryHealth, PassesThroughAReadingTheAutopilotActuallySent)
{
  BatteryStatus battery{};
  battery.voltage_filtered_v = 15.2F;
  battery.current_filtered_a = 8.4F;
  battery.remaining = 0.73F;
  battery.temperature = 27.5F;

  const auto health = px4::toBatteryHealth(battery);

  EXPECT_FLOAT_EQ(health.voltage, 15.2F);
  EXPECT_FLOAT_EQ(health.current, 8.4F);
  EXPECT_FLOAT_EQ(health.remaining, 0.73F);
  EXPECT_FLOAT_EQ(health.temperature, 27.5F);
}

// PX4 documents these as "unknown"; -1 is below every threshold.
TEST(BatteryHealth, RejectsThePx4UnknownSentinels)
{
  BatteryStatus battery{};
  battery.voltage_filtered_v = 0.0F;
  battery.remaining = -1.0F;
  battery.temperature = std::numeric_limits<float>::quiet_NaN();

  const auto health = px4::toBatteryHealth(battery);

  EXPECT_TRUE(std::isnan(health.voltage));
  EXPECT_TRUE(std::isnan(health.remaining));
  EXPECT_TRUE(std::isnan(health.temperature));
}

// Golden-value pin (P8 checkpoint T3): mirrored by uav_safety's
// Battery.SentinelNanChainedFromBackendNeverReadsAsCritical (R1 forbids that
// test calling toBatteryHealth() directly).
TEST(BatteryHealth, GoldenSentinelChainedToSafetysNanContract)
{
  BatteryStatus battery{};
  battery.voltage_filtered_v = 0.0F;   // PX4's own "unknown" sentinel
  battery.remaining = -1.0F;           // PX4's own "unknown" sentinel

  const auto health = px4::toBatteryHealth(battery);

  EXPECT_TRUE(std::isnan(health.remaining))
    << "this exact NaN must reach uav_safety as CANNOT_MEASURE, never BATTERY_CRITICAL";
}

TEST(BatteryHealth, StillReportsAGenuinelyEmptyBattery)
{
  BatteryStatus battery{};
  battery.remaining = 0.0F;

  EXPECT_FLOAT_EQ(px4::toBatteryHealth(battery).remaining, 0.0F);
}

TEST(BatteryHealth, RefusesAChargeFractionOutsideItsRange)
{
  BatteryStatus battery{};
  battery.remaining = 1.5F;
  EXPECT_TRUE(std::isnan(px4::toBatteryHealth(battery).remaining));

  battery.remaining = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(std::isnan(px4::toBatteryHealth(battery).remaining));
}

TEST(TimestampMicros, ConvertsNanosecondsToMicroseconds)
{
  const rclcpp::Time stamp(0, 1500, RCL_ROS_TIME);
  EXPECT_EQ(px4::timestampMicros(stamp), 1U);

  const rclcpp::Time one_second(1, 0, RCL_ROS_TIME);
  EXPECT_EQ(px4::timestampMicros(one_second), 1000000U);
}

TEST(NavSatStatusMapping, OnlyAThreeDimensionalFixCountsAsAFix)
{
  EXPECT_EQ(px4::toNavSatStatus(SensorGps::FIX_TYPE_3D), NavSatStatus::STATUS_FIX);
  EXPECT_EQ(
    px4::toNavSatStatus(SensorGps::FIX_TYPE_RTCM_CODE_DIFFERENTIAL),
    NavSatStatus::STATUS_SBAS_FIX);
  EXPECT_EQ(px4::toNavSatStatus(SensorGps::FIX_TYPE_RTK_FLOAT), NavSatStatus::STATUS_GBAS_FIX);
  EXPECT_EQ(px4::toNavSatStatus(SensorGps::FIX_TYPE_RTK_FIXED), NavSatStatus::STATUS_GBAS_FIX);
}

TEST(NavSatStatusMapping, TreatsAltitudelessAndDeadReckonedFixesAsNoFix)
{
  EXPECT_EQ(px4::toNavSatStatus(SensorGps::FIX_TYPE_2D), NavSatStatus::STATUS_NO_FIX);
  EXPECT_EQ(px4::toNavSatStatus(SensorGps::FIX_TYPE_EXTRAPOLATED), NavSatStatus::STATUS_NO_FIX);
}

TEST(NavSatStatusMapping, ReportsNoFixForBothWaysPx4SaysItHasNone)
{
  // PX4 documents 0 and FIX_TYPE_NONE as equally meaning "no fix".
  EXPECT_EQ(px4::toNavSatStatus(0), NavSatStatus::STATUS_NO_FIX);
  EXPECT_EQ(px4::toNavSatStatus(SensorGps::FIX_TYPE_NONE), NavSatStatus::STATUS_NO_FIX);
}

TEST(Px4ClockOffset, KnowsNothingUntilItHasSeenPx4)
{
  const px4::Px4ClockOffset offset;

  EXPECT_FALSE(offset.known());
}

// The real failure: ours 78 s, PX4 1786364504 s.
TEST(Px4ClockOffset, TranslatesOurClockIntoPx4s)
{
  px4::Px4ClockOffset offset;
  offset.observe(1786364504612828ULL, 78884000ULL);

  ASSERT_TRUE(offset.known());
  EXPECT_EQ(offset.toPx4Micros(78884000ULL), 1786364504612828ULL);
  EXPECT_EQ(offset.toPx4Micros(78885000ULL), 1786364504613828ULL);
}

// Transport delay can only make a measured offset look smaller.
TEST(Px4ClockOffset, PrefersTheLeastDelayedSample)
{
  px4::Px4ClockOffset offset;
  offset.observe(1000000ULL, 100000ULL);   // offset 900000, delayed
  offset.observe(1100000ULL, 150000ULL);   // offset 950000, prompt
  offset.observe(1200000ULL, 320000ULL);   // offset 880000, very delayed

  EXPECT_EQ(offset.offsetMicros(), 950000);
}

TEST(Px4ClockOffset, ForgetsSamplesOlderThanItsWindow)
{
  px4::Px4ClockOffset offset(1.0);
  offset.observe(9000000ULL, 1000000ULL);   // offset 8000000
  offset.observe(4000000ULL, 3000000ULL);   // 2 s later, offset 1000000

  EXPECT_EQ(offset.offsetMicros(), 1000000) << "a stale offset must not linger";
}

TEST(Px4ClockOffset, IgnoresAnUnstampedMessage)
{
  px4::Px4ClockOffset offset;
  offset.observe(0ULL, 500000ULL);

  EXPECT_FALSE(offset.known());
}

TEST(VehicleCommandBuilder, MarksTheCommandAsExternallyOriginated)
{
  const auto msg = px4::makeVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 12345U,
    px4::kCustomModeEnabled, px4::kMainOffboard);

  EXPECT_TRUE(msg.from_external);
  EXPECT_EQ(msg.target_system, 1U);
  EXPECT_EQ(msg.target_component, 1U);
  EXPECT_EQ(msg.timestamp, 12345U);
  EXPECT_FLOAT_EQ(msg.param1, px4::kCustomModeEnabled);
  EXPECT_FLOAT_EQ(msg.param2, px4::kMainOffboard);
  EXPECT_FLOAT_EQ(msg.param3, 0.0F);
}
