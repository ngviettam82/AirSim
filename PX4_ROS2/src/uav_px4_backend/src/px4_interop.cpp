#include "uav_px4_backend/px4_interop.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>

namespace uav_px4_backend
{
namespace px4
{

using px4_msgs::msg::BatteryStatus;
using px4_msgs::msg::SensorGps;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleStatus;
using sensor_msgs::msg::NavSatStatus;
using uav_interfaces::msg::VehicleState;

FlightModeCommand toPx4Mode(uint8_t internal_mode)
{
  switch (internal_mode) {
    case VehicleState::FLIGHT_MODE_MANUAL: return {true, kMainManual, kSubNone};
    case VehicleState::FLIGHT_MODE_ALTITUDE: return {true, kMainAltitude, kSubNone};
    case VehicleState::FLIGHT_MODE_POSITION: return {true, kMainPosition, kSubNone};
    case VehicleState::FLIGHT_MODE_OFFBOARD: return {true, kMainOffboard, kSubNone};
    case VehicleState::FLIGHT_MODE_HOLD: return {true, kMainAuto, kSubAutoLoiter};
    case VehicleState::FLIGHT_MODE_MISSION: return {true, kMainAuto, kSubAutoMission};
    case VehicleState::FLIGHT_MODE_RETURN: return {true, kMainAuto, kSubAutoRtl};
    case VehicleState::FLIGHT_MODE_LAND: return {true, kMainAuto, kSubAutoLand};
    case VehicleState::FLIGHT_MODE_TAKEOFF: return {true, kMainAuto, kSubAutoTakeoff};
    default: return {false, 0.0F, 0.0F};
  }
}

uint8_t toInternalFlightMode(uint8_t nav_state)
{
  switch (nav_state) {
    case VehicleStatus::NAVIGATION_STATE_MANUAL: return VehicleState::FLIGHT_MODE_MANUAL;
    case VehicleStatus::NAVIGATION_STATE_ALTCTL: return VehicleState::FLIGHT_MODE_ALTITUDE;
    case VehicleStatus::NAVIGATION_STATE_POSCTL: return VehicleState::FLIGHT_MODE_POSITION;
    case VehicleStatus::NAVIGATION_STATE_OFFBOARD: return VehicleState::FLIGHT_MODE_OFFBOARD;
    case VehicleStatus::NAVIGATION_STATE_AUTO_LOITER: return VehicleState::FLIGHT_MODE_HOLD;
    case VehicleStatus::NAVIGATION_STATE_AUTO_MISSION: return VehicleState::FLIGHT_MODE_MISSION;
    case VehicleStatus::NAVIGATION_STATE_AUTO_RTL: return VehicleState::FLIGHT_MODE_RETURN;
    case VehicleStatus::NAVIGATION_STATE_AUTO_LAND: return VehicleState::FLIGHT_MODE_LAND;
    case VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF: return VehicleState::FLIGHT_MODE_TAKEOFF;
    default: return VehicleState::FLIGHT_MODE_UNKNOWN;
  }
}

int8_t toNavSatStatus(uint8_t fix_type)
{
  switch (fix_type) {
    case SensorGps::FIX_TYPE_RTCM_CODE_DIFFERENTIAL:
      return NavSatStatus::STATUS_SBAS_FIX;
    case SensorGps::FIX_TYPE_RTK_FLOAT:
    case SensorGps::FIX_TYPE_RTK_FIXED:
      return NavSatStatus::STATUS_GBAS_FIX;
    case SensorGps::FIX_TYPE_3D:
      return NavSatStatus::STATUS_FIX;
    // 2D has no altitude; extrapolated is dead reckoning.
    default:
      return NavSatStatus::STATUS_NO_FIX;
  }
}

bool autopilotOwnsTheFlight(uint8_t nav_state, bool failsafe_active)
{
  if (failsafe_active) {
    return true;
  }
  switch (nav_state) {
    case VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
    case VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
    case VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
    case VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
    case VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND:
    case VehicleStatus::NAVIGATION_STATE_DESCEND:
    case VehicleStatus::NAVIGATION_STATE_TERMINATION:
      return true;
    default:
      return false;
  }
}

// Deliberately NOT the inverse of autopilotOwnsTheFlight(). That one is asked before
// engaging and must stay false for pilot modes, because every flight engages offboard
// out of a POSCTL/MANUAL ground state. This one is asked after we LOSE offboard, where
// the question is whether a human took the aircraft -- and taking it back from them is
// the one thing the stack must never do.
bool pilotIsFlying(uint8_t nav_state)
{
  switch (nav_state) {
    case VehicleStatus::NAVIGATION_STATE_MANUAL:
    case VehicleStatus::NAVIGATION_STATE_ALTCTL:
    case VehicleStatus::NAVIGATION_STATE_POSCTL:
    case VehicleStatus::NAVIGATION_STATE_POSITION_SLOW:
    case VehicleStatus::NAVIGATION_STATE_ACRO:
    case VehicleStatus::NAVIGATION_STATE_STAB:
      return true;
    default:
      return false;
  }
}

// Order matters and is the whole content of this function. DISARMED is checked first so
// the ground state -- which is POSCTL or MANUAL on every flight -- can never leave the
// latch set; otherwise the stack would refuse to engage offboard and no flight would
// ever start. Once ARMED, seeing a stick-flown mode means a human is holding this
// aircraft in the air, and the latch stays until they put it down.
bool updatePilotOverride(bool latched, uint8_t nav_state, uint8_t arming_state)
{
  if (arming_state == VehicleStatus::ARMING_STATE_DISARMED) {
    return false;
  }
  if (pilotIsFlying(nav_state)) {
    return true;
  }
  return latched;
}

bool allFinite(const std::array<float, 3> & values)
{
  return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

BatteryHealth toBatteryHealth(const std::optional<BatteryStatus> & battery)
{
  constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
  BatteryHealth health{kNan, kNan, kNan, kNan};
  if (!battery) {
    return health;
  }

  // PX4 sends 0 V and remaining -1 when unknown.
  if (std::isfinite(battery->voltage_filtered_v) && battery->voltage_filtered_v > 0.0F) {
    health.voltage = battery->voltage_filtered_v;
  }
  if (std::isfinite(battery->current_filtered_a)) {
    health.current = battery->current_filtered_a;
  }
  if (battery->remaining >= 0.0F && battery->remaining <= 1.0F) {
    health.remaining = battery->remaining;
  }
  if (std::isfinite(battery->temperature)) {
    health.temperature = battery->temperature;
  }
  return health;
}

uint64_t timestampMicros(const rclcpp::Time & stamp)
{
  return static_cast<uint64_t>(stamp.nanoseconds() / 1000);
}

Px4ClockOffset::Px4ClockOffset(double window_sec)
: window_sec_(window_sec)
{
}

void Px4ClockOffset::observe(uint64_t px4_timestamp_us, uint64_t local_us)
{
  if (px4_timestamp_us == 0) {
    return;
  }

  samples_.push_back(
    Sample{local_us, static_cast<int64_t>(px4_timestamp_us) - static_cast<int64_t>(local_us)});

  const auto window_us = static_cast<uint64_t>(window_sec_ * 1e6);
  while (samples_.size() > 1 && local_us - samples_.front().local_us > window_us) {
    samples_.pop_front();
  }
}

int64_t Px4ClockOffset::offsetMicros() const
{
  int64_t largest = std::numeric_limits<int64_t>::min();
  for (const Sample & sample : samples_) {
    largest = std::max(largest, sample.offset_us);
  }
  return samples_.empty() ? 0 : largest;
}

uint64_t Px4ClockOffset::toPx4Micros(uint64_t local_us) const
{
  const int64_t converted = static_cast<int64_t>(local_us) + offsetMicros();
  return converted < 0 ? 0U : static_cast<uint64_t>(converted);
}

VehicleCommand makeVehicleCommand(
  uint16_t command, uint64_t timestamp_us, float param1, float param2, float param3)
{
  VehicleCommand msg{};
  msg.command = command;
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = timestamp_us;
  return msg;
}

}  // namespace px4
}  // namespace uav_px4_backend
