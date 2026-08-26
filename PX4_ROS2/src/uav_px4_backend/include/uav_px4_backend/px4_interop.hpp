// A duplicated mode table once made RETURN mean ACRO.

#ifndef UAV_PX4_BACKEND__PX4_INTEROP_HPP_
#define UAV_PX4_BACKEND__PX4_INTEROP_HPP_

#include <array>
#include <cstdint>
#include <deque>
#include <optional>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

namespace uav_px4_backend
{
namespace px4
{

// Values mirror PX4 src/modules/commander/px4_custom_mode.h.
constexpr float kCustomModeEnabled = 1.0F;

constexpr float kMainManual = 1.0F;
constexpr float kMainAltitude = 2.0F;
constexpr float kMainPosition = 3.0F;
constexpr float kMainAuto = 4.0F;
constexpr float kMainOffboard = 6.0F;

constexpr float kSubNone = 0.0F;
constexpr float kSubAutoTakeoff = 2.0F;
constexpr float kSubAutoLoiter = 3.0F;
constexpr float kSubAutoMission = 4.0F;
constexpr float kSubAutoRtl = 5.0F;
constexpr float kSubAutoLand = 6.0F;

/// AUTO needs main plus sub mode; main alone picks wrong.
struct FlightModeCommand
{
  bool supported;
  float main_mode;
  float sub_mode;
};

FlightModeCommand toPx4Mode(uint8_t internal_mode);

/// Keeps PX4 numbering out of internal messages.
uint8_t toInternalFlightMode(uint8_t nav_state);

/// Conservative on 2D and extrapolated fixes; see docs/package-status.md.
int8_t toNavSatStatus(uint8_t fix_type);

bool autopilotOwnsTheFlight(uint8_t nav_state, bool failsafe_active);

/// True when a HUMAN is flying on the sticks. Not the inverse of the above.
bool pilotIsFlying(uint8_t nav_state);

/// New value of the "a pilot has this aircraft" latch. Cleared only by disarm.
bool updatePilotOverride(bool latched, uint8_t nav_state, uint8_t arming_state);

bool allFinite(const std::array<float, 3> & values);

/// Fields are NaN wherever PX4 has no usable reading.
struct BatteryHealth
{
  float voltage;
  float current;
  float remaining;
  float temperature;
};

/// Empty means no autopilot data yet; zero would lie.
BatteryHealth toBatteryHealth(const std::optional<px4_msgs::msg::BatteryStatus> & battery);

uint64_t timestampMicros(const rclcpp::Time & stamp);

/// Our clock into PX4's, ~1.79e9 s apart; see docs/package-status.md.
class Px4ClockOffset
{
public:
  explicit Px4ClockOffset(double window_sec = 2.0);

  void observe(uint64_t px4_timestamp_us, uint64_t local_us);

  bool known() const {return !samples_.empty();}

  /// Largest offset wins: transport delay only makes it look smaller.
  int64_t offsetMicros() const;

  uint64_t toPx4Micros(uint64_t local_us) const;

private:
  struct Sample
  {
    uint64_t local_us;
    int64_t offset_us;
  };

  double window_sec_;
  std::deque<Sample> samples_;
};

px4_msgs::msg::VehicleCommand makeVehicleCommand(
  uint16_t command, uint64_t timestamp_us,
  float param1 = 0.0F, float param2 = 0.0F, float param3 = 0.0F);

}  // namespace px4
}  // namespace uav_px4_backend

#endif  // UAV_PX4_BACKEND__PX4_INTEROP_HPP_
