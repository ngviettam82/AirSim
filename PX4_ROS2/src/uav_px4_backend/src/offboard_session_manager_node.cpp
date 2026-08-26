// Owns the offboard handover: prime, request, verify.

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/offboard_status.hpp>

#include "uav_px4_backend/px4_interop.hpp"

namespace uav_px4_backend
{

using namespace std::chrono_literals;

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleControlMode;
using px4_msgs::msg::VehicleStatus;
using uav_interfaces::msg::OffboardStatus;

/// Well under the rate window, so stalls surface promptly.
constexpr auto kSupervisionInterval = 200ms;

class OffboardSessionManagerNode : public rclcpp::Node
{
public:
  OffboardSessionManagerNode()
  : Node("offboard_session_manager_node")
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    priming_cycles_ = declare_parameter<int>("priming_cycles", 10);
    min_stream_rate_hz_ = declare_parameter<double>("min_stream_rate_hz", 2.0);
    auto_engage_ = declare_parameter<bool>("auto_engage", true);
    engage_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("engage_timeout_sec", 2.0));

    rclcpp::QoS px4_qos(rclcpp::KeepLast(10));
    px4_qos.best_effort();
    px4_qos.transient_local();

    status_subscription_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos,
      [this](const VehicleStatus::SharedPtr msg) {
        nav_state_ = msg->nav_state;
        failsafe_ = msg->failsafe;
        // vehicle_status arrives at ~2 Hz, so a takeover shorter than one sample would
        // not be seen. A real RC override holds the aircraft far longer than 500 ms;
        // documented as an edge to re-check on hardware, not a live hazard (P11.6).
        pilot_override_ = px4::updatePilotOverride(
          pilot_override_, msg->nav_state, msg->arming_state);
      });

    control_mode_subscription_ = create_subscription<VehicleControlMode>(
      "/fmu/out/vehicle_control_mode", px4_qos,
      [this](const VehicleControlMode::SharedPtr msg) {
        offboard_confirmed_ = msg->flag_control_offboard_enabled;
      });

    // Counting the real stream beats trusting any status message.
    stream_subscription_ = create_subscription<OffboardControlMode>(
      "/fmu/in/offboard_control_mode", rclcpp::QoS(10),
      [this](const OffboardControlMode::SharedPtr) {
        ++stream_count_;
        ++stream_count_in_window_;
      });

    vehicle_command_publisher_ =
      create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    status_publisher_ = create_publisher<OffboardStatus>(
      "/uav/" + uav_id_ + "/backend/offboard_status", rclcpp::QoS(10));

    window_start_ = now();
    // PX4 judges this rate on its own clock.
    supervise_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration(kSupervisionInterval),
      [this]() {supervise();});

    RCLCPP_INFO(get_logger(), "offboard session manager ready for %s", uav_id_.c_str());
  }

private:
  void supervise()
  {
    updateStreamRate();

    switch (state_) {
      case OffboardStatus::STATE_IDLE:
        if (measured_rate_hz_ >= min_stream_rate_hz_) {
          state_ = OffboardStatus::STATE_STREAMING;
          detail_ = "setpoint stream detected";
        }
        break;

      case OffboardStatus::STATE_STREAMING:
        if (measured_rate_hz_ < min_stream_rate_hz_) {
          state_ = OffboardStatus::STATE_IDLE;
          resetPriming();
          detail_ = "stream stopped";
        } else if (offboard_confirmed_) {
          state_ = OffboardStatus::STATE_ACTIVE;
          detail_ = "offboard already active";
        } else if (readyToEngage()) {
          requestOffboard();
        }
        break;

      case OffboardStatus::STATE_ENGAGING:
        if (offboard_confirmed_) {
          state_ = OffboardStatus::STATE_ACTIVE;
          detail_ = "offboard confirmed by autopilot";
        } else if (now() - engage_requested_at_ > engage_timeout_) {
          state_ = OffboardStatus::STATE_STREAMING;
          detail_ = "mode switch not confirmed, will retry";
        }
        break;

      case OffboardStatus::STATE_ACTIVE:
        // Leaving offboard always costs the priming credit.
        if (!offboard_confirmed_) {
          resetPriming();
          if (autopilotOwnsTheFlight()) {
            // A commanded land or return home is not a fault.
            state_ = OffboardStatus::STATE_STREAMING;
            detail_ = "autopilot owns the flight, standing by";
          } else if (pilot_override_) {
            // P11.6, RC override. A human moved the sticks; PX4 handed them the
            // aircraft. Calling that a fault was wrong twice over -- it blames the
            // autopilot for a deliberate act, and it leaves auto_engage free to take
            // the aircraft BACK the moment a setpoint stream reappears. The latch is
            // set in the status callback and holds until disarm.
            state_ = OffboardStatus::STATE_STREAMING;
            detail_ = "pilot has the aircraft, standing down until disarm";
          } else {
            state_ = OffboardStatus::STATE_FAULT;
            detail_ = "autopilot left offboard mode unexpectedly";
          }
        } else if (measured_rate_hz_ < min_stream_rate_hz_) {
          state_ = OffboardStatus::STATE_FAULT;
          resetPriming();
          detail_ = "setpoint rate below safe minimum";
        }
        break;

      case OffboardStatus::STATE_FAULT:
        if (measured_rate_hz_ < min_stream_rate_hz_) {
          state_ = OffboardStatus::STATE_IDLE;
          resetPriming();
          detail_ = "stream stopped, ready for next flight";
        } else if (offboard_confirmed_) {
          state_ = OffboardStatus::STATE_ACTIVE;
          detail_ = "recovered";
        }
        break;

      default:
        break;
    }

    publishStatus();
  }

  /// Offboard before arm: waiting on armed deadlocks indoors.
  bool readyToEngage() const
  {
    return auto_engage_ && !autopilotOwnsTheFlight() && !pilot_override_ &&
           stream_count_ >= static_cast<uint64_t>(priming_cycles_);
  }

  /// Never take control from a landing, RTL, or failsafe.
  bool autopilotOwnsTheFlight() const
  {
    return px4::autopilotOwnsTheFlight(nav_state_, failsafe_);
  }

  void requestOffboard()
  {
    vehicle_command_publisher_->publish(
      px4::makeVehicleCommand(
        VehicleCommand::VEHICLE_CMD_DO_SET_MODE, px4::timestampMicros(now()),
        px4::kCustomModeEnabled, px4::kMainOffboard));

    engage_requested_at_ = now();
    state_ = OffboardStatus::STATE_ENGAGING;
    detail_ = "mode switch requested";
    RCLCPP_INFO(
      get_logger(), "requesting offboard after %lu primed setpoints",
      static_cast<unsigned long>(stream_count_.load()));
  }

  /// A stale count would let the next entry skip priming.
  void resetPriming()
  {
    stream_count_ = 0;
  }

  void updateStreamRate()
  {
    const double window_sec = (now() - window_start_).seconds();
    if (window_sec < 1.0) {
      return;
    }
    measured_rate_hz_ = static_cast<double>(stream_count_in_window_) / window_sec;
    stream_count_in_window_ = 0;
    window_start_ = now();
  }

  void publishStatus()
  {
    OffboardStatus status;
    status.header.stamp = now();
    status.uav_id = uav_id_;
    status.state = state_;
    status.setpoint_rate_hz = static_cast<float>(measured_rate_hz_);
    status.command_fresh = measured_rate_hz_ >= min_stream_rate_hz_;
    status.offboard_confirmed = offboard_confirmed_;
    status.detail = detail_;
    status_publisher_->publish(status);
  }

  std::string uav_id_;
  int priming_cycles_{10};
  double min_stream_rate_hz_{2.0};
  bool auto_engage_{true};
  rclcpp::Duration engage_timeout_{0, 0};

  uint8_t state_{OffboardStatus::STATE_IDLE};
  std::string detail_{"waiting for setpoint stream"};

  std::atomic<bool> failsafe_{false};
  std::atomic<uint8_t> nav_state_{0};
  /// Latched by an RC takeover; cleared only by disarm (P11.6).
  std::atomic<bool> pilot_override_{false};
  std::atomic<bool> offboard_confirmed_{false};
  std::atomic<uint64_t> stream_count_{0};
  std::atomic<uint64_t> stream_count_in_window_{0};

  double measured_rate_hz_{0.0};
  rclcpp::Time window_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time engage_requested_at_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<VehicleControlMode>::SharedPtr control_mode_subscription_;
  rclcpp::Subscription<OffboardControlMode>::SharedPtr stream_subscription_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Publisher<OffboardStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr supervise_timer_;
};

}  // namespace uav_px4_backend

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_px4_backend::OffboardSessionManagerNode>());
  rclcpp::shutdown();
  return 0;
}
