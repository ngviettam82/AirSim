// Outbound half of the bridge: internal ControlCommand -> PX4 setpoints.

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/control_command.hpp>
#include <uav_interfaces/msg/result_code.hpp>
#include <uav_interfaces/srv/arm.hpp>
#include <uav_interfaces/srv/disarm.hpp>
#include <uav_interfaces/srv/set_flight_mode.hpp>

#include "uav_px4_backend/frame_conversions.hpp"
#include "uav_px4_backend/px4_interop.hpp"

namespace uav_px4_backend
{

using namespace std::chrono_literals;

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleStatus;
using uav_interfaces::msg::ControlCommand;
using uav_interfaces::msg::ResultCode;
using uav_interfaces::srv::Arm;
using uav_interfaces::srv::Disarm;
using uav_interfaces::srv::SetFlightMode;

constexpr float kUncontrolled = std::numeric_limits<float>::quiet_NaN();

constexpr auto kAckPollInterval = 50ms;

/// False means the clock stopped or shutdown began.
inline bool waitOnePollInterval(const rclcpp::Clock::SharedPtr & clock)
{
  return rclcpp::ok() && clock->sleep_for(rclcpp::Duration(kAckPollInterval));
}

class Px4CommandGatewayNode : public rclcpp::Node
{
public:
  Px4CommandGatewayNode()
  : Node("px4_command_gateway_node")
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    const double stream_rate_hz = declare_parameter<double>("stream_rate_hz", 20.0);
    command_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("command_timeout_sec", 0.5));
    ack_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("ack_timeout_sec", 5.0));

    subscription_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = subscription_group_;

    rclcpp::QoS px4_qos(rclcpp::KeepLast(10));
    px4_qos.best_effort();
    px4_qos.transient_local();

    status_subscription_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos,
      [this](const VehicleStatus::SharedPtr msg) {
        armed_ = msg->arming_state == VehicleStatus::ARMING_STATE_ARMED;
        nav_state_ = msg->nav_state;
      },
      sub_options);

    command_subscription_ = create_subscription<ControlCommand>(
      "/uav/" + uav_id_ + "/control/command_selected", rclcpp::QoS(10),
      [this](const ControlCommand::SharedPtr msg) {
        latest_command_ = *msg;
        last_command_received_ = now();
      },
      sub_options);

    offboard_mode_publisher_ =
      create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    setpoint_publisher_ =
      create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ =
      create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    arm_service_ = create_service<Arm>(
      "/uav/" + uav_id_ + "/backend/arm",
      [this](
        const std::shared_ptr<Arm::Request>,
        std::shared_ptr<Arm::Response> response) {
        handleArmRequest(true, response->success, response->result_code, response->message);
      },
      rmw_qos_profile_services_default, service_group_);

    disarm_service_ = create_service<Disarm>(
      "/uav/" + uav_id_ + "/backend/disarm",
      [this](
        const std::shared_ptr<Disarm::Request>,
        std::shared_ptr<Disarm::Response> response) {
        handleArmRequest(false, response->success, response->result_code, response->message);
      },
      rmw_qos_profile_services_default, service_group_);

    set_mode_service_ = create_service<SetFlightMode>(
      "/uav/" + uav_id_ + "/backend/set_mode",
      [this](
        const std::shared_ptr<SetFlightMode::Request> request,
        std::shared_ptr<SetFlightMode::Response> response) {
        handleSetModeRequest(request->mode, *response);
      },
      rmw_qos_profile_services_default, service_group_);

    // Not a wall timer: PX4 measures on its own clock.
    stream_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / stream_rate_hz),
      [this]() {streamSetpoint();}, subscription_group_);

    RCLCPP_INFO(get_logger(), "command gateway ready for %s", uav_id_.c_str());
  }

private:
  /// Silence is deliberate: PX4 failsafe beats a stale target.
  void streamSetpoint()
  {
    if (!hasFreshCommand()) {
      return;
    }
    if (!isImplementedMode(latest_command_.control_mode)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "rejecting control_mode %u: this gateway sends no setpoint for it",
        latest_command_.control_mode);
      return;
    }

    publishOffboardControlMode();
    publishTrajectorySetpoint();
  }

  // A mode without a setpoint leaves offboard nothing to follow.
  static bool isImplementedMode(uint8_t control_mode)
  {
    switch (control_mode) {
      case ControlCommand::MODE_POSITION:
      case ControlCommand::MODE_VELOCITY:
      case ControlCommand::MODE_ACCELERATION:
        return true;
      default:
        return false;
    }
  }

  bool hasFreshCommand() const
  {
    if (last_command_received_.nanoseconds() == 0) {
      return false;
    }
    return (now() - last_command_received_) < command_timeout_;
  }

  void publishOffboardControlMode()
  {
    OffboardControlMode mode{};
    switch (latest_command_.control_mode) {
      case ControlCommand::MODE_POSITION:
        mode.position = true;
        break;
      case ControlCommand::MODE_VELOCITY:
        mode.velocity = true;
        break;
      case ControlCommand::MODE_ACCELERATION:
        mode.acceleration = true;
        break;
      default:
        return;
    }
    mode.timestamp = px4::timestampMicros(now());
    offboard_mode_publisher_->publish(mode);
  }

  /// NaN means "leave this axis alone"; 0 means the origin.
  void publishTrajectorySetpoint()
  {
    TrajectorySetpoint setpoint{};
    setpoint.position = {kUncontrolled, kUncontrolled, kUncontrolled};
    setpoint.velocity = {kUncontrolled, kUncontrolled, kUncontrolled};
    setpoint.acceleration = {kUncontrolled, kUncontrolled, kUncontrolled};
    setpoint.yaw = kUncontrolled;
    setpoint.yawspeed = kUncontrolled;

    switch (latest_command_.control_mode) {
      case ControlCommand::MODE_POSITION:
        setpoint.position = frames::enuToNedArray(latest_command_.position);
        setpoint.yaw = static_cast<float>(frames::yawEnuToNed(latest_command_.yaw));
        break;
      case ControlCommand::MODE_VELOCITY:
        setpoint.velocity = frames::enuToNedArray(latest_command_.velocity);
        setpoint.yawspeed =
          static_cast<float>(frames::yawRateEnuToNed(latest_command_.yaw_rate));
        break;
      case ControlCommand::MODE_ACCELERATION:
        setpoint.acceleration = frames::enuToNedArray(latest_command_.acceleration);
        setpoint.yaw = static_cast<float>(frames::yawEnuToNed(latest_command_.yaw));
        break;
      default:
        return;
    }

    setpoint.timestamp = px4::timestampMicros(now());
    setpoint_publisher_->publish(setpoint);
  }

  /// Confirmed by the autopilot, not assumed from the send.
  void handleArmRequest(bool arm, bool & success, uint8_t & result_code, std::string & message)
  {
    publishVehicleCommand(
      VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, arm ? 1.0F : 0.0F);

    const rclcpp::Time deadline = now() + ack_timeout_;
    while (now() < deadline) {
      if (armed_ == arm) {
        success = true;
        result_code = ResultCode::SUCCEEDED;
        message = arm ? "armed" : "disarmed";
        return;
      }
      if (!waitOnePollInterval(get_clock())) {
        break;
      }
    }

    success = false;
    result_code = ResultCode::ABORTED_VEHICLE_REJECTED;
    message = arm ? "autopilot did not arm within timeout"
      : "autopilot did not disarm within timeout";
  }

  /// Also confirmed by the autopilot, never assumed.
  void handleSetModeRequest(
    uint8_t requested_mode, SetFlightMode::Response & response)
  {
    const px4::FlightModeCommand target = px4::toPx4Mode(requested_mode);
    if (!target.supported) {
      response.success = false;
      response.result_code = ResultCode::ABORTED_INVALID_GOAL;
      response.message = "flight mode not supported by this gateway";
      response.current_mode = px4::toInternalFlightMode(nav_state_);
      return;
    }

    publishVehicleCommand(
      VehicleCommand::VEHICLE_CMD_DO_SET_MODE, px4::kCustomModeEnabled,
      target.main_mode, target.sub_mode);

    const rclcpp::Time deadline = now() + ack_timeout_;
    while (now() < deadline) {
      if (px4::toInternalFlightMode(nav_state_) == requested_mode) {
        response.success = true;
        response.result_code = ResultCode::SUCCEEDED;
        response.message = "mode change confirmed";
        response.current_mode = requested_mode;
        return;
      }
      if (!waitOnePollInterval(get_clock())) {
        break;
      }
    }

    response.success = false;
    response.result_code = ResultCode::ABORTED_VEHICLE_REJECTED;
    response.message = "autopilot did not enter requested mode";
    response.current_mode = px4::toInternalFlightMode(nav_state_);
  }

  void publishVehicleCommand(
    uint16_t command, float param1 = 0.0F, float param2 = 0.0F, float param3 = 0.0F)
  {
    vehicle_command_publisher_->publish(
      px4::makeVehicleCommand(
        command, px4::timestampMicros(now()), param1, param2, param3));
  }

  std::string uav_id_;
  rclcpp::Duration command_timeout_{0, 0};
  rclcpp::Duration ack_timeout_{0, 0};
  rclcpp::Time last_command_received_{0, 0, RCL_ROS_TIME};

  // Lock-free only because writer and reader share one exclusive group.
  ControlCommand latest_command_;

  // Atomic because services run in a different callback group.
  std::atomic<bool> armed_{false};
  std::atomic<uint8_t> nav_state_{0};

  rclcpp::CallbackGroup::SharedPtr subscription_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<ControlCommand>::SharedPtr command_subscription_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Service<Arm>::SharedPtr arm_service_;
  rclcpp::Service<Disarm>::SharedPtr disarm_service_;
  rclcpp::Service<SetFlightMode>::SharedPtr set_mode_service_;
  rclcpp::TimerBase::SharedPtr stream_timer_;
};

}  // namespace uav_px4_backend

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_px4_backend::Px4CommandGatewayNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
