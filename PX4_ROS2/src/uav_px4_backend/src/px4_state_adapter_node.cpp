// Inbound half of the bridge: PX4 topics -> internal state contracts.

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <uav_interfaces/msg/vehicle_health.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>

#include "uav_px4_backend/frame_conversions.hpp"
#include "uav_px4_backend/px4_interop.hpp"

namespace uav_px4_backend
{

using nav_msgs::msg::Odometry;
using px4_msgs::msg::BatteryStatus;
using px4_msgs::msg::FailsafeFlags;
using px4_msgs::msg::SensorGps;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleStatus;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;
using uav_interfaces::msg::VehicleHealth;
using uav_interfaces::msg::VehicleState;

class Px4StateAdapterNode : public rclcpp::Node
{
public:
  Px4StateAdapterNode()
  : Node("px4_state_adapter_node")
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);
    connection_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("connection_timeout_sec", 2.0));

    // PX4 publishes best effort; a reliable subscription would never match.
    rclcpp::QoS px4_qos(rclcpp::KeepLast(10));
    px4_qos.best_effort();
    px4_qos.transient_local();

    status_subscription_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos,
      [this](const VehicleStatus::SharedPtr msg) {
        latest_status_ = *msg;
        last_status_received_ = now();
      });

    odometry_subscription_ = create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      [this](const VehicleOdometry::SharedPtr msg) {publishOdometry(*msg);});

    battery_subscription_ = create_subscription<BatteryStatus>(
      "/fmu/out/battery_status", px4_qos,
      [this](const BatteryStatus::SharedPtr msg) {latest_battery_ = *msg;});

    failsafe_subscription_ = create_subscription<FailsafeFlags>(
      "/fmu/out/failsafe_flags", px4_qos,
      [this](const FailsafeFlags::SharedPtr msg) {latest_failsafe_ = *msg;});

    gnss_subscription_ = create_subscription<SensorGps>(
      "/fmu/out/vehicle_gps_position", px4_qos,
      [this](const SensorGps::SharedPtr msg) {publishGnssFix(*msg);});

    local_position_subscription_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", px4_qos,
      [this](const VehicleLocalPosition::SharedPtr msg) {publishGnssOrigin(*msg);});

    state_publisher_ = create_publisher<VehicleState>(
      "/uav/" + uav_id_ + "/state/vehicle", rclcpp::QoS(10));
    odometry_publisher_ = create_publisher<Odometry>(
      "/uav/" + uav_id_ + "/state/odometry_raw", rclcpp::QoS(10));
    health_publisher_ = create_publisher<VehicleHealth>(
      "/uav/" + uav_id_ + "/state/health_px4", rclcpp::QoS(10));
    gnss_fix_publisher_ = create_publisher<NavSatFix>(
      "/uav/" + uav_id_ + "/state/gnss_fix", rclcpp::QoS(10));

    // Latched: changes rarely, but every later subscriber needs it.
    rclcpp::QoS datum_qos(rclcpp::KeepLast(1));
    datum_qos.transient_local();
    gnss_origin_publisher_ = create_publisher<NavSatFix>(
      "/uav/" + uav_id_ + "/state/gnss_origin", datum_qos);

    // Stamps come from the node clock; cadence must match.
    publish_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_rate_hz),
      [this]() {
        publishVehicleState();
        publishHealth();
      });

    RCLCPP_INFO(get_logger(), "state adapter ready for %s", uav_id_.c_str());
  }

private:
  /// Timer-driven, not receipt-driven, so losing PX4 is itself reported.
  void publishVehicleState()
  {
    VehicleState state;
    state.header.stamp = now();
    state.uav_id = uav_id_;
    state.connected = isConnected();

    if (state.connected) {
      state.armed = latest_status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
      state.in_air = latest_status_.takeoff_time > 0;
      state.flight_mode = px4::toInternalFlightMode(latest_status_.nav_state);
      state.nav_state_raw = latest_status_.nav_state;
      state.failsafe_active = latest_status_.failsafe;
      state.pre_flight_checks_pass = latest_status_.pre_flight_checks_pass;
    } else {
      state.flight_mode = VehicleState::FLIGHT_MODE_UNKNOWN;
    }

    state_publisher_->publish(state);
  }

  /// Twist goes in the body frame to satisfy REP-105.
  void publishOdometry(const VehicleOdometry & odometry)
  {
    if (odometry.pose_frame != VehicleOdometry::POSE_FRAME_NED) {
      return;
    }
    if (!px4::allFinite(odometry.position) || std::isnan(odometry.q[0])) {
      return;
    }

    Odometry out;
    out.header.stamp = now();
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;

    out.pose.pose.position = frames::nedToEnuPoint(odometry.position);
    out.pose.pose.orientation = frames::px4ToRosQuaternion(odometry.q);

    if (px4::allFinite(odometry.velocity)) {
      if (odometry.velocity_frame == VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
        out.twist.twist.linear = frames::frdToFluVector(odometry.velocity);
      } else {
        out.twist.twist.linear = frames::rotateEnuToBodyFlu(
          frames::nedToEnuVector(odometry.velocity), out.pose.pose.orientation);
      }
    }
    if (px4::allFinite(odometry.angular_velocity)) {
      out.twist.twist.angular = frames::frdToFluVector(odometry.angular_velocity);
    }

    // Diagonal only; PX4 reports variances, not a full covariance matrix.
    if (px4::allFinite(odometry.position_variance)) {
      out.pose.covariance[0] = odometry.position_variance[1];   // ENU x from NED y
      out.pose.covariance[7] = odometry.position_variance[0];
      out.pose.covariance[14] = odometry.position_variance[2];
    }

    odometry_publisher_->publish(out);
  }

  /// A standard ROS type keeps px4_msgs out of localization (R1).
  void publishGnssFix(const SensorGps & gps)
  {
    if (!std::isfinite(gps.latitude_deg) || !std::isfinite(gps.longitude_deg)) {
      return;
    }

    NavSatFix fix;
    fix.header.stamp = now();
    fix.header.frame_id = base_frame_;
    fix.status.service = NavSatStatus::SERVICE_GPS;
    fix.status.status = px4::toNavSatStatus(gps.fix_type);
    fix.latitude = gps.latitude_deg;
    fix.longitude = gps.longitude_deg;
    fix.altitude = gps.altitude_msl_m;

    // A receiver without a stated accuracy must not look accurate.
    if (std::isfinite(gps.eph) && std::isfinite(gps.epv)) {
      const double horizontal_variance = static_cast<double>(gps.eph) * gps.eph;
      fix.position_covariance[0] = horizontal_variance;
      fix.position_covariance[4] = horizontal_variance;
      fix.position_covariance[8] = static_cast<double>(gps.epv) * gps.epv;
      fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
      fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    gnss_fix_publisher_->publish(fix);
  }

  /// The geodetic point that local (0,0,0) means.
  void publishGnssOrigin(const VehicleLocalPosition & local_position)
  {
    if (!local_position.xy_global || !local_position.z_global) {
      return;
    }
    if (local_position.ref_timestamp == last_datum_timestamp_) {
      return;
    }
    if (!std::isfinite(local_position.ref_lat) || !std::isfinite(local_position.ref_lon)) {
      return;
    }

    NavSatFix datum;
    datum.header.stamp = now();
    datum.header.frame_id = odom_frame_;
    datum.status.service = NavSatStatus::SERVICE_GPS;
    datum.status.status = NavSatStatus::STATUS_FIX;
    datum.latitude = local_position.ref_lat;
    datum.longitude = local_position.ref_lon;
    datum.altitude = local_position.ref_alt;
    datum.position_covariance_type = NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    gnss_origin_publisher_->publish(datum);

    if (last_datum_timestamp_ != 0) {
      RCLCPP_WARN(get_logger(), "estimator origin moved; local frame shifted");
    }
    last_datum_timestamp_ = local_position.ref_timestamp;
  }

  /// PX4 exposes estimator validity, not raw sensor health.
  void publishHealth()
  {
    VehicleHealth health;
    health.header.stamp = now();
    health.uav_id = uav_id_;

    const px4::BatteryHealth battery = px4::toBatteryHealth(latest_battery_);
    health.battery_voltage = battery.voltage;
    health.battery_current = battery.current;
    health.battery_remaining = battery.remaining;
    health.battery_temperature = battery.temperature;

    // These bools cannot say "unknown"; overall_level does.
    const FailsafeFlags flags = latest_failsafe_.value_or(FailsafeFlags());
    health.imu_ok = !flags.angular_velocity_invalid && !flags.attitude_invalid;
    health.gps_ok = !flags.global_position_invalid;
    health.barometer_ok = !flags.local_altitude_invalid;
    health.magnetometer_ok = !flags.attitude_invalid;

    const bool sensors_ok = health.imu_ok && health.barometer_ok;
    if (!isConnected()) {
      health.overall_level = VehicleHealth::LEVEL_UNKNOWN;
      health.detail = "no data from autopilot";
    } else if (!latest_failsafe_) {
      health.overall_level = VehicleHealth::LEVEL_UNKNOWN;
      health.detail = "no failsafe flags from autopilot";
    } else if (flags.battery_unhealthy || !sensors_ok) {
      health.overall_level = VehicleHealth::LEVEL_CRITICAL;
      health.detail = "estimator or battery reported unhealthy";
    } else if (!health.gps_ok) {
      health.overall_level = VehicleHealth::LEVEL_WARNING;
      health.detail = "global position estimate invalid";
    } else if (!std::isfinite(health.battery_remaining)) {
      health.overall_level = VehicleHealth::LEVEL_WARNING;
      health.detail = "no battery charge reading from autopilot";
    } else {
      health.overall_level = VehicleHealth::LEVEL_OK;
      health.detail = "derived from failsafe_flags";
    }

    health_publisher_->publish(health);
  }

  bool isConnected() const
  {
    if (last_status_received_.nanoseconds() == 0) {
      return false;
    }
    return (now() - last_status_received_) < connection_timeout_;
  }

  std::string uav_id_;
  std::string odom_frame_;
  std::string base_frame_;
  rclcpp::Duration connection_timeout_{0, 0};
  rclcpp::Time last_status_received_{0, 0, RCL_ROS_TIME};

  VehicleStatus latest_status_;
  std::optional<BatteryStatus> latest_battery_;
  std::optional<FailsafeFlags> latest_failsafe_;
  uint64_t last_datum_timestamp_{0};

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_subscription_;
  rclcpp::Subscription<FailsafeFlags>::SharedPtr failsafe_subscription_;
  rclcpp::Subscription<SensorGps>::SharedPtr gnss_subscription_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
  rclcpp::Publisher<VehicleState>::SharedPtr state_publisher_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<VehicleHealth>::SharedPtr health_publisher_;
  rclcpp::Publisher<NavSatFix>::SharedPtr gnss_fix_publisher_;
  rclcpp::Publisher<NavSatFix>::SharedPtr gnss_origin_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace uav_px4_backend

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_px4_backend::Px4StateAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
