// External vision into the PX4 estimator, gated on localization validity.

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/localization_status.hpp>

#include "uav_px4_backend/frame_conversions.hpp"
#include "uav_px4_backend/px4_interop.hpp"

namespace uav_px4_backend
{

using nav_msgs::msg::Odometry;
using px4_msgs::msg::VehicleOdometry;
using uav_interfaces::msg::LocalizationStatus;

class Px4ExternalOdometryNode : public rclcpp::Node
{
public:
  Px4ExternalOdometryNode()
  : Node("px4_external_odometry_node")
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    require_valid_localization_ =
      declare_parameter<bool>("require_valid_localization", true);
    status_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("status_timeout_sec", 1.0));

    // Never odometry_fused: it is built partly from this same estimator.
    const std::string source_topic = declare_parameter<std::string>(
      "external_odometry_topic", "localization/vio_odometry");

    // 0 = forward every sample. Real VIO runs 20-30 Hz; do not throttle below that.
    const double max_publish_rate_hz =
      declare_parameter<double>("max_publish_rate_hz", 0.0);
    min_publish_interval_ = max_publish_rate_hz > 0.0
      ? rclcpp::Duration::from_seconds(1.0 / max_publish_rate_hz)
      : rclcpp::Duration(0, 0);

    odometry_subscription_ = create_subscription<Odometry>(
      "/uav/" + uav_id_ + "/" + source_topic, rclcpp::QoS(10),
      [this](const Odometry::SharedPtr msg) {forward(*msg);});

    status_subscription_ = create_subscription<LocalizationStatus>(
      "/uav/" + uav_id_ + "/state/localization_status", rclcpp::QoS(10),
      [this](const LocalizationStatus::SharedPtr msg) {
        latest_status_ = *msg;
        last_status_received_ = now();
      });

    // Only way to learn PX4's clock is its own stamps.
    rclcpp::QoS px4_qos(rclcpp::KeepLast(10));
    px4_qos.best_effort();
    px4_qos.transient_local();
    px4_clock_subscription_ = create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      [this](const VehicleOdometry::SharedPtr msg) {
        clock_offset_.observe(msg->timestamp, px4::timestampMicros(now()));
      });

    visual_odometry_publisher_ =
      create_publisher<VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

    RCLCPP_INFO(get_logger(), "external odometry bridge ready for %s", uav_id_.c_str());
  }

private:
  void forward(const Odometry & external)
  {
    if (!localizationTrusted()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "localization not trusted, withholding pose");
      return;
    }

    const auto & position = external.pose.pose.position;
    const auto & orientation = external.pose.pose.orientation;
    if (!isFinite(position) || !isFinite(orientation)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "external pose is not finite, withholding");
      return;
    }

    if (!clock_offset_.known()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "PX4 clock not observed yet, withholding pose");
      return;
    }

    if (min_publish_interval_.nanoseconds() > 0 &&
      last_forward_.nanoseconds() != 0 &&
      (now() - last_forward_) < min_publish_interval_)
    {
      return;
    }
    last_forward_ = now();

    VehicleOdometry out{};
    out.timestamp = clock_offset_.toPx4Micros(px4::timestampMicros(now()));
    // Estimator aligns on sample time: when measured, not forwarded.
    const rclcpp::Time measured_at(external.header.stamp);
    out.timestamp_sample = measured_at.nanoseconds() > 0
      ? clock_offset_.toPx4Micros(px4::timestampMicros(measured_at))
      : out.timestamp;
    out.pose_frame = VehicleOdometry::POSE_FRAME_NED;
    out.position = frames::enuToNedArray(position);
    out.q = frames::rosToPx4Quaternion(orientation);

    // Twist arrives body FLU per REP-105; PX4 wants FRD.
    const auto & velocity_flu = external.twist.twist.linear;
    if (isFinite(velocity_flu)) {
      out.velocity_frame = VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
      out.velocity = frames::fluToFrdArray(velocity_flu);
    } else {
      // Declaring a frame with a zero-filled velocity claims "stationary".
      out.velocity_frame = VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
      out.velocity.fill(std::numeric_limits<float>::quiet_NaN());
    }

    out.position_variance = toPx4Variance(external.pose.covariance);

    visual_odometry_publisher_->publish(out);
  }

  /// Variance is unsigned, so the z flip is a no-op.
  static std::array<float, 3> toPx4Variance(
    const std::array<double, 36> & enu_covariance)
  {
    constexpr size_t kEnuVarianceX = 0;
    constexpr size_t kEnuVarianceY = 7;
    constexpr size_t kEnuVarianceZ = 14;

    return {
      knownVariance(enu_covariance[kEnuVarianceY]),
      knownVariance(enu_covariance[kEnuVarianceX]),
      knownVariance(enu_covariance[kEnuVarianceZ])};
  }

  /// A zero variance would claim this pose is beyond doubt.
  static float knownVariance(double value)
  {
    return (std::isfinite(value) && value > 0.0)
           ? static_cast<float>(value)
           : std::numeric_limits<float>::quiet_NaN();
  }

  static bool isFinite(const geometry_msgs::msg::Point & p)
  {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }

  static bool isFinite(const geometry_msgs::msg::Vector3 & v)
  {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  }

  static bool isFinite(const geometry_msgs::msg::Quaternion & q)
  {
    return std::isfinite(q.w) && std::isfinite(q.x) &&
           std::isfinite(q.y) && std::isfinite(q.z);
  }

  bool localizationTrusted() const
  {
    if (!require_valid_localization_) {
      return true;
    }
    if (last_status_received_.nanoseconds() == 0) {
      return false;
    }
    if ((now() - last_status_received_) > status_timeout_) {
      return false;
    }
    return latest_status_.is_valid &&
           latest_status_.quality != LocalizationStatus::QUALITY_BAD;
  }

  std::string uav_id_;
  bool require_valid_localization_{true};
  rclcpp::Duration status_timeout_{0, 0};
  rclcpp::Duration min_publish_interval_{0, 0};
  rclcpp::Time last_status_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_forward_{0, 0, RCL_ROS_TIME};

  LocalizationStatus latest_status_;

  px4::Px4ClockOffset clock_offset_;

  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<LocalizationStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr px4_clock_subscription_;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr visual_odometry_publisher_;
};

}  // namespace uav_px4_backend

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_px4_backend::Px4ExternalOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
