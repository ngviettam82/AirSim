// Downward beam -> height above ground, measured directly.

#include <cmath>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "uav_localization/height_from_range.hpp"

namespace uav_localization
{

using nav_msgs::msg::Odometry;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::Range;

class RangefinderAdapterNode : public rclcpp::Node
{
public:
  RangefinderAdapterNode()
  : Node("rangefinder_adapter_node")
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    frame_id_ = declare_parameter<std::string>("range_frame", "rangefinder_link");
    max_tilt_rad_ = declare_parameter<double>("max_tilt_rad", 0.6981);   // 40 degrees
    attitude_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("attitude_timeout_sec", 0.5));

    const std::string prefix = "/uav/" + uav_id_;

    scan_subscription_ = create_subscription<LaserScan>(
      prefix + "/localization/range_scan_raw", rclcpp::QoS(10),
      [this](const LaserScan::SharedPtr msg) {onScan(*msg);});

    // Raw, not fused: reading fused here closes a loop.
    rclcpp::QoS state_qos(rclcpp::KeepLast(10));
    state_qos.best_effort();
    attitude_subscription_ = create_subscription<Odometry>(
      prefix + "/state/odometry_raw", state_qos,
      [this](const Odometry::SharedPtr msg) {
        orientation_ = msg->pose.pose.orientation;
        last_attitude_ = now();
      });

    range_publisher_ = create_publisher<Range>(prefix + "/localization/range", rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(), "rangefinder adapter ready for %s", uav_id_.c_str());
  }

private:
  void onScan(const LaserScan & scan)
  {
    if (scan.ranges.empty()) {
      return;
    }

    Range range;
    range.header.stamp = scan.header.stamp;
    range.header.frame_id = frame_id_;
    range.radiation_type = Range::INFRARED;
    range.field_of_view = std::max(0.0F, scan.angle_max - scan.angle_min);
    range.min_range = scan.range_min;
    range.max_range = scan.range_max;

    if (!hasFreshAttitude()) {
      // No attitude, no tilt correction; height would read high.
      range.range = std::numeric_limits<float>::infinity();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no recent attitude, height withheld");
      range_publisher_->publish(range);
      return;
    }

    const HeightEstimate estimate = heightFromRange(
      scan.ranges[0], orientation_, scan.range_min, scan.range_max, max_tilt_rad_);

    if (!estimate.valid) {
      // Out-of-interval is the sensor_msgs/Range way of saying "unusable".
      range.range = std::numeric_limits<float>::infinity();
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000, "height unavailable: %s", estimate.reason);
    } else {
      range.range = static_cast<float>(estimate.height_m);
    }

    range_publisher_->publish(range);
  }

  bool hasFreshAttitude() const
  {
    if (last_attitude_.nanoseconds() == 0) {
      return false;
    }
    return (now() - last_attitude_) < attitude_timeout_;
  }

  std::string uav_id_;
  std::string frame_id_;
  double max_tilt_rad_{0.6981};
  rclcpp::Duration attitude_timeout_{0, 0};
  rclcpp::Time last_attitude_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Quaternion orientation_;

  rclcpp::Subscription<LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr attitude_subscription_;
  rclcpp::Publisher<Range>::SharedPtr range_publisher_;
};

}  // namespace uav_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_localization::RangefinderAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
