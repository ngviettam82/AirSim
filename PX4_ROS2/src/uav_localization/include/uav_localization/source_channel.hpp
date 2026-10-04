// No pose can be published without its uncertainty. See README.

#ifndef UAV_LOCALIZATION__SOURCE_CHANNEL_HPP_
#define UAV_LOCALIZATION__SOURCE_CHANNEL_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/localization_status.hpp>

namespace uav_localization
{

/// Shared policy, so no source invents its own "good enough".
struct QualityThresholds
{
  double degraded_above_m{0.5};
  double bad_above_m{2.0};
  double stale_after_sec{0.5};
};

uint8_t classifyQuality(double position_stddev, const QualityThresholds & thresholds);

struct SourceReport
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;   // child frame, per REP-105
  bool twist_known{false};           // an unset twist is not a measured zero
  double twist_stddev{-1.0};         // 1-sigma, m/s. Negative leaves it unstated.
  double position_stddev{-1.0};      // 1-sigma, m. Negative means unknown.
  double heading_stddev{-1.0};       // 1-sigma, rad
  std::string detail;
};

class SourceChannel
{
public:
  struct Options
  {
    uint8_t source_id{uav_interfaces::msg::LocalizationStatus::SOURCE_NONE};
    std::string uav_id{"uav0"};
    std::string odometry_topic;
    std::string status_topic;
    std::string odom_frame{"odom"};
    std::string base_frame{"base_link"};
    QualityThresholds thresholds{};
  };

  SourceChannel(rclcpp::Node & node, Options options);

  /// Untrustworthy report: status only, never a pose.
  void publish(const SourceReport & report);

  /// Speaks up; silence downstream reads as a dead node.
  void publishUnavailable(const rclcpp::Time & stamp, const std::string & reason);

  bool lastPublishWasValid() const {return last_valid_;}

private:
  void publishStatus(
    const rclcpp::Time & stamp, uint8_t quality, bool is_valid,
    double position_stddev, double heading_stddev, const std::string & detail);

  rclcpp::Node & node_;
  Options options_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<uav_interfaces::msg::LocalizationStatus>::SharedPtr status_publisher_;
  rclcpp::Time last_valid_stamp_{0, 0, RCL_ROS_TIME};
  bool last_valid_{false};
};

}  // namespace uav_localization

#endif  // UAV_LOCALIZATION__SOURCE_CHANNEL_HPP_
