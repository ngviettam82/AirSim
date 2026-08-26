// GNSS fix -> position-source contract. Reads NavSatFix, not px4_msgs (R1).

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <uav_interfaces/msg/localization_status.hpp>

#include "uav_localization/gauss_markov_drift.hpp"
#include "uav_localization/geodetic_projection.hpp"
#include "uav_localization/source_channel.hpp"

namespace uav_localization
{

using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;
using uav_interfaces::msg::LocalizationStatus;

class GpsAdapterNode : public rclcpp::Node
{
public:
  GpsAdapterNode()
  : Node("gps_adapter_node")
  {
    const std::string uav_id = declare_parameter<std::string>("uav_id", "uav0");
    const std::string prefix = "/uav/" + uav_id;

    SourceChannel::Options options;
    options.source_id = LocalizationStatus::SOURCE_GPS;
    options.uav_id = uav_id;
    options.odometry_topic = prefix + "/localization/gps_odometry";
    options.status_topic = prefix + "/localization/gps_status";
    options.odom_frame = declare_parameter<std::string>("odom_frame", "odom");
    options.base_frame = declare_parameter<std::string>("base_frame", "base_link");
    options.thresholds.degraded_above_m = declare_parameter<double>("degraded_above_m", 0.5);
    options.thresholds.bad_above_m = declare_parameter<double>("bad_above_m", 2.0);
    channel_ = std::make_unique<SourceChannel>(*this, options);

    DriftParameters drift;
    drift.enabled = declare_parameter<bool>("drift.enabled", false);
    drift.correlation_time_sec = declare_parameter<double>("drift.correlation_time_sec", 60.0);
    drift.sigma_m = declare_parameter<double>("drift.sigma_m", 5.0);
    drift.seed = static_cast<uint32_t>(declare_parameter<int>("drift.seed", 0));
    drift_.configure(drift);
    declare_parameter<bool>("drift.force_no_fix", false);

    if (drift.enabled || forcedNoFix()) {
      RCLCPP_WARN(
        get_logger(), "GNSS fault injection active: simulation only, never a real flight");
    }

    fix_subscription_ = create_subscription<NavSatFix>(
      prefix + "/state/gnss_fix", rclcpp::QoS(10),
      [this](const NavSatFix::SharedPtr msg) {onFix(*msg);});

    // Latched, so a late start still gets the datum.
    rclcpp::QoS datum_qos(rclcpp::KeepLast(1));
    datum_qos.transient_local();
    origin_subscription_ = create_subscription<NavSatFix>(
      prefix + "/state/gnss_origin", datum_qos,
      [this](const NavSatFix::SharedPtr msg) {onOrigin(*msg);});

    RCLCPP_INFO(get_logger(), "gps adapter ready for %s", uav_id.c_str());
  }

private:
  void onOrigin(const NavSatFix & origin)
  {
    if (datum_valid_) {
      RCLCPP_WARN(get_logger(), "estimator origin moved; reprojecting from the new datum");
    }
    datum_ = {origin.latitude, origin.longitude, origin.altitude};
    datum_valid_ = true;
  }

  void onFix(const NavSatFix & fix)
  {
    const rclcpp::Time stamp(fix.header.stamp);

    // Read live: losing the fix mid-flight is the case tested.
    if (forcedNoFix()) {
      channel_->publishUnavailable(stamp, "fix loss forced by parameter");
      return;
    }
    if (fix.status.status < NavSatStatus::STATUS_FIX) {
      channel_->publishUnavailable(stamp, "receiver reports no usable fix");
      return;
    }
    if (!datum_valid_) {
      // No datum, no shared frame to place the fix.
      channel_->publishUnavailable(stamp, "waiting for the estimator origin");
      return;
    }

    const LocalPoint local =
      projectToLocalEnu(datum_, {fix.latitude, fix.longitude, fix.altitude});
    const auto drift_offset = drift_.advance(secondsSince(last_fix_stamp_, stamp));

    SourceReport report;
    report.stamp = stamp;
    report.pose.position.x = local.east_m + drift_offset[0];
    report.pose.position.y = local.north_m + drift_offset[1];
    report.pose.position.z = local.up_m + drift_offset[2];
    report.pose.orientation.w = 1.0;

    // GNSS measures neither attitude nor body velocity.
    report.heading_stddev = -1.0;
    report.twist_known = false;

    report.position_stddev = fix.position_covariance_type == NavSatFix::COVARIANCE_TYPE_UNKNOWN
      ? -1.0
      : std::sqrt(fix.position_covariance[0]);

    // Injected drift deliberately does not inflate the reported accuracy.
    report.detail = drift_.parameters().enabled ? "simulated drift active" : "";

    channel_->publish(report);
    last_fix_stamp_ = stamp;
  }

  bool forcedNoFix() {return get_parameter("drift.force_no_fix").as_bool();}

  static double secondsSince(const rclcpp::Time & previous, const rclcpp::Time & current)
  {
    if (previous.nanoseconds() == 0) {
      return 0.0;
    }
    return (current - previous).seconds();
  }

  std::unique_ptr<SourceChannel> channel_;
  GaussMarkovDrift drift_;
  GeodeticPoint datum_{};
  bool datum_valid_{false};
  rclcpp::Time last_fix_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<NavSatFix>::SharedPtr fix_subscription_;
  rclcpp::Subscription<NavSatFix>::SharedPtr origin_subscription_;
};

}  // namespace uav_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_localization::GpsAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
