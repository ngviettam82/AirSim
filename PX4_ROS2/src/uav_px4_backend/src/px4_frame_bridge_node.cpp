// Broadcasts odom -> base_link TF from PX4 odometry.

#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "uav_px4_backend/frame_conversions.hpp"
#include "uav_px4_backend/px4_interop.hpp"

namespace uav_px4_backend
{

using px4_msgs::msg::VehicleOdometry;

class Px4FrameBridgeNode : public rclcpp::Node
{
public:
  Px4FrameBridgeNode()
  : Node("px4_frame_bridge_node")
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rclcpp::QoS px4_qos(rclcpp::KeepLast(10));
    px4_qos.best_effort();
    px4_qos.transient_local();

    odometry_subscription_ = create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      [this](const VehicleOdometry::SharedPtr msg) {broadcastTransform(*msg);});

    RCLCPP_INFO(get_logger(), "frame bridge ready for %s", uav_id_.c_str());
  }

private:
  void broadcastTransform(const VehicleOdometry & odometry)
  {
    if (odometry.pose_frame != VehicleOdometry::POSE_FRAME_NED) {
      warnThrottled("unsupported pose_frame, skipping");
      return;
    }
    if (!px4::allFinite(odometry.position) || std::isnan(odometry.q[0])) {
      warnThrottled("estimator reports invalid pose, skipping");
      return;
    }

    const auto position_enu = frames::nedToEnuPoint(odometry.position);
    const auto orientation_ros = frames::px4ToRosQuaternion(odometry.q);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = position_enu.x;
    transform.transform.translation.y = position_enu.y;
    transform.transform.translation.z = position_enu.z;
    transform.transform.rotation = orientation_ros;

    broadcaster_->sendTransform(transform);
  }

  void warnThrottled(const char * reason)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", reason);
  }

  std::string uav_id_;
  std::string odom_frame_;
  std::string base_frame_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_subscription_;
};

}  // namespace uav_px4_backend

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_px4_backend::Px4FrameBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
