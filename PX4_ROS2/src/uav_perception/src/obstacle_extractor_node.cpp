// Frame stays optical; body/map conversion is world_model's job (see README).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <uav_interfaces/msg/obstacle.hpp>
#include <uav_interfaces/msg/obstacle_array.hpp>

#include "uav_perception/obstacle_extraction.hpp"

namespace
{

using nav_msgs::msg::Odometry;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using uav_interfaces::msg::Obstacle;
using uav_interfaces::msg::ObstacleArray;

// Same helper/formula as uav_navigation's local_planner_node.cpp -- avoids
// rclcpp::Time-vs-builtin_interfaces::msg::Time clock-source mismatches.
double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

class ObstacleExtractorNode : public rclcpp::Node
{
public:
  ObstacleExtractorNode()
  : Node("obstacle_extractor_node")
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    const std::string camera = declare_parameter<std::string>("camera", "front");
    optical_frame_ = declare_parameter<std::string>(
      "optical_frame", uav_id_ + "/camera_" + camera + "_optical");

    // Sourced from sensor_depth_front/model.sdf clip near/far (see README).
    params_.min_range_m = declare_parameter<double>("min_range_m", 0.2);
    params_.max_range_m = declare_parameter<double>("max_range_m", 19.1);
    params_.cluster_depth_tolerance_m =
      declare_parameter<double>("cluster_depth_tolerance_m", 0.3);
    params_.max_cluster_depth_span_m =
      declare_parameter<double>("max_cluster_depth_span_m", 2.0);
    params_.pixel_stride = static_cast<std::size_t>(
      declare_parameter<int>("pixel_stride", 2));
    params_.min_cluster_points = static_cast<std::size_t>(
      declare_parameter<int>("min_cluster_points", 30));
    params_.confidence_saturating_points =
      declare_parameter<double>("confidence_saturating_points", 150.0);
    params_.max_obstacles = static_cast<std::size_t>(
      declare_parameter<int>("max_obstacles", 20));

    // Ground-plane pre-filter (README "phan manh nen"); on by default.
    params_.enable_ground_filter = declare_parameter<bool>("enable_ground_filter", true);
    params_.ground_margin_m = declare_parameter<double>("ground_margin_m", 0.05);
    params_.ground_min_inlier_ratio =
      declare_parameter<double>("ground_min_inlier_ratio", 0.15);
    // Raw int kept until validated below: a negative value cast straight to
    // size_t wraps to a huge floor the ratio/depth gates can never clear --
    // the filter would die silently instead of refusing to start (R0).
    ground_min_inlier_points_raw_ = declare_parameter<int>("ground_min_inlier_points", 200);
    params_.ground_min_inlier_points = static_cast<std::size_t>(
      std::max(0, ground_min_inlier_points_raw_));
    // Round 3 (G-M3 gate re-block, 2026-08-22): the safety gate that makes
    // the lowered ratio above safe -- see README/obstacle_extraction.cpp.
    params_.ground_min_depth_below_camera_m =
      declare_parameter<double>("ground_min_depth_below_camera_m", 0.15);
    // R32: sample-and-hold must carry an age and self-expire. Tier 2 (README
    // "bu nghieng") is the old, level-only behaviour, not "skip the filter".
    odometry_max_age_sec_ = declare_parameter<double>("odometry_max_age_sec", 0.5);
    validateGroundFilterParams();

    const std::string prefix = "/uav/" + uav_id_ + "/perception";

    depth_subscription_ = create_subscription<Image>(
      prefix + "/" + camera + "/depth_image", rclcpp::SensorDataQoS(),
      [this](const Image::SharedPtr message) {onDepth(*message);});
    info_subscription_ = create_subscription<CameraInfo>(
      prefix + "/" + camera + "/camera_info", rclcpp::SensorDataQoS(),
      [this](const CameraInfo::SharedPtr message) {onCameraInfo(*message);});
    // Orientation ONLY -- never position, see README. Single-threaded spin()
    // in main() (no callback groups/MultiThreadedExecutor), so this and
    // onDepth/onCameraInfo never run concurrently -- R24 is a non-issue here
    // by construction, not by locking; do not add a MultiThreadedExecutor
    // without revisiting this.
    odometry_subscription_ = create_subscription<Odometry>(
      "/uav/" + uav_id_ + "/state/odometry_fused", rclcpp::SensorDataQoS(),
      [this](const Odometry::SharedPtr message) {latest_odometry_ = message;});

    publisher_ = create_publisher<ObstacleArray>(
      "/uav/" + uav_id_ + "/perception/obstacles_local", rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(), "obstacle extractor ready for %s, %s depth camera",
      uav_id_.c_str(), camera.c_str());
  }

private:
  // Refuses to start rather than fly with a ground filter that could pass
  // silently as a no-op (margin<=0) or as an over-eager scrub of real returns
  // (ratio out of (0,1]) -- same "refuse at startup" style as uav_navigation.
  void validateGroundFilterParams()
  {
    std::string reason;
    if (!std::isfinite(params_.ground_margin_m) || params_.ground_margin_m <= 0.0) {
      reason = "ground_margin_m must be finite and positive";
    } else if (!std::isfinite(params_.ground_min_inlier_ratio) ||
      params_.ground_min_inlier_ratio <= 0.0 || params_.ground_min_inlier_ratio > 1.0)
    {
      reason = "ground_min_inlier_ratio must be in (0, 1]";
    } else if (ground_min_inlier_points_raw_ <= 0) {
      // <=0 would let the ratio/count gates trip on almost nothing (0) or,
      // cast raw, wrap to a huge floor the filter can never clear -- either
      // way a silently dead filter, not a refuse-to-start (R0, 🟢 review).
      reason = "ground_min_inlier_points must be a positive integer";
    } else if (!std::isfinite(params_.ground_min_depth_below_camera_m) ||
      params_.ground_min_depth_below_camera_m <= 0.0)
    {
      // ==0 would let a candidate exactly AT camera height pass -- a wall
      // dead ahead, not ground. Must stay strictly positive (R0).
      reason = "ground_min_depth_below_camera_m must be finite and positive";
    } else if (!std::isfinite(odometry_max_age_sec_) || odometry_max_age_sec_ <= 0.0) {
      reason = "odometry_max_age_sec must be finite and positive";
    }
    if (!reason.empty()) {
      RCLCPP_FATAL(get_logger(), "ground filter params refused: %s", reason.c_str());
      throw std::runtime_error("ground filter params refused: " + reason);
    }
  }

  void onCameraInfo(const CameraInfo & message)
  {
    intrinsics_ = uav_perception::depthIntrinsicsFrom(message.k);
    has_intrinsics_ = intrinsics_.valid();
  }

  void onDepth(const Image & message)
  {
    // No intrinsics yet: backprojection would be meaningless.
    if (!has_intrinsics_) {
      throttledWarning("no camera_info yet, not extracting obstacles");
      return;
    }
    if (message.encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
      throttledWarning("unsupported depth encoding: " + message.encoding);
      return;
    }

    cv::Mat depth;
    try {
      // Pass-through encoding only: no cv_bridge auto-convert of depth units.
      depth = cv_bridge::toCvCopy(message, message.encoding)->image;
    } catch (const cv_bridge::Exception & error) {
      throttledWarning(std::string("cannot read depth image: ") + error.what());
      return;
    }
    if (!depth.isContinuous()) {
      depth = depth.clone();
    }

    const auto extracted = uav_perception::extractObstacles(
      depth.ptr<float>(0), static_cast<std::size_t>(depth.cols),
      static_cast<std::size_t>(depth.rows), intrinsics_, params_, groundFilterAttitude());

    publish(message.header, extracted);
  }

  // Two-tier degrade (R30/R32): fresh, finite odometry orientation, else
  // identity (the pre-round-2 level-only behaviour, see obstacle_extraction
  // .cpp). Position is never read -- only pose.pose.orientation.
  uav_perception::AttitudeQuaternion groundFilterAttitude() const
  {
    const bool have_odometry = static_cast<bool>(latest_odometry_);
    uav_perception::AttitudeQuaternion orientation;
    double age_sec = std::numeric_limits<double>::infinity();
    if (have_odometry) {
      const auto & q = latest_odometry_->pose.pose.orientation;
      orientation = uav_perception::AttitudeQuaternion{q.x, q.y, q.z, q.w};
      age_sec = now().seconds() - stampSeconds(latest_odometry_->header.stamp);
    }
    return uav_perception::resolveGroundFilterAttitude(
      have_odometry, orientation, age_sec, odometry_max_age_sec_);
  }

  void publish(
    const std_msgs::msg::Header & header,
    const std::vector<uav_perception::ExtractedObstacle> & extracted)
  {
    ObstacleArray array;
    array.header.stamp = header.stamp;
    // frame_id names the optical frame, not the camera link.
    array.header.frame_id = optical_frame_;
    array.uav_id = uav_id_;
    array.sensing_range = static_cast<float>(params_.max_range_m);

    for (std::size_t index = 0; index < extracted.size(); ++index) {
      const auto & source = extracted[index];
      Obstacle obstacle;
      obstacle.obstacle_id = -(static_cast<std::int32_t>(index) + 1);
      obstacle.shape = Obstacle::SHAPE_BOX;
      obstacle.center.x = source.center_x;
      obstacle.center.y = source.center_y;
      obstacle.center.z = source.center_z;
      obstacle.size.x = source.size_x;
      obstacle.size.y = source.size_y;
      obstacle.size.z = source.size_z;
      obstacle.distance = static_cast<float>(source.distance);
      obstacle.confidence = static_cast<float>(source.confidence);
      // No noise model on this placeholder depth sensor yet (see README).
      obstacle.position_uncertainty = -1.0f;
      array.obstacles.push_back(obstacle);
    }

    publisher_->publish(array);
  }

  void throttledWarning(const std::string & text)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", text.c_str());
  }

  std::string uav_id_;
  std::string optical_frame_;
  uav_perception::ExtractionParams params_;
  double odometry_max_age_sec_{0.5};
  int ground_min_inlier_points_raw_{200};

  uav_perception::DepthCameraIntrinsics intrinsics_;
  bool has_intrinsics_{false};
  Odometry::SharedPtr latest_odometry_;

  rclcpp::Subscription<Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<ObstacleArray>::SharedPtr publisher_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleExtractorNode>());
  rclcpp::shutdown();
  return 0;
}
