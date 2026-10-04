// Not a camera driver; sim bridge covers that (see README).

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "uav_perception/camera_health_checks.hpp"

namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using uav_perception::HealthLevel;

KeyValue entry(const std::string & key, const std::string & value)
{
  KeyValue pair;
  pair.key = key;
  pair.value = value;
  return pair;
}

std::string number(double value, int decimals = 2)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
  return std::string(buffer);
}

class CameraHealthNode : public rclcpp::Node
{
public:
  CameraHealthNode()
  : Node("camera_health_node")
  {
    uav_id_ = declare_parameter<std::string>("uav_id", "uav0");
    const std::string prefix = "/uav/" + uav_id_;

    delivery_.warn_fraction = declare_parameter<double>("delivery_warn_fraction", 0.8);
    delivery_.error_fraction = declare_parameter<double>("delivery_error_fraction", 0.5);
    gap_.warn_sec = declare_parameter<double>("gap_warn_sec", 0.3);
    gap_.error_sec = declare_parameter<double>("gap_error_sec", 1.0);
    content_.min_distinct = static_cast<std::size_t>(
      declare_parameter<int>("min_distinct_values", 5));
    stale_after_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("stale_after_sec", 2.0));

    watch("front/rgb", prefix + "/perception/front/image_raw",
      prefix + "/perception/front/camera_info");
    watch("front/depth", prefix + "/perception/front/depth_image", "");
    watch("down/rgb", prefix + "/perception/down/image_raw",
      prefix + "/perception/down/camera_info");

    diagnostics_publisher_ = create_publisher<DiagnosticArray>(
      prefix + "/diagnostics/perception", rclcpp::QoS(10));
    summary_publisher_ = create_publisher<DiagnosticStatus>(
      prefix + "/state/camera_health", rclcpp::QoS(10));

    report_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(
        declare_parameter<double>("report_period_sec", 1.0)),
      [this]() {report();});

    RCLCPP_INFO(get_logger(), "camera health ready for %s", uav_id_.c_str());
  }

private:
  struct Stream
  {
    std::string name;
    uav_perception::StreamDelivery delivery;
    rclcpp::Time last_frame{0, 0, RCL_ROS_TIME};
    bool has_frame{false};
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::size_t distinct{0};
    bool has_info{false};
    std::uint32_t info_width{0};
    std::uint32_t info_height{0};
    double focal_x{0.0};
    rclcpp::Subscription<Image>::SharedPtr image_subscription;
    rclcpp::Subscription<CameraInfo>::SharedPtr info_subscription;
  };

  void watch(
    const std::string & name, const std::string & image_topic,
    const std::string & info_topic)
  {
    auto stream = std::make_unique<Stream>();
    stream->name = name;
    Stream * raw = stream.get();

    raw->image_subscription = create_subscription<Image>(
      image_topic, rclcpp::SensorDataQoS(),
      [this, raw](const Image::SharedPtr message) {onImage(*raw, *message);});

    if (!info_topic.empty()) {
      raw->info_subscription = create_subscription<CameraInfo>(
        info_topic, rclcpp::SensorDataQoS(),
        [raw](const CameraInfo::SharedPtr message) {
          raw->has_info = true;
          raw->info_width = message->width;
          raw->info_height = message->height;
          raw->focal_x = message->k[0];
        });
    }

    streams_.push_back(std::move(stream));
  }

  void onImage(Stream & stream, const Image & message)
  {
    const double stamp =
      static_cast<double>(message.header.stamp.sec) + message.header.stamp.nanosec * 1e-9;
    stream.delivery.record(stamp);
    stream.last_frame = now();
    stream.has_frame = true;
    stream.width = message.width;
    stream.height = message.height;
    stream.distinct = uav_perception::distinctValuesAcross(
      message.data.data(), message.data.size());
  }

  DiagnosticStatus judge(Stream & stream)
  {
    DiagnosticStatus status;
    status.name = "camera: " + stream.name;
    status.hardware_id = uav_id_;

    const bool stale = !stream.has_frame || (now() - stream.last_frame) > stale_after_;
    if (stale) {
      status.level = DiagnosticStatus::ERROR;
      status.message = stream.has_frame ? "no frames" : "never received a frame";
      status.values.push_back(entry("received", "0"));
      stream.delivery.startWindow();
      return status;
    }

    if (!stream.delivery.ready()) {
      status.level = DiagnosticStatus::WARN;
      status.message = "too few frames to judge delivery";
      status.values.push_back(
        entry("received", std::to_string(stream.delivery.received())));
      stream.delivery.startWindow();
      return status;
    }

    const double fraction = stream.delivery.deliveredFraction();
    const double longest_gap = stream.delivery.longestGapSec();

    HealthLevel level = uav_perception::classifyDelivery(fraction, delivery_);
    level = uav_perception::worst(level, uav_perception::classifyGap(longest_gap, gap_));
    level = uav_perception::worst(
      level, uav_perception::classifyContent(stream.distinct, content_));

    std::string message = "ok";
    if (uav_perception::classifyDelivery(fraction, delivery_) != HealthLevel::Ok) {
      message = "losing frames";
    } else if (uav_perception::classifyGap(longest_gap, gap_) != HealthLevel::Ok) {
      message = "frame gap too long";
    } else if (uav_perception::classifyContent(stream.distinct, content_) !=
      HealthLevel::Ok)
    {
      // Cannot distinguish a dead sensor from one facing uniform sky.
      message = "flat image, may be legitimate";
    }

    if (stream.has_info && !uav_perception::infoMatchesImage(
        stream.info_width, stream.info_height, stream.focal_x,
        stream.width, stream.height))
    {
      level = HealthLevel::Error;
      message = "camera_info does not match the image";
    }

    status.level = static_cast<unsigned char>(level);
    status.message = message;
    status.values.push_back(entry("source_hz", number(stream.delivery.sourceHz(), 1)));
    status.values.push_back(entry("delivered_hz", number(stream.delivery.deliveredHz(), 1)));
    status.values.push_back(entry("delivered_fraction", number(fraction)));
    status.values.push_back(entry("longest_gap_sec", number(longest_gap)));
    status.values.push_back(entry("lost_frames", std::to_string(stream.delivery.lostFrames())));
    status.values.push_back(entry("received", std::to_string(stream.delivery.received())));
    status.values.push_back(
      entry("resolution", std::to_string(stream.width) + "x" + std::to_string(stream.height)));
    status.values.push_back(entry("distinct_values", std::to_string(stream.distinct)));

    stream.delivery.startWindow();
    return status;
  }

  void report()
  {
    DiagnosticArray array;
    array.header.stamp = now();

    HealthLevel overall = HealthLevel::Ok;
    for (auto & stream : streams_) {
      DiagnosticStatus status = judge(*stream);
      overall = uav_perception::worst(overall, static_cast<HealthLevel>(status.level));
      array.status.push_back(status);
    }
    diagnostics_publisher_->publish(array);

    DiagnosticStatus summary;
    summary.name = "perception: cameras";
    summary.hardware_id = uav_id_;
    summary.level = static_cast<unsigned char>(overall);
    summary.message = overall == HealthLevel::Ok ? "all streams healthy" :
      "at least one stream degraded";
    summary_publisher_->publish(summary);
  }

  std::string uav_id_;
  std::vector<std::unique_ptr<Stream>> streams_;
  uav_perception::DeliveryCheck delivery_;
  uav_perception::GapCheck gap_;
  uav_perception::ContentCheck content_;
  rclcpp::Duration stale_after_{rclcpp::Duration::from_seconds(2.0)};
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr summary_publisher_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraHealthNode>());
  rclcpp::shutdown();
  return 0;
}
