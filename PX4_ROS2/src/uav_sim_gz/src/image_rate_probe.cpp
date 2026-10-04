// C++ cross-check for Debt #10 (docs/package-status.md); rate from header stamps.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

struct StreamStats
{
  std::string label;
  std::size_t count = 0;
  double first_stamp = 0.0;
  double last_stamp = 0.0;
  std::vector<double> gaps;
  std::size_t payload_bytes = 0;
  std::size_t bytes_per_message = 0;
  std::chrono::steady_clock::time_point first_arrival;
  std::chrono::steady_clock::time_point last_arrival;

  void record(double stamp, std::size_t bytes = 0)
  {
    payload_bytes += bytes;
    bytes_per_message = bytes;
    const auto now = std::chrono::steady_clock::now();
    if (count == 0) {
      first_stamp = stamp;
      first_arrival = now;
    } else {
      gaps.push_back(stamp - last_stamp);
    }
    last_stamp = stamp;
    last_arrival = now;
    ++count;
  }
};

double stampSeconds(const std_msgs::msg::Header & header)
{
  return static_cast<double>(header.stamp.sec) + header.stamp.nanosec * 1e-9;
}

void report(const StreamStats & stream)
{
  if (stream.count < 3) {
    std::printf("  %-26s only %zu messages\n", stream.label.c_str(), stream.count);
    return;
  }

  const double simulated_span = stream.last_stamp - stream.first_stamp;
  const double wall_span = std::chrono::duration<double>(
    stream.last_arrival - stream.first_arrival).count();
  const double delivered_hz = simulated_span > 0.0
    ? static_cast<double>(stream.count - 1) / simulated_span : 0.0;
  const double wall_hz = wall_span > 0.0
    ? static_cast<double>(stream.count - 1) / wall_span : 0.0;

  std::vector<double> sorted = stream.gaps;
  std::sort(sorted.begin(), sorted.end());
  const double shortest_gap = sorted.front();
  const double median_gap = sorted[sorted.size() / 2];
  const double source_hz = shortest_gap > 0.0 ? 1.0 / shortest_gap : 0.0;

  // gap/shortest_gap - 1 counts frames lost within that gap.
  std::size_t lost = 0;
  for (const double gap : stream.gaps) {
    lost += static_cast<std::size_t>(std::max(0.0, std::round(gap / shortest_gap) - 1.0));
  }

  std::printf(
    "  %-26s %4zu msgs | source %5.1f Hz | delivered %5.1f Hz (%3.0f%%) | "
    "wall %5.1f Hz | lost %zu | gap p50 %.1f ms max %.1f ms\n",
    stream.label.c_str(), stream.count, source_hz, delivered_hz,
    source_hz > 0.0 ? 100.0 * delivered_hz / source_hz : 0.0,
    wall_hz, lost, median_gap * 1e3, sorted.back() * 1e3);

  // The point of this line: with every image ~1 MiB, "29.7 msg/s" and
  // "28.2 MiB/s" are the SAME number, so no measurement so far could tell a
  // message-rate ceiling from a bandwidth one. Resolution is the discriminator.
  const double mib = 1024.0 * 1024.0;
  const double delivered_mib_s = wall_span > 0.0
    ? static_cast<double>(stream.payload_bytes) / mib / wall_span : 0.0;
  std::printf(
    "  %-26s                            %8.3f MiB/msg | %7.2f MiB/s\n",
    stream.label.c_str(),
    static_cast<double>(stream.bytes_per_message) / mib, delivered_mib_s);
}

class ImageRateProbe : public rclcpp::Node
{
public:
  ImageRateProbe()
  : Node("image_rate_probe")
  {
    const std::string prefix =
      "/uav/" + declare_parameter<std::string>("uav_id", "uav0") + "/perception";

    addImage("front/image_raw", prefix + "/front/image_raw");
    addInfo("front/camera_info", prefix + "/front/camera_info");
    addImage("front/depth_image", prefix + "/front/depth_image");
    addImage("down/image_raw", prefix + "/down/image_raw");
    addInfo("down/camera_info", prefix + "/down/camera_info");
  }

  void reportAll(double window_sec) const
  {
    for (const auto & stream : streams_) {
      report(*stream);
    }

    // The aggregate is the number that decides the debt: if MiB/s is what stays
    // put across resolutions, resolution is the lever and the frame rate never
    // has to move (which is what P4's optical flow actually cares about).
    std::size_t total_messages = 0;
    std::size_t total_bytes = 0;
    for (const auto & stream : streams_) {
      if (stream->bytes_per_message == 0) {
        continue;                     // camera_info: a fixed tiny struct
      }
      total_messages += stream->count;
      total_bytes += stream->payload_bytes;
    }
    const double mib = 1024.0 * 1024.0;
    std::printf(
      "\n  IMAGE STREAMS TOTAL over %.0f s: %7.2f msg/s | %7.2f MiB/s\n",
      window_sec,
      window_sec > 0.0 ? static_cast<double>(total_messages) / window_sec : 0.0,
      window_sec > 0.0 ? static_cast<double>(total_bytes) / mib / window_sec : 0.0);
  }

private:
  void addImage(const std::string & label, const std::string & topic)
  {
    auto stream = std::make_shared<StreamStats>();
    stream->label = label;
    streams_.push_back(stream);
    image_subscriptions_.push_back(
      create_subscription<Image>(
        topic, rclcpp::SensorDataQoS(),
        [stream](const Image::SharedPtr message) {
          stream->record(stampSeconds(message->header), message->data.size());
        }));
  }

  void addInfo(const std::string & label, const std::string & topic)
  {
    auto stream = std::make_shared<StreamStats>();
    stream->label = label;
    streams_.push_back(stream);
    info_subscriptions_.push_back(
      create_subscription<CameraInfo>(
        topic, rclcpp::SensorDataQoS(),
        [stream](const CameraInfo::SharedPtr message) {
          stream->record(stampSeconds(message->header));
        }));
  }

  std::vector<std::shared_ptr<StreamStats>> streams_;
  std::vector<rclcpp::Subscription<Image>::SharedPtr> image_subscriptions_;
  std::vector<rclcpp::Subscription<CameraInfo>::SharedPtr> info_subscriptions_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageRateProbe>();

  const double seconds = node->declare_parameter<double>("seconds", 30.0);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(seconds);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::printf("\nC++ consumer, %.0f s window:\n", seconds);
  node->reportAll(seconds);
  rclcpp::shutdown();
  return 0;
}
