// Delivery loss is invisible to rate alone; see README.

#ifndef UAV_PERCEPTION__CAMERA_HEALTH_CHECKS_HPP_
#define UAV_PERCEPTION__CAMERA_HEALTH_CHECKS_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace uav_perception
{

enum class HealthLevel
{
  Ok = 0,
  Warn = 1,
  Error = 2
};

// Source rate inferred from shortest gap, never configured.
class StreamDelivery
{
public:
  void record(double stamp_sec);

  bool ready() const {return gaps_.size() >= 2;}
  double sourceHz() const;
  double deliveredHz() const;
  double deliveredFraction() const;
  double longestGapSec() const;
  std::size_t lostFrames() const;
  std::size_t received() const {return received_;}

  // Resets window stats only; source rate survives (see README).
  void startWindow();

private:
  std::vector<double> gaps_;
  double last_stamp_{0.0};
  double shortest_gap_{0.0};
  std::size_t received_{0};
  bool has_last_{false};
};

struct DeliveryCheck
{
  double warn_fraction{0.8};
  double error_fraction{0.5};
};

HealthLevel classifyDelivery(double delivered_fraction, const DeliveryCheck & check);

struct GapCheck
{
  // Warn: tracker coasting ends. Error: guessing (see README).
  double warn_sec{0.3};
  double error_sec{1.0};
};

HealthLevel classifyGap(double longest_gap_sec, const GapCheck & check);

// Even sampling; corner-only sampling misread sky as dead (see README).
std::size_t distinctValuesAcross(
  const std::uint8_t * data, std::size_t size, std::size_t samples = 4096);

struct ContentCheck
{
  std::size_t min_distinct{5};
};

// Warn only: sky looks flat too, not just a dead sensor.
HealthLevel classifyContent(std::size_t distinct_values, const ContentCheck & check);

// Catches Gazebo's shared camera_info collision (see README).
bool infoMatchesImage(
  std::uint32_t info_width, std::uint32_t info_height, double focal_x,
  std::uint32_t image_width, std::uint32_t image_height);

HealthLevel worst(HealthLevel a, HealthLevel b);

}  // namespace uav_perception

#endif  // UAV_PERCEPTION__CAMERA_HEALTH_CHECKS_HPP_
