#include "uav_perception/camera_health_checks.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>

namespace uav_perception
{

void StreamDelivery::record(double stamp_sec)
{
  ++received_;
  if (!has_last_) {
    last_stamp_ = stamp_sec;
    has_last_ = true;
    return;
  }

  const double gap = stamp_sec - last_stamp_;
  last_stamp_ = stamp_sec;
  // Backward stamp = clock reset, not an early frame.
  if (gap <= 0.0) {
    return;
  }

  gaps_.push_back(gap);
  if (shortest_gap_ <= 0.0 || gap < shortest_gap_) {
    shortest_gap_ = gap;
  }
}

double StreamDelivery::sourceHz() const
{
  return shortest_gap_ > 0.0 ? 1.0 / shortest_gap_ : 0.0;
}

double StreamDelivery::deliveredHz() const
{
  if (gaps_.empty()) {
    return 0.0;
  }
  const double span = std::accumulate(gaps_.begin(), gaps_.end(), 0.0);
  return span > 0.0 ? static_cast<double>(gaps_.size()) / span : 0.0;
}

double StreamDelivery::deliveredFraction() const
{
  const double source = sourceHz();
  return source > 0.0 ? std::min(1.0, deliveredHz() / source) : 0.0;
}

double StreamDelivery::longestGapSec() const
{
  if (gaps_.empty()) {
    return 0.0;
  }
  return *std::max_element(gaps_.begin(), gaps_.end());
}

std::size_t StreamDelivery::lostFrames() const
{
  if (shortest_gap_ <= 0.0) {
    return 0;
  }
  std::size_t lost = 0;
  for (const double gap : gaps_) {
    // periods - 1 = frames lost in this gap.
    const double periods = std::round(gap / shortest_gap_);
    if (periods > 1.0) {
      lost += static_cast<std::size_t>(periods - 1.0);
    }
  }
  return lost;
}

void StreamDelivery::startWindow()
{
  gaps_.clear();
  received_ = 0;
  has_last_ = false;
  // shortest_gap_ deliberately survives: it describes the camera, not the window.
}

HealthLevel classifyDelivery(double delivered_fraction, const DeliveryCheck & check)
{
  if (delivered_fraction < check.error_fraction) {
    return HealthLevel::Error;
  }
  if (delivered_fraction < check.warn_fraction) {
    return HealthLevel::Warn;
  }
  return HealthLevel::Ok;
}

HealthLevel classifyGap(double longest_gap_sec, const GapCheck & check)
{
  if (longest_gap_sec >= check.error_sec) {
    return HealthLevel::Error;
  }
  if (longest_gap_sec >= check.warn_sec) {
    return HealthLevel::Warn;
  }
  return HealthLevel::Ok;
}

std::size_t distinctValuesAcross(
  const std::uint8_t * data, std::size_t size, std::size_t samples)
{
  if (data == nullptr || size == 0) {
    return 0;
  }
  const std::size_t stride = std::max<std::size_t>(1, size / std::max<std::size_t>(1, samples));
  std::set<std::uint8_t> seen;
  for (std::size_t index = 0; index < size; index += stride) {
    seen.insert(data[index]);
  }
  return seen.size();
}

HealthLevel classifyContent(std::size_t distinct_values, const ContentCheck & check)
{
  return distinct_values < check.min_distinct ? HealthLevel::Warn : HealthLevel::Ok;
}

bool infoMatchesImage(
  std::uint32_t info_width, std::uint32_t info_height, double focal_x,
  std::uint32_t image_width, std::uint32_t image_height)
{
  return info_width == image_width && info_height == image_height && focal_x > 0.0;
}

HealthLevel worst(HealthLevel a, HealthLevel b)
{
  return static_cast<int>(a) >= static_cast<int>(b) ? a : b;
}

}  // namespace uav_perception
