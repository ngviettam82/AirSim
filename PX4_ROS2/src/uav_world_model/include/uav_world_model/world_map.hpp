#ifndef UAV_WORLD_MODEL__WORLD_MAP_HPP_
#define UAV_WORLD_MODEL__WORLD_MAP_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "uav_world_model/frame_transforms.hpp"
#include "uav_world_model/uncertainty.hpp"

namespace uav_world_model
{

struct LandmarkObservation
{
  int32_t marker_id{0};
  uint8_t family{0};
  Vec3 position;
  Quat orientation;
  double position_uncertainty_m{-1.0};
  double stamp_sec{0.0};
};

struct ReportedLandmark
{
  int32_t marker_id{0};
  uint8_t family{0};
  Vec3 position;
  Quat orientation;
  double position_uncertainty_m{-1.0};
  double time_since_seen_sec{0.0};
};

struct ObstacleObservation
{
  int32_t obstacle_id{-1};
  uint8_t shape{0};
  Vec3 center;
  Vec3 size;
  double confidence{0.0};
  double position_uncertainty_m{-1.0};
};

struct ObstacleBatch
{
  std::string sensing_frame;
  double stamp_sec{0.0};
  double sensing_range_m{0.0};
  std::vector<ObstacleObservation> obstacles;
};

struct ReportedObstacle
{
  ObstacleObservation observation;
  double time_since_seen_sec{0.0};
};

// Mirrors TargetTrack::STATUS_LOST; the node static_asserts the pair.
constexpr uint8_t kTargetStatusLost = 3;

struct TargetObservation
{
  int32_t track_id{0};
  uint8_t status{0};
  Vec3 position;
  Quat orientation;
  Vec3 velocity;
  double confidence{0.0};
  double position_uncertainty_m{-1.0};
  double stamp_sec{0.0};
  // Age of the sighting at stamp_sec, not of the message.
  double time_since_seen_sec{0.0};
};

struct ReportedTarget
{
  bool valid{false};
  TargetObservation observation;
  double position_uncertainty_m{-1.0};
  double time_since_seen_sec{0.0};
};

struct WorldMapOptions
{
  double landmark_forget_sec{120.0};
  double obstacle_forget_sec{5.0};
  double target_forget_sec{10.0};
  double target_track_switch_after_sec{1.0};
  DriftModel drift;
};

class WorldMap
{
public:
  explicit WorldMap(WorldMapOptions options = {});

  void observeLandmark(const LandmarkObservation & observation);
  void observeObstacles(const ObstacleBatch & batch);
  void observeTarget(const TargetObservation & observation);

  void expire(double now_sec);

  std::vector<ReportedLandmark> landmarks(double now_sec) const;
  std::vector<ReportedObstacle> obstacles(double now_sec) const;
  ReportedTarget target(double now_sec) const;

  bool hasObstacleSource() const;
  bool hasSurvivingBatch() const;
  double newestObstacleStamp() const;
  // The map may not look fresher than its contents.
  double oldestObstacleStamp() const;
  double obstacleSensingRange() const;

  std::size_t landmarkCount() const;
  uint64_t targetTrackSwitchCount() const;
  uint64_t unselectedTargetTrackCount() const;

private:
  using LandmarkKey = std::pair<uint8_t, int32_t>;

  bool targetSelectionIsOpen(const TargetObservation & candidate) const;
  bool prefersCandidateTargetTrack(const TargetObservation & candidate) const;
  void selectTargetTrack(const TargetObservation & observation);

  WorldMapOptions options_;
  std::map<LandmarkKey, LandmarkObservation> landmarks_;
  std::map<std::string, ObstacleBatch> obstacle_batches_;
  TargetObservation target_;
  bool has_target_{false};
  double target_selection_stamp_sec_{0.0};
  uint64_t target_track_switches_{0};
  uint64_t unselected_target_tracks_{0};
  bool has_obstacle_source_{false};
  double newest_obstacle_stamp_{0.0};
};

}  // namespace uav_world_model

#endif  // UAV_WORLD_MODEL__WORLD_MAP_HPP_
