#pragma once

#include "px4_airsim_autonomy/algorithm_base.hpp"
#include "px4_airsim_autonomy/production/photogrammetry_calc.hpp"
#include "px4_airsim_autonomy/production/boustrophedon_planner.hpp"
#include "px4_airsim_autonomy/production/mission_continuity.hpp"

#include <vector>
#include <memory>

namespace px4_airsim_autonomy {
namespace enterprise {

/**
 * @brief Production Enterprise Photogrammetry Survey Mission.
 * 
 * Replaces simplistic grid patrolling with commercial-grade photogrammetry:
 * - Freeman-Shapira minimum-turn Boustrophedon path planning.
 * - Exact optical GSD, overlap triggering, and shutter blur speed limits.
 * - Smart Oblique Capture (SOC) 5-way gimbal sequences.
 * - Breakpoint serialization and dynamic backtrack resumption buffers.
 */
class PhotogrammetrySurveyMission : public IAutonomyAlgorithm {
public:
    PhotogrammetrySurveyMission();
    ~PhotogrammetrySurveyMission() override = default;

    std::string getName() const override { return "photogrammetry_survey"; }

    void init(rclcpp::Node& node) override;
    void onActivate() override;
    void onDeactivate() override;
    AutonomyCommand update(const SensorSnapshot& sensors, float dt) override;
    void reset() override;

    // Mission controls
    bool saveBreakpoint(const std::string& filepath, const SensorSnapshot& sensors);
    bool resumeFromBreakpoint(const std::string& filepath);

    size_t getCurrentWaypointIndex() const { return current_wp_idx_; }
    size_t getTotalWaypoints() const { return mission_waypoints_.size(); }
    double getTargetGsdCm() const { return target_gsd_cm_; }
    double getForwardOverlap() const { return forward_overlap_; }
    double getSideOverlap() const { return side_overlap_; }
    double getOptimalAngleDeg() const { return optimal_angle_deg_; }

private:
    void generateSurveyPlan();

    production::CameraSpec camera_spec_;
    production::SurveyGridMetrics grid_metrics_;
    std::vector<production::SweepWaypoint> mission_waypoints_;
    size_t current_wp_idx_{0};

    // Configuration parameters
    double target_altitude_agl_{30.0};
    double target_gsd_cm_{1.5};
    double forward_overlap_{0.75};
    double side_overlap_{0.65};
    double acceptance_radius_{2.0};
    double survey_speed_m_s_{4.0};
    double optimal_angle_deg_{0.0};

    std::vector<Eigen::Vector2d> survey_polygon_;
    bool plan_generated_{false};
    bool is_resuming_{false};
    double backtrack_distance_m_{0.0};
};

} // namespace enterprise
} // namespace px4_airsim_autonomy
