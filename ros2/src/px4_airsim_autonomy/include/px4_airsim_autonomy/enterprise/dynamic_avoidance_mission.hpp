#pragma once

#include "px4_airsim_autonomy/algorithm_base.hpp"
#include "px4_airsim_autonomy/production/dynamic_stopping_bubble.hpp"
#include "px4_airsim_autonomy/production/safe_flight_corridor.hpp"

namespace px4_airsim_autonomy {
namespace enterprise {

/**
 * @brief Production Enterprise Dynamic Avoidance Mission.
 * 
 * Replaces simplistic repulsive potential fields with:
 * - Velocity-aligned 3D dynamic stopping ellipsoid ($d_{stop}(v)$).
 * - Safe Flight Corridors (SFC) with convex polyhedral decomposition.
 * - Transverse jerk bounds $\|j_\perp\| \le 10.0$ m/s$^3$ to prevent rate-gyro clipping.
 */
class DynamicAvoidanceMission : public IAutonomyAlgorithm {
public:
    DynamicAvoidanceMission();
    ~DynamicAvoidanceMission() override = default;

    std::string getName() const override { return "dynamic_avoidance"; }

    void init(rclcpp::Node& node) override;
    void onActivate() override;
    void onDeactivate() override;
    AutonomyCommand update(const SensorSnapshot& sensors, float dt) override;
    void reset() override;

    float getCruiseSpeed() const { return cruise_speed_; }
    float getStoppingDistance(float speed) const { return bubble_.computeStoppingDistance(speed); }

private:
    production::DynamicStoppingBubble bubble_;
    production::SafeFlightCorridor corridor_;

    float cruise_speed_{2.0f};
    float max_accel_{3.0f};
    float max_jerk_{10.0f};
    float safety_margin_{1.5f};
    float cruise_altitude_{10.0f};

    Eigen::Vector3f current_velocity_cmd_{Eigen::Vector3f::Zero()};
};

} // namespace enterprise
} // namespace px4_airsim_autonomy
