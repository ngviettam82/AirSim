#pragma once

#include "types.hpp"
#include <rclcpp/rclcpp.hpp>

namespace px4_airsim_autonomy {

/**
 * @brief Base interface for all modular autonomy algorithms.
 * Every custom behavior (obstacle avoidance, scanning, guiding, finding)
 * implements this interface in its own dedicated file.
 */
class IAutonomyAlgorithm {
public:
    virtual ~IAutonomyAlgorithm() = default;

    /**
     * @brief Unique identifier for this algorithm (e.g. "obstacle_avoidance", "scanning_patrol")
     */
    virtual std::string getName() const = 0;

    /**
     * @brief One-time initialization to declare parameters or initialize internal state.
     */
    virtual void init(rclcpp::Node& node) = 0;

    /**
     * @brief Called when PX4 activates this flight mode.
     */
    virtual void onActivate() = 0;

    /**
     * @brief Called when PX4 leaves this flight mode.
     */
    virtual void onDeactivate() = 0;

    /**
     * @brief Main computation loop called at rate dt (typically 20 Hz / 50ms).
     * @param sensors Latest processed sensor data (Depth, odometry, target info).
     * @param dt Elapsed seconds since last update.
     * @return AutonomyCommand Desired velocity or position setpoint.
     */
    virtual AutonomyCommand update(const SensorSnapshot& sensors, float dt) = 0;

    /**
     * @brief Reset internal state (e.g. waypoint index, path buffers).
     */
    virtual void reset() = 0;
};

} // namespace px4_airsim_autonomy

