#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <memory>
#include <cstdint>

namespace px4_airsim_autonomy {

enum class ControlType {
    Velocity,
    Position
};

struct AutonomyCommand {
    ControlType type = ControlType::Velocity;
    
    // Command vector:
    // When type == Velocity: [vx, vy, vz] in m/s (FLU body frame or ENU frame as specified)
    // When type == Position: [x, y, z] in meters (ENU local frame)
    Eigen::Vector3f vector = Eigen::Vector3f::Zero();
    
    // Yaw command:
    // When type == Velocity: yaw rate in rad/s
    // When type == Position: absolute yaw angle in radians
    float yaw_or_yaw_rate = 0.0f;
    
    // Diagnostic / log message describing the current action of the algorithm
    std::string status_message;
};

struct SensorSnapshot {
    // Current vehicle odometry / state
    Eigen::Vector3f position = Eigen::Vector3f::Zero(); // [x, y, z] in local ENU frame (meters)
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero(); // [vx, vy, vz] in local ENU frame (m/s)
    float yaw = 0.0f;                                   // Vehicle heading in ENU frame (rad, 0=East, pi/2=North, CCW+)
    float yaw_ned = 0.0f;                               // Vehicle heading in NED frame (rad, 0=North, pi/2=East, CW+)

    // Depth camera summary metrics from AirSim (zero-copy evaluated)
    int depth_width = 0;
    int depth_height = 0;
    float min_depth_distance = 100.0f;                  // Minimum measured obstacle distance in meters
    float center_sector_distance = 100.0f;              // Forward clearance in central band (m)
    float left_sector_distance = 100.0f;                // Left sector clearance (m)
    float right_sector_distance = 100.0f;               // Right sector clearance (m)
    float upper_sector_distance = 100.0f;               // Overhead / ceiling clearance (m)
    float lower_sector_distance = 100.0f;               // Ground / lower clearance (m)

    // Full 3x3 directional sector depth grid:
    // Row 0: Upper [Left, Center, Right]
    // Row 1: Mid   [Left, Center, Right]
    // Row 2: Lower [Left, Center, Right]
    float sectors_3d[3][3] = {
        {100.0f, 100.0f, 100.0f},
        {100.0f, 100.0f, 100.0f},
        {100.0f, 100.0f, 100.0f}
    };

    // Target detection / finding (e.g., from visual detector or bounding box)
    bool target_detected = false;
    Eigen::Vector3f target_position = Eigen::Vector3f::Zero(); // 3D relative position of target in Body FLU (m)
    float target_confidence = 0.0f;

    // Simulation / loop timing
    float dt = 0.05f; // Seconds elapsed since previous iteration
    uint64_t timestamp_us = 0;
};

} // namespace px4_airsim_autonomy

