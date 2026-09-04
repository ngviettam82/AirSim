#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <limits>
#include <string>

#include "px4_airsim_autonomy/types.hpp"

namespace px4_airsim_autonomy {
namespace production {

/**
 * @brief Configuration parameters for the dynamic stopping bubble ellipsoid.
 */
struct DynamicStoppingBubbleConfig {
    /// Maximum achievable emergency braking deceleration in m/s^2 (must be > 0).
    float a_max = 4.0f;

    /// Total system latency in seconds (sensing pipeline + transport + actuator response).
    float t_latency = 0.15f;

    /// Static safety margin / drone physical footprint buffer in meters.
    float d_margin = 0.8f;

    /// Lateral semi-axis base clearance in meters (transverse to velocity vector).
    float r_lateral = 0.8f;

    /// Vertical semi-axis base clearance in meters (perpendicular to lateral and velocity).
    float r_vertical = 0.6f;

    /// Velocity expansion scaling factor for lateral semi-axis (seconds).
    float k_lateral_vel = 0.05f;

    /// Velocity expansion scaling factor for vertical semi-axis (seconds).
    float k_vertical_vel = 0.02f;

    /// Velocity magnitude threshold below which vehicle is considered stationary (m/s).
    float min_velocity_threshold = 0.05f;
};

/**
 * @brief Detailed diagnostic output for obstacle penetration evaluations.
 */
struct BubbleViolation {
    /// True if an obstacle penetrates inside the dynamic stopping bubble ellipsoid.
    bool is_violated = false;

    /// Minimum normalized ellipsoid distance (d_norm < 1.0 indicates penetration).
    float min_normalized_distance = std::numeric_limits<float>::infinity();

    /// Estimated penetration depth in meters (> 0 when violated).
    float max_penetration_depth = 0.0f;

    /// World-frame coordinates of the most critical / penetrating obstacle point.
    Eigen::Vector3f critical_obstacle_point = Eigen::Vector3f::Zero();

    /// Vector from current vehicle position to the critical obstacle.
    Eigen::Vector3f relative_obstacle_vector = Eigen::Vector3f::Zero();

    /// Euclidean distance from vehicle to critical obstacle (meters).
    float obstacle_distance = std::numeric_limits<float>::infinity();

    /// Estimated time to impact along current velocity vector (seconds, infinity if opening).
    float time_to_impact = std::numeric_limits<float>::infinity();

    /// Diagnostic description.
    std::string description;
};

/**
 * @brief Dynamic Stopping Bubble (E_stop) for high-speed multirotor collision avoidance.
 *
 * Implements a velocity-aligned 3D ellipsoid representation centered ahead of the vehicle:
 *   d_stop(v) = ||v||^2 / (2 * a_max) + ||v|| * t_latency + d_margin
 *
 * The ellipsoid extends along the instantaneous velocity vector v_hat.
 * Obstacles are checked against the algebraic quadric form:
 *   (x - p_c)^T M (x - p_c) <= 1
 * where M = R * diag(1/a^2, 1/b^2, 1/c^2) * R^T.
 *
 * Provides emergency braking vector generation:
 *   a_brake = -a_max * (v / ||v||)
 */
class DynamicStoppingBubble {
public:
    /**
     * @brief Construct a new Dynamic Stopping Bubble object.
     * @param config Configuration parameters.
     */
    explicit DynamicStoppingBubble(const DynamicStoppingBubbleConfig& config = DynamicStoppingBubbleConfig());

    ~DynamicStoppingBubble() = default;

    /**
     * @brief Update the internal ellipsoid state given current vehicle kinematics.
     * @param vehicle_position Current position in world / ENU frame [x, y, z] (m).
     * @param vehicle_velocity Current velocity vector in world / ENU frame [vx, vy, vz] (m/s).
     */
    void update(const Eigen::Vector3f& vehicle_position, const Eigen::Vector3f& vehicle_velocity);

    /**
     * @brief Calculate the scalar quadratic stopping distance for a given speed.
     * d_stop(v) = v^2 / (2 * a_max) + v * t_latency + d_margin
     * @param speed Speed magnitude in m/s.
     * @return float Required stopping distance in meters.
     */
    float computeStoppingDistance(float speed) const;

    /**
     * @brief Calculate the scalar stopping distance for a 3D velocity vector.
     * @param velocity Velocity vector in m/s.
     * @return float Required stopping distance in meters.
     */
    float computeStoppingDistance(const Eigen::Vector3f& velocity) const;

    /**
     * @brief Calculate total time required to bring vehicle to a complete stop.
     * t_stop = ||v|| / a_max + t_latency
     * @param speed Speed magnitude in m/s.
     * @return float Time in seconds.
     */
    float computeTimeToStop(float speed) const;

    /**
     * @brief Compute the instantaneous emergency deceleration braking vector.
     * a_brake = -a_max * (v / ||v||)
     * @return Eigen::Vector3f Braking acceleration vector in m/s^2 (world frame).
     */
    Eigen::Vector3f computeBrakingAcceleration() const;

    /**
     * @brief Compute emergency deceleration braking vector for an arbitrary velocity.
     * @param velocity Velocity vector in m/s.
     * @return Eigen::Vector3f Braking acceleration vector in m/s^2.
     */
    Eigen::Vector3f computeBrakingAcceleration(const Eigen::Vector3f& velocity) const;

    /**
     * @brief Compute next velocity setpoint during active emergency braking over step dt.
     * Integrates a_brake and clamps when velocity reaches zero to prevent backwards motion.
     * @param dt Integration time step in seconds.
     * @return Eigen::Vector3f Decelerated velocity setpoint (m/s).
     */
    Eigen::Vector3f computeBrakingVelocityStep(float dt) const;

    /**
     * @brief Check if a single 3D world-frame point lies inside the stopping bubble.
     * @param point 3D coordinates [x, y, z] in world frame.
     * @return true if point is inside or on the ellipsoid boundary.
     */
    bool isPointInside(const Eigen::Vector3f& point) const;

    /**
     * @brief Compute normalized algebraic distance from ellipsoid center to point.
     * sqrt((x - p_c)^T M (x - p_c)). Value < 1.0 indicates inside.
     * @param point 3D coordinates in world frame.
     * @return float Normalized distance.
     */
    float computeNormalizedDistance(const Eigen::Vector3f& point) const;

    /**
     * @brief Check an entire point cloud against the dynamic stopping bubble.
     * @param obstacle_points List of 3D obstacle points in world frame.
     * @return BubbleViolation Detailed violation diagnostic.
     */
    BubbleViolation checkObstacles(const std::vector<Eigen::Vector3f>& obstacle_points) const;

    /**
     * @brief Check obstacle violation against depth camera sectors from SensorSnapshot.
     * Evaluates forward, left, right, upper, lower clearances against the dynamic bubble.
     * @param snapshot Current sensor snapshot containing depth metrics.
     * @return BubbleViolation Detailed violation diagnostic.
     */
    BubbleViolation checkSensorSnapshot(const SensorSnapshot& snapshot) const;

    /**
     * @brief Check if a bounding sphere obstacle collides with the stopping bubble.
     * Uses conservative Minkowski inflation of the ellipsoid semi-axes.
     * @param sphere_center World coordinates of sphere center.
     * @param sphere_radius Obstacle radius in meters.
     * @return true if sphere intersects or penetrates the bubble.
     */
    bool isSphereColliding(const Eigen::Vector3f& sphere_center, float sphere_radius) const;

    // Accessors
    const DynamicStoppingBubbleConfig& getConfig() const { return config_; }
    void setConfig(const DynamicStoppingBubbleConfig& config) { config_ = config; }

    const Eigen::Vector3f& getVehiclePosition() const { return vehicle_position_; }
    const Eigen::Vector3f& getVehicleVelocity() const { return vehicle_velocity_; }
    float getCurrentSpeed() const { return current_speed_; }

    const Eigen::Vector3f& getCenter() const { return center_; }
    const Eigen::Matrix3f& getOrientation() const { return orientation_; }
    const Eigen::Vector3f& getSemiAxes() const { return semi_axes_; }
    const Eigen::Matrix3f& getShapeMatrix() const { return shape_matrix_m_; }

private:
    /**
     * @brief Construct an orthonormal rotation matrix with local X aligned with unit_v.
     * Uses robust Gram-Schmidt with singularity handling.
     * @param unit_v Normalized direction vector.
     * @return Eigen::Matrix3f Orthonormal matrix [u_x, u_y, u_z].
     */
    static Eigen::Matrix3f buildOrthonormalBasis(const Eigen::Vector3f& unit_v);

    DynamicStoppingBubbleConfig config_;

    Eigen::Vector3f vehicle_position_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f vehicle_velocity_ = Eigen::Vector3f::Zero();
    float current_speed_ = 0.0f;

    // Geometric parameters of the 3D ellipsoid E_stop
    Eigen::Vector3f center_ = Eigen::Vector3f::Zero();
    Eigen::Matrix3f orientation_ = Eigen::Matrix3f::Identity();
    Eigen::Vector3f semi_axes_ = Eigen::Vector3f::Constant(1.0f); // [a (longitudinal), b (lateral), c (vertical)]
    Eigen::Matrix3f shape_matrix_m_ = Eigen::Matrix3f::Identity(); // M = R * diag(a^-2, b^-2, c^-2) * R^T
};

} // namespace production
} // namespace px4_airsim_autonomy
