#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cstdint>

namespace px4_airsim_autonomy::production {

/**
 * @brief Volumetric boundary classification for 3D airspace partitioning.
 */
enum class GeofenceVolumeType : uint8_t {
    KeepIn = 0,  ///< Vehicle must remain strictly inside this operational volume
    KeepOut = 1  ///< Vehicle must remain strictly outside this exclusion/no-fly zone
};

/**
 * @brief 2D planar polygon defined by ordered vertices in the horizontal plane (local ENU [x, y]).
 */
struct Polygon2D {
    std::vector<Eigen::Vector2f> vertices;

    /**
     * @brief Validate that polygon has at least 3 non-degenerate vertices.
     */
    [[nodiscard]] bool isValid() const noexcept {
        return vertices.size() >= 3;
    }

    /**
     * @brief Compute signed 2D area (positive = CCW winding, negative = CW winding).
     */
    [[nodiscard]] float signedArea() const noexcept;

    /**
     * @brief True if polygon vertices are ordered counter-clockwise.
     */
    [[nodiscard]] bool isCounterClockwise() const noexcept {
        return signedArea() > 0.0f;
    }

    /**
     * @brief Point-in-polygon test using winding number algorithm.
     * @param pt 2D test point [x, y]
     * @return True if strictly inside or on boundary
     */
    [[nodiscard]] bool contains(const Eigen::Vector2f& pt) const noexcept;

    /**
     * @brief Calculate minimum Euclidean distance from point to polygon perimeter.
     * @param pt 2D query point
     * @param out_closest_pt Closest point on polygon boundary
     * @param out_normal Outward-pointing unit normal at closest point
     * @return Euclidean distance in meters
     */
    [[nodiscard]] float distanceToBoundary(
        const Eigen::Vector2f& pt,
        Eigen::Vector2f& out_closest_pt,
        Eigen::Vector2f& out_normal) const noexcept;

    /**
     * @brief 2D Ray-casting intersection against polygon edges.
     * @param origin Ray start point
     * @param dir Unit ray direction vector
     * @param out_dist Distance along ray to first intersection
     * @param out_normal Outward unit normal of intersected edge
     * @return True if ray intersects polygon boundary
     */
    [[nodiscard]] bool rayIntersect(
        const Eigen::Vector2f& origin,
        const Eigen::Vector2f& dir,
        float& out_dist,
        Eigen::Vector2f& out_normal) const noexcept;
};

/**
 * @brief 3D Volumetric prismatic airspace partition bounded by 2D polygon and altitude floor/ceiling.
 */
struct Volume3D {
    std::string id;                                     ///< Unique volume identifier
    GeofenceVolumeType type{GeofenceVolumeType::KeepIn};///< Keep-in or Keep-out classification
    Polygon2D boundary;                                 ///< 2D horizontal footprint
    float z_min{-10.0f};                                ///< Minimum allowable altitude in ENU (m)
    float z_max{100.0f};                                ///< Maximum allowable altitude in ENU (m)

    /**
     * @brief Check whether 3D position is inside this volume.
     */
    [[nodiscard]] bool contains(const Eigen::Vector3f& pt) const noexcept {
        if (pt.z() < z_min || pt.z() > z_max) {
            return false;
        }
        return boundary.contains(pt.head<2>());
    }

    /**
     * @brief Compute minimum 3D Euclidean distance to the volumetric boundary.
     * @param pt 3D query point in ENU
     * @param out_closest_pt Closest 3D point on the prism surface
     * @return Euclidean distance in meters
     */
    [[nodiscard]] float distanceToBoundary(
        const Eigen::Vector3f& pt,
        Eigen::Vector3f& out_closest_pt) const noexcept;

    /**
     * @brief 3D Ray intersection against the prism faces (vertical walls + top/bottom caps).
     * @param origin Ray origin [x, y, z]
     * @param dir Ray direction unit vector
     * @param out_dist Distance along ray to intersection
     * @param out_normal Outward normal of intersected face
     * @return True if ray intersects volume boundary
     */
    [[nodiscard]] bool rayIntersect(
        const Eigen::Vector3f& origin,
        const Eigen::Vector3f& dir,
        float& out_dist,
        Eigen::Vector3f& out_normal) const noexcept;
};

/**
 * @brief Dynamic braking and safety margin parameters.
 */
struct GeofenceParams {
    float t_reaction{0.30f};      ///< Actuator and telemetry latency reaction time (s)
    float a_max{2.50f};           ///< Maximum braking deceleration (m/s^2)
    float buffer_dist{3.0f};      ///< Minimum standoff distance buffer before boundary (m)
    float recovery_speed{1.5f};   ///< Maximum inward recovery velocity if boundary is breached (m/s)
};

/**
 * @brief Evaluation result detailing geofence compliance and evasive braking commands.
 */
struct GeofenceStatus {
    bool is_safe{true};                                            ///< True if strictly inside KeepIn, outside KeepOut, and stopping distance clear
    bool breach_imminent{false};                                   ///< True if predictive stopping distance violates boundary buffer
    bool breached{false};                                          ///< True if current vehicle position is currently violating geofence
    float dynamic_stopping_distance{0.0f};                         ///< Predicted stopping distance d_stop = v*t_react + v^2/(2*a_max)
    float min_distance_to_boundary{std::numeric_limits<float>::infinity()}; ///< Minimum clearance to nearest boundary in meters
    float trajectory_distance_to_boundary{std::numeric_limits<float>::infinity()}; ///< Distance to boundary along current velocity vector
    Eigen::Vector3f braking_velocity_cmd{Eigen::Vector3f::Zero()}; ///< Maximum deceleration velocity command to stop before boundary
    Eigen::Vector3f hold_point{Eigen::Vector3f::Zero()};           ///< Safe target position to hold station
    std::string active_volume_id;                                  ///< ID of critical volume
    std::string status_message;                                    ///< Diagnostic log string
};

/**
 * @class Geofence3D
 * @brief Production 3D volumetric geofence enforcement and predictive braking manager.
 *
 * Implements:
 * 1. 3D Keep-In and Keep-Out containment using Winding Number polygon algorithm + altitude bounds.
 * 2. Predictive dynamic stopping distance: d_stop = v * t_reaction + v^2 / (2 * a_max).
 * 3. Ray-casting boundary trajectory intersection.
 * 4. Maximum deceleration braking vector and hold point generation.
 */
class Geofence3D {
public:
    Geofence3D();
    explicit Geofence3D(const GeofenceParams& params);

    /**
     * @brief Add a 3D volumetric boundary (KeepIn or KeepOut).
     */
    void addVolume(const Volume3D& volume);

    /**
     * @brief Remove all registered volumes.
     */
    void clearVolumes() noexcept;

    /**
     * @brief Configure braking kinematics and safety margins.
     */
    void setParams(const GeofenceParams& params) noexcept;

    [[nodiscard]] const GeofenceParams& getParams() const noexcept { return params_; }
    [[nodiscard]] const std::vector<Volume3D>& getVolumes() const noexcept { return volumes_; }

    /**
     * @brief Calculate dynamic stopping distance based on current speed.
     *
     * Formula:
     *   d_stop = v * t_reaction + (v^2) / (2 * a_max)
     *
     * @param speed 3D speed magnitude (m/s)
     * @return Stopping distance in meters
     */
    [[nodiscard]] float computeStoppingDistance(float speed) const noexcept;

    /**
     * @brief Evaluate geofence compliance and generate braking commands if proximity alert triggers.
     *
     * @param position Current 3D position [x, y, z] in ENU local frame (m)
     * @param velocity Current 3D velocity [vx, vy, vz] in ENU local frame (m/s)
     * @param dt Time step since last update (s)
     * @return GeofenceStatus containing compliance state, braking commands, and safe hold point
     */
    [[nodiscard]] GeofenceStatus evaluate(
        const Eigen::Vector3f& position,
        const Eigen::Vector3f& velocity,
        float dt) const;

private:
    GeofenceParams params_;
    std::vector<Volume3D> volumes_;
};

} // namespace px4_airsim_autonomy::production
