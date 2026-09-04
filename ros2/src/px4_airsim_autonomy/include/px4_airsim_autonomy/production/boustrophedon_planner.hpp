#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <memory>
#include <optional>
#include <cmath>

namespace px4_airsim_autonomy::production {

/**
 * @brief Waypoint classification in photogrammetric survey missions.
 */
enum class WaypointType {
    SurveyEntry = 0,     ///< Ingress waypoint approaching survey boundary
    SweepStart = 1,      ///< Start of active linear sweep (photo capture starts)
    SweepEnd = 2,        ///< End of active linear sweep (photo capture stops)
    TurnOvershoot = 3,   ///< Lead-out waypoint parallel to sweep orientation
    TurnEntry = 4,       ///< Lead-in waypoint aligned with subsequent sweep line
    SurveyExit = 5       ///< Final waypoint exiting surveyed polygon
};

/**
 * @brief Turn execution style between adjacent boustrophedon strips.
 */
enum class TurnStyle {
    SquareOvershoot = 0, ///< Lead-out parallel to strip, lateral shift, lead-in parallel to strip
    Direct = 1           ///< Direct straight-line diagonal transition between strips
};

/**
 * @brief Autonomous survey waypoint with photogrammetry triggering metadata.
 */
struct SweepWaypoint {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()}; ///< [x, y, z] in local coordinate frame (meters)
    double yaw_rad{0.0};                               ///< Desired vehicle heading in radians
    double target_speed_m_s{5.0};                      ///< Desired vehicle speed in m/s
    WaypointType type{WaypointType::SweepStart};       ///< Waypoint functional role
    int strip_index{0};                                ///< 0-indexed strip number
    bool is_photo_trigger_zone{false};                 ///< True if camera trigger should be active
    size_t global_index{0};                            ///< Sequential index in mission plan

    std::string typeString() const;
};

/**
 * @brief Result of Rotating Calipers orientation optimization.
 */
struct RotatingCalipersResult {
    double optimal_angle_rad{0.0};        ///< Optimal sweep angle θ* in radians [-pi, pi]
    double optimal_angle_deg{0.0};        ///< Optimal sweep angle θ* in degrees [0, 180)
    double projected_width_m{0.0};        ///< Projected cross-track width W_proj(θ*) in meters
    int strip_count{0};                   ///< Minimum number of parallel strips ceil(W_proj / S_strip)
    double edge_length_m{0.0};            ///< Length of edge establishing caliper orientation
    std::vector<Eigen::Vector2d> convex_hull; ///< 2D convex hull of survey polygon
};

/**
 * @brief Configuration parameters for Boustrophedon path generation.
 */
struct PlannerConfig {
    double strip_spacing_m{20.0};         ///< Cross-track strip spacing S_strip in meters
    double flight_altitude_agl_m{50.0};   ///< Survey altitude AGL in meters
    double survey_speed_m_s{5.0};         ///< Cruise speed along linear sweeps in m/s
    double turn_speed_m_s{2.5};           ///< Speed during transition turns in m/s
    TurnStyle turn_style{TurnStyle::SquareOvershoot}; ///< Turn geometry type
    double turn_overshoot_m{6.0};         ///< Lead-out/lead-in distance outside boundary parallel to strip
    double boundary_margin_m{0.0};        ///< Offset margin applied to polygon (+ expand, - inset)
    bool force_custom_angle{false};       ///< If true, override rotating calipers with custom_angle_rad
    double custom_angle_rad{0.0};         ///< User-defined sweep angle in radians
    double angular_resolution_deg{1.0};   ///< Angular search resolution for rotating calipers in degrees

    bool isValid() const;
};

/**
 * @brief Summary statistics of planned boustrophedon mission.
 */
struct MissionSummary {
    RotatingCalipersResult orientation;
    size_t total_waypoints{0};
    int total_strips{0};
    double total_flight_distance_m{0.0};
    double active_survey_distance_m{0.0};
    double turn_distance_m{0.0};
    double estimated_flight_time_s{0.0};
    double covered_area_m2{0.0};
};

/**
 * @brief Production-grade 2D/3D Boustrophedon (lawnmower) path planner.
 * Utilizes Rotating Calipers algorithm to find minimum-turn sweep orientation θ*
 * that minimizes total strip count ceil(W_proj(θ) / S_strip), and synthesizes
 * alternating sweep waypoints with turning segments parallel to the sweep tracks.
 */
class BoustrophedonPlanner {
public:
    BoustrophedonPlanner() = delete;

    /**
     * @brief Compute 2D convex hull of arbitrary 2D polygon vertices using Andrew's Monotone Chain algorithm.
     * Guaranteed O(N log N) time complexity, robust against collinear and duplicate vertices.
     * @param vertices Input 2D points in arbitrary order.
     * @return std::vector<Eigen::Vector2d> Counter-clockwise convex hull vertices.
     */
    static std::vector<Eigen::Vector2d> computeConvexHull(const std::vector<Eigen::Vector2d>& vertices);

    /**
     * @brief Compute projected cross-track width of a point set perpendicular to sweep angle θ.
     * Perpendicular normal n = (-sin θ, cos θ).
     * Formula:
     *   W_proj(θ) = max(v · n) - min(v · n)
     * @param vertices Polygon or convex hull vertices.
     * @param sweep_angle_rad Sweep orientation angle θ in radians.
     * @return Cross-track width in meters.
     */
    static double computeProjectedWidth(const std::vector<Eigen::Vector2d>& vertices, double sweep_angle_rad);

    /**
     * @brief Execute Rotating Calipers search to find optimal orientation θ* that minimizes strip count.
     * Explores all convex hull edge orientations (Freeman-Shapira theorem) and fine angular discretization.
     * Minimizes: ceil(W_proj(θ) / S_strip).
     * Tie-breaker: Minimizes W_proj(θ) and maximizes edge length for longest straight runs.
     * @param polygon_vertices 2D boundary polygon vertices.
     * @param strip_spacing_m Side strip spacing S_strip in meters.
     * @param angular_step_deg Discretization step in degrees (default 1.0°).
     * @return RotatingCalipersResult Contains optimal angle θ*, minimum strip count, and convex hull.
     */
    static RotatingCalipersResult findOptimalOrientation(const std::vector<Eigen::Vector2d>& polygon_vertices,
                                                        double strip_spacing_m,
                                                        double angular_step_deg = 1.0);

    /**
     * @brief Compute intersection segments between a horizontal sweep line y' = y_val and a 2D polygon.
     * @param rotated_polygon Polygon vertices transformed into sweep-aligned coordinate frame.
     * @param y_line Y-coordinate of sweep line in aligned frame.
     * @return std::vector<std::pair<double, double>> Disjoint [x_start, x_end] intervals inside polygon.
     */
    static std::vector<std::pair<double, double>> intersectSweepLine(const std::vector<Eigen::Vector2d>& rotated_polygon,
                                                                     double y_line);

    /**
     * @brief Plan complete boustrophedon survey mission for 2D polygon.
     * @param polygon_vertices Boundary vertices in local Cartesian frame (ENU/NED).
     * @param config Planner configuration (spacing, altitude, speeds, turn style).
     * @return std::vector<SweepWaypoint> Sequential mission waypoints with trigger metadata.
     */
    static std::vector<SweepWaypoint> planMission(const std::vector<Eigen::Vector2d>& polygon_vertices,
                                                  const PlannerConfig& config);

    /**
     * @brief Plan complete boustrophedon survey mission for 3D polygon vertices.
     * @param polygon_vertices_3d Boundary vertices [x, y, z]. Altitude can be overridden by config.
     * @param config Planner configuration.
     * @return std::vector<SweepWaypoint> Sequential mission waypoints.
     */
    static std::vector<SweepWaypoint> planMission3D(const std::vector<Eigen::Vector3d>& polygon_vertices_3d,
                                                    const PlannerConfig& config);

    /**
     * @brief Calculate comprehensive mission summary metrics (distance, time, strips, area).
     * @param waypoints Generated mission waypoints.
     * @param polygon_vertices Input boundary polygon.
     * @param result Calipers optimization result.
     * @return MissionSummary Detailed statistics.
     */
    static MissionSummary computeMissionSummary(const std::vector<SweepWaypoint>& waypoints,
                                                const std::vector<Eigen::Vector2d>& polygon_vertices,
                                                const RotatingCalipersResult& result);

    /**
     * @brief Compute polygon area using Shoelace formula.
     */
    static double computePolygonArea(const std::vector<Eigen::Vector2d>& polygon);

    /**
     * @brief Offset/buffer a 2D polygon by margin distance (+ expands, - insets).
     */
    static std::vector<Eigen::Vector2d> offsetPolygon(const std::vector<Eigen::Vector2d>& polygon,
                                                      double margin_m);
};

} // namespace px4_airsim_autonomy::production
