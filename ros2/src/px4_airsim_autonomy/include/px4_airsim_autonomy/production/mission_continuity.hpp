#pragma once

#include "px4_airsim_autonomy/production/boustrophedon_planner.hpp"
#include <Eigen/Core>
#include <string>
#include <vector>
#include <optional>
#include <cstdint>

namespace px4_airsim_autonomy::production {

/**
 * @brief Persistent state snapshot captured when an autonomous survey mission is paused or aborted.
 */
struct BreakpointState {
    std::string mission_id{"mission_default"};  ///< Unique identifier of the mission
    size_t current_waypoint_index{0};           ///< Index of the waypoint being executed at abort
    int strip_index{0};                         ///< Survey strip index active at abort
    Eigen::Vector3d abort_position{Eigen::Vector3d::Zero()}; ///< [x, y, z] local coordinates at abort
    double progress_ratio{0.0};                 ///< Completed fraction of mission [0.0, 1.0]
    double heading_rad{0.0};                    ///< Vehicle heading at abort in radians
    double flight_speed_m_s{0.0};               ///< Vehicle speed at abort in m/s
    double battery_percentage{0.0};             ///< Remaining battery SoC percentage [0, 100]
    int64_t timestamp_utc_ms{0};                ///< Abort timestamp in milliseconds UTC
    std::string abort_reason{"manual_pause"};   ///< Triggering reason (e.g. "low_battery_rtl", "weather_pause")

    /**
     * @brief Serialize BreakpointState into a standard JSON string.
     */
    std::string toJson() const;

    /**
     * @brief Deserialize BreakpointState from JSON string.
     * @param json_str Standard JSON string representation.
     * @return Parsed BreakpointState or std::nullopt if malformed.
     */
    static std::optional<BreakpointState> fromJson(const std::string& json_str);

    /**
     * @brief Atomically save BreakpointState to disk.
     */
    bool saveToFile(const std::string& filepath) const;

    /**
     * @brief Load BreakpointState from file.
     */
    static std::optional<BreakpointState> loadFromFile(const std::string& filepath);
};

/**
 * @brief Parameters for calculating the dynamic photogrammetric backtrack resumption distance.
 */
struct BacktrackParameters {
    double trigger_distance_m{10.0};     ///< Along-track photogrammetric trigger interval D_trigger in meters
    double nominal_flight_speed_m_s{5.0}; ///< Target survey speed v in m/s
    double max_acceleration_m_s2{2.0};    ///< Maximum acceleration/deceleration a in m/s^2
    double settling_time_s{1.5};         ///< Gimbal and flight controller stabilization time t_settle in seconds
    double min_backtrack_buffer_m{3.0};   ///< Absolute minimum safety floor for backtrack distance

    /**
     * @brief Calculate dynamic backtrack distance D_backtrack.
     * Formula:
     *   D_backtrack = max(2 * D_trigger, (v^2 / (2 * a)) + v * t_settle)
     */
    double computeBacktrackDistance() const;
};

/**
 * @brief Ingress transit and approach geometry configuration for mission resumption.
 */
struct IngressConfig {
    double safe_transit_altitude_agl_m{60.0}; ///< Elevated altitude during transit to clear terrain/obstacles
    double alignment_lead_distance_m{12.0};   ///< Distance along track before resumption point to establish steady flight
    double transit_speed_m_s{8.0};            ///< Fast cruise speed during ingress transit
    double approach_speed_m_s{5.0};           ///< Nominal survey entry speed
};

/**
 * @brief Complete resumption trajectory plan synthesized from a mission breakpoint.
 */
struct ResumptionPlan {
    BreakpointState breakpoint;                   ///< The original breakpoint state
    double applied_backtrack_distance_m{0.0};     ///< Actual distance backtracked along mission path
    Eigen::Vector3d reengagement_point{Eigen::Vector3d::Zero()}; ///< The exact point where survey resumes
    size_t resumed_waypoint_index{0};             ///< Waypoint index in original plan corresponding to resumption
    std::vector<SweepWaypoint> resumed_waypoints; ///< Full synthesized sequential trajectory for resumed mission
    double total_ingress_distance_m{0.0};         ///< Distance flown from launch/current pos to re-engagement
    double remaining_survey_distance_m{0.0};      ///< Remaining active photogrammetry distance
};

/**
 * @brief Mission continuity and dynamic resumption manager for aerial survey operations.
 */
class MissionContinuity {
public:
    MissionContinuity() = delete;

    /**
     * @brief Compute the dynamic photogrammetric backtrack buffer distance.
     * Formula:
     *   D_backtrack = max(2 * D_trigger, (v^2 / (2 * a)) + v * t_settle)
     * - 2 * D_trigger guarantees at least two overlapping photo frames with the prior sortie,
     *   ensuring seamless Structure-from-Motion (SfM) bundle adjustment tie points across the seam.
     * - (v^2 / (2*a)) + v * t_settle guarantees the drone accelerates to steady cruise velocity
     *   and gimbal oscillations settle before crossing the abort seam.
     * @param params Kinematic and photogrammetric parameters.
     * @return Dynamic backtrack distance in meters.
     */
    static double computeBacktrackDistance(const BacktrackParameters& params);

    /**
     * @brief Synthesize smooth re-engagement trajectory starting D_backtrack meters prior to abort point.
     * Generates safe vertical clearance ingress transit, linear track alignment corridor,
     * and seamlessly spliced survey waypoints with photographic triggering enabled at re-engagement.
     *
     * @param original_mission Full waypoint list of the original survey mission.
     * @param breakpoint Captured breakpoint state from the interrupted flight.
     * @param drone_current_position Current 3D position of the drone (e.g. takeoff pad or hover).
     * @param backtrack_params Parameters to compute dynamic backtrack distance.
     * @param ingress_config Geometric and speed parameters for ingress transit corridor.
     * @return ResumptionPlan Complete synthesized trajectory ready for autonomous execution.
     */
    static ResumptionPlan synthesizeResumptionTrajectory(
        const std::vector<SweepWaypoint>& original_mission,
        const BreakpointState& breakpoint,
        const Eigen::Vector3d& drone_current_position,
        const BacktrackParameters& backtrack_params,
        const IngressConfig& ingress_config);
};

} // namespace px4_airsim_autonomy::production
