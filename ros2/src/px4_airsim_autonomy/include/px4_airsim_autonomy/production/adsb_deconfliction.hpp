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
 * @brief DO-365B Alerting Rungs for Detect and Avoid (DAA) airspace deconfliction.
 */
enum class DaaAlertLevel : uint8_t {
    Level0_Normal = 0,             ///< Level 0: Normal / No Alert (traffic clear)
    Level1_TrafficAdvisory = 1,    ///< Level 1: Traffic Advisory (TA) - Pilot/operator situational awareness
    Level2_Preventive = 2,         ///< Level 2: Preventive Advisory - Prevent maneuvers that degrade separation
    Level3_Corrective = 3,         ///< Level 3: Corrective Advisory - Timely avoidance maneuver recommended
    Level4_Warning = 4             ///< Level 4: Warning Alert / Well Clear Violation (WCV) - Immediate evasive maneuver
};

/**
 * @brief State telemetry for ownship UAS in local ENU frame.
 */
struct OwnshipState {
    Eigen::Vector3f position{Eigen::Vector3f::Zero()}; ///< [x, y, z] in local ENU meters
    Eigen::Vector3f velocity{Eigen::Vector3f::Zero()}; ///< [vx, vy, vz] in local ENU m/s
    float heading_rad{0.0f};                           ///< Heading in radians (0=East, pi/2=North)
};

/**
 * @brief Telemetry report of an ADS-B / transponder-equipped intruder aircraft.
 */
struct AdsbTarget {
    uint32_t icao_address{0};                          ///< 24-bit ICAO aircraft address
    std::string callsign{"UNKNOWN"};                   ///< Aircraft flight ID / callsign
    Eigen::Vector3f position{Eigen::Vector3f::Zero()}; ///< [x, y, z] in local ENU meters
    Eigen::Vector3f velocity{Eigen::Vector3f::Zero()}; ///< [vx, vy, vz] in local ENU m/s
    uint64_t timestamp_us{0};                          ///< Report timestamp in microseconds
    bool is_valid{true};                               ///< Telemetry validity flag
};

/**
 * @brief DO-365B DAA Well Clear (DWC) threshold parameters per alert rung.
 */
struct DaaRungThresholds {
    float tau_mod_sec{15.0f};   ///< Modified Tau threshold (s)
    float d_mod_m{450.0f};      ///< Distance modification threshold D_MOD (m)
    float hmd_m{450.0f};        ///< Horizontal Miss Distance threshold HMD (m)
    float z_thr_m{100.0f};      ///< Vertical separation boundary threshold Z_thr (m)
    float t_v_sec{15.0f};       ///< Time to co-altitude threshold t_V (s)
};

/**
 * @brief Full suite of DO-365B thresholds across all alerting rungs.
 */
struct DaaConfig {
    DaaRungThresholds warning{15.0f, 450.0f, 450.0f, 100.0f, 15.0f};     ///< Level 4: Warning Alert / WCV
    DaaRungThresholds corrective{30.0f, 600.0f, 600.0f, 120.0f, 25.0f};  ///< Level 3: Corrective Advisory
    DaaRungThresholds preventive{55.0f, 800.0f, 800.0f, 150.0f, 40.0f};  ///< Level 2: Preventive Advisory
    DaaRungThresholds advisory{75.0f, 1000.0f, 1000.0f, 200.0f, 55.0f};   ///< Level 1: Traffic Advisory

    float emergency_dive_rate{-4.0f};   ///< Commanded vertical dive rate for Level 4 warning (m/s)
    float corrective_dive_rate{-2.0f};  ///< Commanded vertical descent rate for Level 3 corrective (m/s)
    float emergency_climb_rate{3.0f};   ///< Alternative emergency climb rate if ground proximity prevents dive (m/s)
    float min_safe_agl_for_dive{20.0f}; ///< Minimum AGL altitude (m) required to permit emergency dive
    float evasive_cruise_speed{8.0f};   ///< Commanded lateral evasive speed magnitude (m/s)
};

/**
 * @brief Intruder pairwise DAA metrics evaluated under DO-365B equations.
 */
struct IntruderMetrics {
    uint32_t icao_address{0};
    std::string callsign;
    DaaAlertLevel alert_level{DaaAlertLevel::Level0_Normal};

    float horizontal_range{0.0f};                                     ///< Current horizontal separation r_horiz = ||r_h|| (m)
    float vertical_separation{0.0f};                                  ///< Current vertical separation |dz| (m)
    float range_rate{0.0f};                                           ///< Closing speed: - (r_h . v_rel,h) / r_horiz (m/s, >0 converging)
    float tau_mod{std::numeric_limits<float>::infinity()};            ///< Modified Tau: - (r_horiz^2 - D_MOD^2) / (r_h . v_rel,h) (s)
    float hmd{0.0f};                                                  ///< Horizontal Miss Distance at CPA (m)
    float time_to_cpa{std::numeric_limits<float>::infinity()};        ///< Time to closest point of approach t_cpa (s)
    float time_to_coaltitude{std::numeric_limits<float>::infinity()}; ///< Time to co-altitude t_coa (s)
    bool is_converging_horiz{false};                                  ///< True if relative horizontal distance is decreasing
    bool is_converging_vert{false};                                   ///< True if relative altitude is closing
};

/**
 * @brief Consolidated airspace deconfliction assessment across all tracked traffic.
 */
struct DeconflictionResult {
    DaaAlertLevel max_alert_level{DaaAlertLevel::Level0_Normal};     ///< Highest alert level triggered among all intruders
    bool evasive_action_required{false};                              ///< True if Level 3 Corrective or Level 4 Warning
    uint32_t primary_threat_icao{0};                                  ///< ICAO of most critical threat
    std::string primary_threat_callsign;                              ///< Callsign of most critical threat
    IntruderMetrics primary_threat_metrics;                           ///< Detailed metrics of most critical threat
    Eigen::Vector3f recommended_velocity_cmd{Eigen::Vector3f::Zero()};///< Evasive velocity command in local ENU (m/s)
    std::vector<IntruderMetrics> all_intruder_metrics;                ///< Metrics for all tracked traffic
    std::string advisory_message;                                     ///< Human-readable diagnostic description
};

/**
 * @class AdsbDeconfliction
 * @brief Production RTCA DO-365B compliant Detect and Avoid (DAA) airspace deconfliction system.
 *
 * Implements:
 * 1. 5-tier DO-365B alerting rungs: Level 0 (Normal) to Level 4 (Warning / Well Clear Violation).
 * 2. Modified Tau: tau_mod = - (r_horiz^2 - D_MOD^2) / (r_horiz . v_rel,horiz).
 * 3. Exact Horizontal Miss Distance (HMD) at CPA and vertical co-altitude timing.
 * 4. Multi-threat arbitration and evasive emergency commands (e.g. -4.0 m/s dive rate).
 */
class AdsbDeconfliction {
public:
    AdsbDeconfliction();
    explicit AdsbDeconfliction(const DaaConfig& config);

    /**
     * @brief Configure DAA alert thresholds and avoidance parameters.
     */
    void setConfig(const DaaConfig& config) noexcept;

    [[nodiscard]] const DaaConfig& getConfig() const noexcept { return config_; }

    /**
     * @brief Evaluate pairwise DO-365B metrics and determine alert rung for a single intruder.
     *
     * @param ownship Ownship UAS state
     * @param intruder ADS-B intruder state
     * @return IntruderMetrics containing tau_mod, HMD, co-altitude time, and alert rung
     */
    [[nodiscard]] IntruderMetrics evaluateIntruder(
        const OwnshipState& ownship,
        const AdsbTarget& intruder) const noexcept;

    /**
     * @brief Evaluate all active ADS-B traffic and generate recommended evasive commands.
     *
     * @param ownship Ownship UAS state
     * @param traffic Vector of tracked ADS-B intruders
     * @return Consolidated DeconflictionResult with highest alert level and evasive velocity
     */
    [[nodiscard]] DeconflictionResult evaluateTraffic(
        const OwnshipState& ownship,
        const std::vector<AdsbTarget>& traffic) const;

    /**
     * @brief Generate recommended 3D evasive velocity vector for a specific threat intruder.
     *
     * @param ownship Ownship UAS state
     * @param threat Critical intruder metrics and telemetry
     * @return Commanded evasive velocity in local ENU frame [vx, vy, vz] (m/s)
     */
    [[nodiscard]] Eigen::Vector3f computeEvasiveCommand(
        const OwnshipState& ownship,
        const AdsbTarget& threat,
        const IntruderMetrics& metrics) const noexcept;

    /**
     * @brief Convert DAA alert level enumeration to string.
     */
    [[nodiscard]] static std::string alertLevelToString(DaaAlertLevel level) noexcept;

private:
    DaaConfig config_;

    /**
     * @brief Check if pairwise geometry violates thresholds for a specific DAA rung.
     */
    [[nodiscard]] bool checkRungViolation(
        float r_horiz,
        float tau_mod,
        float hmd,
        float dz,
        float t_coa,
        bool is_conv_horiz,
        bool is_conv_vert,
        const DaaRungThresholds& rung) const noexcept;
};

} // namespace px4_airsim_autonomy::production
