#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cstdint>

namespace px4_airsim_autonomy::production {

/**
 * @brief Discrete operational safety states for Smart Return-To-Home (RTH) battery contingency.
 */
enum class RthBatteryState : uint8_t {
    Nominal = 0,             ///< Sufficient battery capacity with safe operational margin
    AdvisoryWarning = 1,     ///< Battery remaining energy approaching threshold, operator advisory
    CriticalRthTrigger = 2,  ///< Available energy matches required return energy (+ reserve); immediate RTH trigger required
    EmergencyCutoffLand = 3  ///< Voltage under critical load threshold or critical SoC depleted; immediate landing on spot required
};

/**
 * @brief Physical and electrochemical parameters of the multirotor battery pack.
 */
struct BatteryConfig {
    float nominal_voltage{22.2f};           ///< Pack nominal voltage (V), e.g., 6S = 22.2 V
    int cells_in_series{6};                 ///< Number of series cells (e.g., 6 for 6S)
    float capacity_mah{5000.0f};            ///< Nameplate pack capacity in milliamp-hours (mAh)
    float internal_resistance_ohm{0.015f};  ///< Lumped internal pack resistance R_int in Ohms (e.g., 2.5 mOhm/cell * 6)
    float crit_cell_voltage{3.30f};         ///< Cell voltage cutoff requiring immediate emergency land (V/cell)
    float warn_cell_voltage{3.50f};         ///< Cell voltage threshold triggering mandatory RTH (V/cell)
    float emergency_cutoff_soc{0.05f};      ///< Minimum allowable State-of-Charge (SoC, 0.0 - 1.0) before cutoff
    float advisory_soc_margin{0.08f};       ///< SoC margin above critical RTH for operator advisory warning
    float reserve_soc_fraction{0.10f};      ///< Uncompromised energy reserve fraction (e.g., 10% reserve capacity)
    float min_reserve_time_sec{120.0f};     ///< Minimum duration of hover reserve energy in seconds (e.g., 2 min)

    /**
     * @brief Compute total usable battery energy capacity in Joules (W*s).
     */
    [[nodiscard]] inline float totalCapacityJoules() const noexcept {
        // E = V * I * t = V * (capacity_mah / 1000) * 3600 J
        return nominal_voltage * (capacity_mah * 0.001f) * 3600.0f;
    }
};

/**
 * @brief Aerodynamic power consumption and flight kinematics configuration.
 */
struct VehiclePowerConfig {
    float p_hover{250.0f};           ///< Power consumed during steady hover (Watts)
    float p_cruise{280.0f};          ///< Power consumed during cruise airspeed flight (Watts)
    float p_climb{420.0f};           ///< Power consumed during constant climb (Watts)
    float p_descend{180.0f};         ///< Power consumed during steady descent (Watts)
    float p_land{200.0f};            ///< Power consumed during final touch-down/flare (Watts)

    float v_cruise_air{12.0f};       ///< Commanded cruise airspeed in still air (m/s)
    float v_climb{3.0f};             ///< Commanded vertical climb rate (m/s)
    float v_descend{2.0f};           ///< Commanded vertical descent rate (m/s)
    float v_land{0.8f};              ///< Final landing touch-down rate (m/s)
    float h_flare{3.0f};             ///< Altitude above touchdown plane where flare/land phase initiates (m)
    float min_rth_altitude{30.0f};   ///< Minimum transit altitude for obstacle clearance during RTH (m)
    float safe_climb_clearance{10.0f}; ///< Additional clearance above home altitude (m)
};

/**
 * @brief Result of wind triangle resolution for groundspeed and crabbing angle.
 */
struct WindTriangleResult {
    float groundspeed{0.0f};          ///< Resultant groundspeed along track toward home (m/s)
    float parallel_wind{0.0f};        ///< Wind component along track (m/s, >0 tailwind, <0 headwind)
    float crosswind{0.0f};            ///< Wind component perpendicular to track (m/s)
    bool wind_exceeds_airspeed{false};///< True if crosswind exceeds airspeed (unable to hold course)
    bool course_unachievable{false};  ///< True if groundspeed <= 0 due to severe headwind
};

/**
 * @brief Breakdown of calculated energy requirements per return trajectory phase.
 */
struct EnergyBreakdown {
    float e_climb_joules{0.0f};       ///< Energy required for initial climb to safe RTH transit altitude (J)
    float e_cruise_joules{0.0f};      ///< Energy required for horizontal transit against wind (J)
    float e_descend_joules{0.0f};     ///< Energy required for descent to flare height (J)
    float e_land_joules{0.0f};        ///< Energy required for touchdown / flare (J)
    float e_reserve_joules{0.0f};     ///< Regulatory and hover safety reserve energy (J)
    float e_sag_joules{0.0f};         ///< Energy lost to internal resistance voltage sag (Joule heating) (J)
    float e_total_req_joules{0.0f};   ///< Total required return energy: E_req = E_climb + E_cruise + E_descend + E_land + E_reserve + Delta_E_sag

    float t_climb_sec{0.0f};          ///< Climb duration (s)
    float t_cruise_sec{0.0f};         ///< Cruise duration (s)
    float t_descend_sec{0.0f};        ///< Descent duration (s)
    float t_land_sec{0.0f};           ///< Final landing duration (s)
    float t_return_total_sec{0.0f};   ///< Total flight time to complete RTH (s)
};

/**
 * @brief Comprehensive output of smart RTH evaluation.
 */
struct RthEvaluationResult {
    RthBatteryState state{RthBatteryState::Nominal}; ///< High-level contingency state
    bool rth_required{false};                       ///< True if RTH must be initiated immediately
    bool immediate_land_required{false};            ///< True if emergency cutoff requires landing in-place
    float current_soc{1.0f};                        ///< Current battery SoC (0.0 to 1.0)
    float required_soc{0.0f};                       ///< Minimum SoC required to complete RTH + reserve (0.0 to 1.0)
    float soc_margin{1.0f};                         ///< SoC margin: current_soc - required_soc
    float available_energy_joules{0.0f};            ///< Current usable energy in pack (J)
    float required_energy_joules{0.0f};             ///< Total required return energy (J)
    float current_pack_voltage{0.0f};               ///< Measured total pack voltage under load (V)
    float cell_voltage{0.0f};                       ///< Measured average per-cell voltage under load (V)
    EnergyBreakdown energy_breakdown;                ///< Detailed energy expenditure per phase
    WindTriangleResult wind_triangle;               ///< Wind triangle groundspeed solution
    std::string status_message;                     ///< Human-readable diagnostic description
};

/**
 * @class SmartRthBattery
 * @brief Production-grade contingency system evaluating real-time return-to-home energy feasibility.
 *
 * Implements:
 * 1. 5-phase trajectory energy budget: E_req = E_climb + E_cruise + E_descend + E_land + E_reserve
 * 2. Vector wind triangle groundspeed: v_g = sqrt(v_air^2 - w_perp^2) + w_parallel
 * 3. Lumped battery internal resistance voltage sag model: Delta_E_sag = I^2 * R_int * t_return
 * 4. Multi-tier state evaluation: Nominal, AdvisoryWarning, CriticalRthTrigger, EmergencyCutoffLand
 */
class SmartRthBattery {
public:
    SmartRthBattery();
    explicit SmartRthBattery(const BatteryConfig& battery_cfg, const VehiclePowerConfig& power_cfg);

    /**
     * @brief Update battery physical parameters.
     */
    void setBatteryConfig(const BatteryConfig& cfg) noexcept;

    /**
     * @brief Update vehicle power and aerodynamic parameters.
     */
    void setPowerConfig(const VehiclePowerConfig& cfg) noexcept;

    [[nodiscard]] const BatteryConfig& getBatteryConfig() const noexcept { return battery_cfg_; }
    [[nodiscard]] const VehiclePowerConfig& getPowerConfig() const noexcept { return power_cfg_; }

    /**
     * @brief Compute wind triangle groundspeed along return track toward home.
     *
     * Formula:
     *   w_parallel = wind . u_return
     *   w_perp = |wind x u_return|
     *   v_g = sqrt(max(0, v_air^2 - w_perp^2)) + w_parallel
     *
     * @param to_home_unit_dir Normalized 2D horizontal unit vector pointing from vehicle toward home
     * @param wind_vector 2D horizontal wind vector in m/s (direction wind is blowing toward)
     * @param v_air Commanded cruise airspeed in m/s
     * @return Resolved WindTriangleResult with groundspeed and safety flags
     */
    [[nodiscard]] WindTriangleResult computeWindTriangle(
        const Eigen::Vector2f& to_home_unit_dir,
        const Eigen::Vector2f& wind_vector,
        float v_air) const noexcept;

    /**
     * @brief Calculate multi-phase return trajectory energy and duration.
     *
     * @param current_pos Current 3D position [x, y, z] in local ENU frame (meters)
     * @param home_pos Home 3D position [x, y, z] in local ENU frame (meters)
     * @param wind_vector 2D horizontal wind vector [wx, wy] in m/s
     * @param out_wind Optional pointer to store resolved wind triangle
     * @return Calculated energy breakdown across all return phases
     */
    [[nodiscard]] EnergyBreakdown computeReturnEnergy(
        const Eigen::Vector3f& current_pos,
        const Eigen::Vector3f& home_pos,
        const Eigen::Vector2f& wind_vector,
        WindTriangleResult* out_wind = nullptr) const noexcept;

    /**
     * @brief Calculate resistive Joule heating energy lost to battery internal resistance under load.
     *
     * Formula:
     *   I_eff = P_avg / V_nominal
     *   Delta_E_sag = I_eff^2 * R_int * t_return
     *
     * @param return_time_sec Total predicted return flight time (s)
     * @param avg_power_watts Time-weighted average power consumption across return phases (W)
     * @return Energy dissipated in battery internal resistance (Joules)
     */
    [[nodiscard]] float computeVoltageSagEnergy(
        float return_time_sec,
        float avg_power_watts) const noexcept;

    /**
     * @brief Evaluate real-time contingency state and RTH requirements.
     *
     * @param current_soc Current estimated State-of-Charge (0.0 to 1.0)
     * @param current_voltage Measured total pack terminal voltage under load (V)
     * @param current_pos Current 3D vehicle position [x, y, z] in ENU (m)
     * @param home_pos Home 3D position [x, y, z] in ENU (m)
     * @param wind_vector Estimated 2D horizontal wind vector [wx, wy] in m/s
     * @return Comprehensive evaluation result including state, margin, and actions
     */
    [[nodiscard]] RthEvaluationResult evaluate(
        float current_soc,
        float current_voltage,
        const Eigen::Vector3f& current_pos,
        const Eigen::Vector3f& home_pos,
        const Eigen::Vector2f& wind_vector) const;

    /**
     * @brief Convert state enumeration to human-readable string.
     */
    [[nodiscard]] static std::string stateToString(RthBatteryState state) noexcept;

private:
    BatteryConfig battery_cfg_;
    VehiclePowerConfig power_cfg_;
};

} // namespace px4_airsim_autonomy::production
