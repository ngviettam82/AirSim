#include "px4_airsim_autonomy/production/smart_rth_battery.hpp"

#include <sstream>
#include <iomanip>

namespace px4_airsim_autonomy::production {

SmartRthBattery::SmartRthBattery()
    : battery_cfg_{}, power_cfg_{} {}

SmartRthBattery::SmartRthBattery(const BatteryConfig& battery_cfg, const VehiclePowerConfig& power_cfg)
    : battery_cfg_(battery_cfg), power_cfg_(power_cfg) {}

void SmartRthBattery::setBatteryConfig(const BatteryConfig& cfg) noexcept {
    battery_cfg_ = cfg;
}

void SmartRthBattery::setPowerConfig(const VehiclePowerConfig& cfg) noexcept {
    power_cfg_ = cfg;
}

WindTriangleResult SmartRthBattery::computeWindTriangle(
    const Eigen::Vector2f& to_home_unit_dir,
    const Eigen::Vector2f& wind_vector,
    float v_air) const noexcept {
    WindTriangleResult result;

    // Zero-length or invalid direction guard
    if (to_home_unit_dir.squaredNorm() < 1e-6f || v_air <= 0.0f) {
        result.groundspeed = 0.0f;
        result.course_unachievable = true;
        return result;
    }

    // Parallel wind component: projection of wind along return ground track (positive = tailwind, negative = headwind)
    result.parallel_wind = wind_vector.dot(to_home_unit_dir);

    // Crosswind component: perpendicular projection |wx * uy - wy * ux|
    result.crosswind = std::abs(wind_vector.x() * to_home_unit_dir.y() - wind_vector.y() * to_home_unit_dir.x());

    // Check if crosswind equals or exceeds airspeed capability
    if (result.crosswind >= v_air) {
        result.wind_exceeds_airspeed = true;
        result.groundspeed = 0.0f;
        result.course_unachievable = true;
        return result;
    }

    // Forward component of airspeed after crabbing into crosswind
    const float forward_air_sq = v_air * v_air - result.crosswind * result.crosswind;
    const float forward_air = std::sqrt(std::max(0.0f, forward_air_sq));

    // Resultant groundspeed along track
    const float v_ground = forward_air + result.parallel_wind;

    // Check if headwind prevents forward progress
    if (v_ground <= 0.1f) {
        result.groundspeed = std::max(0.0f, v_ground);
        result.course_unachievable = true;
    } else {
        result.groundspeed = v_ground;
        result.course_unachievable = false;
    }

    return result;
}

EnergyBreakdown SmartRthBattery::computeReturnEnergy(
    const Eigen::Vector3f& current_pos,
    const Eigen::Vector3f& home_pos,
    const Eigen::Vector2f& wind_vector,
    WindTriangleResult* out_wind) const noexcept {
    EnergyBreakdown bd;

    // 1. Safe RTH cruise altitude
    const float safe_transit_z = std::max({
        current_pos.z(),
        power_cfg_.min_rth_altitude,
        home_pos.z() + power_cfg_.safe_climb_clearance
    });

    // 2. Climb phase
    const float delta_z_climb = std::max(0.0f, safe_transit_z - current_pos.z());
    const float v_climb = std::max(0.1f, power_cfg_.v_climb);
    bd.t_climb_sec = delta_z_climb / v_climb;
    bd.e_climb_joules = power_cfg_.p_climb * bd.t_climb_sec;

    // 3. Horizontal Cruise phase
    const Eigen::Vector2f delta_xy = home_pos.head<2>() - current_pos.head<2>();
    const float dist_horiz = delta_xy.norm();

    WindTriangleResult wind_res;
    if (dist_horiz > 0.05f) {
        const Eigen::Vector2f u_return = delta_xy / dist_horiz;
        wind_res = computeWindTriangle(u_return, wind_vector, power_cfg_.v_cruise_air);

        if (wind_res.course_unachievable) {
            // Infinite / critical cruise time penalty when blown back or trapped
            bd.t_cruise_sec = 1e6f;
            bd.e_cruise_joules = 1e9f;
        } else {
            bd.t_cruise_sec = dist_horiz / wind_res.groundspeed;
            bd.e_cruise_joules = power_cfg_.p_cruise * bd.t_cruise_sec;
        }
    } else {
        // Vehicle already directly above home
        bd.t_cruise_sec = 0.0f;
        bd.e_cruise_joules = 0.0f;
        wind_res.groundspeed = power_cfg_.v_cruise_air;
    }

    if (out_wind != nullptr) {
        *out_wind = wind_res;
    }

    // 4. Descent phase (from cruise altitude to flare height above home)
    const float flare_alt = home_pos.z() + power_cfg_.h_flare;
    const float delta_z_descend = std::max(0.0f, safe_transit_z - flare_alt);
    const float v_descend = std::max(0.1f, power_cfg_.v_descend);
    bd.t_descend_sec = delta_z_descend / v_descend;
    bd.e_descend_joules = power_cfg_.p_descend * bd.t_descend_sec;

    // 5. Final landing touchdown phase
    const float delta_z_land = std::max(0.0f, std::min(power_cfg_.h_flare, safe_transit_z - home_pos.z()));
    const float v_land = std::max(0.1f, power_cfg_.v_land);
    bd.t_land_sec = delta_z_land / v_land;
    bd.e_land_joules = power_cfg_.p_land * bd.t_land_sec;

    // 6. Regulatory and safety reserve energy
    const float e_res_time = power_cfg_.p_hover * battery_cfg_.min_reserve_time_sec;
    const float e_res_soc = battery_cfg_.reserve_soc_fraction * battery_cfg_.totalCapacityJoules();
    bd.e_reserve_joules = std::max(e_res_time, e_res_soc);

    // 7. Total active flight duration (excluding reserve hover)
    bd.t_return_total_sec = bd.t_climb_sec + bd.t_cruise_sec + bd.t_descend_sec + bd.t_land_sec;

    // 8. Voltage Sag Model (Joule heating in internal resistance)
    const float active_energy = bd.e_climb_joules + bd.e_cruise_joules + bd.e_descend_joules + bd.e_land_joules;
    const float avg_power = (bd.t_return_total_sec > 0.001f && bd.t_return_total_sec < 1e5f)
        ? (active_energy / bd.t_return_total_sec)
        : power_cfg_.p_hover;

    bd.e_sag_joules = computeVoltageSagEnergy(bd.t_return_total_sec, avg_power);

    // Sum total required energy
    bd.e_total_req_joules = bd.e_climb_joules + bd.e_cruise_joules + bd.e_descend_joules +
                            bd.e_land_joules + bd.e_reserve_joules + bd.e_sag_joules;

    return bd;
}

float SmartRthBattery::computeVoltageSagEnergy(
    float return_time_sec,
    float avg_power_watts) const noexcept {
    if (return_time_sec <= 0.0f || avg_power_watts <= 0.0f || battery_cfg_.nominal_voltage <= 0.0f) {
        return 0.0f;
    }

    if (return_time_sec >= 1e5f) {
        return 1e8f; // Clamp to avoid numerical overflow on unachievable flight
    }

    // Effective return transit current: I_eff = P_avg / V_nominal
    const float i_eff = avg_power_watts / battery_cfg_.nominal_voltage;

    // Delta_E_sag = I^2 * R_int * t_return (Joule heating)
    const float delta_e_sag = (i_eff * i_eff) * battery_cfg_.internal_resistance_ohm * return_time_sec;
    return delta_e_sag;
}

RthEvaluationResult SmartRthBattery::evaluate(
    float current_soc,
    float current_voltage,
    const Eigen::Vector3f& current_pos,
    const Eigen::Vector3f& home_pos,
    const Eigen::Vector2f& wind_vector) const {
    RthEvaluationResult result;
    result.current_soc = std::clamp(current_soc, 0.0f, 1.0f);
    result.current_pack_voltage = current_voltage;

    const int cells = std::max(1, battery_cfg_.cells_in_series);
    result.cell_voltage = current_voltage / static_cast<float>(cells);

    const float total_capacity_j = battery_cfg_.totalCapacityJoules();
    result.available_energy_joules = result.current_soc * total_capacity_j;

    // Compute return energy requirement and wind triangle
    result.energy_breakdown = computeReturnEnergy(current_pos, home_pos, wind_vector, &result.wind_triangle);
    result.required_energy_joules = result.energy_breakdown.e_total_req_joules;

    if (total_capacity_j > 1.0f) {
        result.required_soc = std::clamp(result.required_energy_joules / total_capacity_j, 0.0f, 1.0f);
    } else {
        result.required_soc = 1.0f;
    }

    result.soc_margin = result.current_soc - result.required_soc;

    // Multi-tier State Machine Evaluation
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    // Condition 1: Emergency Cutoff Land
    if (result.cell_voltage <= battery_cfg_.crit_cell_voltage ||
        result.current_soc <= battery_cfg_.emergency_cutoff_soc ||
        result.wind_triangle.course_unachievable) {
        result.state = RthBatteryState::EmergencyCutoffLand;
        result.rth_required = true;
        result.immediate_land_required = true;

        if (result.cell_voltage <= battery_cfg_.crit_cell_voltage) {
            ss << "[EMERGENCY CUTOFF] Critical cell voltage sag (" << result.cell_voltage << "V <= "
               << battery_cfg_.crit_cell_voltage << "V threshold). Immediate land commanded.";
        } else if (result.current_soc <= battery_cfg_.emergency_cutoff_soc) {
            ss << "[EMERGENCY CUTOFF] Critical low SoC (" << (result.current_soc * 100.0f) << "% <= "
               << (battery_cfg_.emergency_cutoff_soc * 100.0f) << "% cutoff). Immediate land commanded.";
        } else {
            ss << "[EMERGENCY CUTOFF] Return course unachievable due to severe wind (Vg="
               << result.wind_triangle.groundspeed << "m/s, crosswind="
               << result.wind_triangle.crosswind << "m/s). Immediate land commanded.";
        }
    }
    // Condition 2: Critical RTH Trigger
    else if (result.current_soc <= result.required_soc ||
             result.cell_voltage <= battery_cfg_.warn_cell_voltage) {
        result.state = RthBatteryState::CriticalRthTrigger;
        result.rth_required = true;
        result.immediate_land_required = false;

        if (result.current_soc <= result.required_soc) {
            ss << "[CRITICAL RTH] Battery energy depleted to return requirement (SoC: "
               << (result.current_soc * 100.0f) << "% <= Req: " << (result.required_soc * 100.0f)
               << "%, ReqEnergy: " << (result.required_energy_joules / 1000.0f) << " kJ). Return-To-Home triggered.";
        } else {
            ss << "[CRITICAL RTH] Cell voltage reached warning threshold (" << result.cell_voltage
               << "V <= " << battery_cfg_.warn_cell_voltage << "V). Return-To-Home triggered.";
        }
    }
    // Condition 3: Advisory Warning
    else if (result.current_soc <= (result.required_soc + battery_cfg_.advisory_soc_margin)) {
        result.state = RthBatteryState::AdvisoryWarning;
        result.rth_required = false;
        result.immediate_land_required = false;

        ss << "[ADVISORY WARNING] Battery approaching RTH threshold (SoC: "
           << (result.current_soc * 100.0f) << "%, Margin: "
           << (result.soc_margin * 100.0f) << "%). Plan return promptly.";
    }
    // Condition 4: Nominal Operation
    else {
        result.state = RthBatteryState::Nominal;
        result.rth_required = false;
        result.immediate_land_required = false;

        ss << "[NOMINAL] Energy budget healthy (SoC: " << (result.current_soc * 100.0f)
           << "%, Req: " << (result.required_soc * 100.0f) << "%, Margin: "
           << (result.soc_margin * 100.0f) << "%).";
    }

    result.status_message = ss.str();
    return result;
}

std::string SmartRthBattery::stateToString(RthBatteryState state) noexcept {
    switch (state) {
        case RthBatteryState::Nominal:
            return "Nominal";
        case RthBatteryState::AdvisoryWarning:
            return "AdvisoryWarning";
        case RthBatteryState::CriticalRthTrigger:
            return "CriticalRthTrigger";
        case RthBatteryState::EmergencyCutoffLand:
            return "EmergencyCutoffLand";
        default:
            return "Unknown";
    }
}

} // namespace px4_airsim_autonomy::production
