#include "px4_airsim_autonomy/production/adsb_deconfliction.hpp"

#include <sstream>
#include <iomanip>

namespace px4_airsim_autonomy::production {

AdsbDeconfliction::AdsbDeconfliction()
    : config_{} {}

AdsbDeconfliction::AdsbDeconfliction(const DaaConfig& config)
    : config_(config) {}

void AdsbDeconfliction::setConfig(const DaaConfig& config) noexcept {
    config_ = config;
}

bool AdsbDeconfliction::checkRungViolation(
    float r_horiz,
    float tau_mod,
    float hmd,
    float dz,
    float t_coa,
    bool is_conv_horiz,
    bool is_conv_vert,
    const DaaRungThresholds& rung) const noexcept {
    // 1. Horizontal DAA condition
    const bool horiz_violation = (r_horiz <= rung.d_mod_m) ||
        (is_conv_horiz && (tau_mod >= 0.0f) && (tau_mod <= rung.tau_mod_sec) && (hmd <= rung.hmd_m));

    // 2. Vertical DAA condition
    const bool vert_violation = (dz <= rung.z_thr_m) ||
        (is_conv_vert && (t_coa >= 0.0f) && (t_coa <= rung.t_v_sec));

    return horiz_violation && vert_violation;
}

IntruderMetrics AdsbDeconfliction::evaluateIntruder(
    const OwnshipState& ownship,
    const AdsbTarget& intruder) const noexcept {
    IntruderMetrics m;
    m.icao_address = intruder.icao_address;
    m.callsign = intruder.callsign;

    // Relative horizontal position and range
    const Eigen::Vector2f r_h = intruder.position.head<2>() - ownship.position.head<2>();
    m.horizontal_range = r_h.norm();

    // Relative vertical separation
    const float delta_z = intruder.position.z() - ownship.position.z();
    m.vertical_separation = std::abs(delta_z);

    // Relative velocities
    const Eigen::Vector2f v_rel_h = intruder.velocity.head<2>() - ownship.velocity.head<2>();
    const float v_rel_z = intruder.velocity.z() - ownship.velocity.z();

    // Range rate and horizontal convergence
    const float r_dot_v = r_h.dot(v_rel_h);
    m.is_converging_horiz = (r_dot_v < -1e-4f);

    if (m.horizontal_range > 1e-4f) {
        m.range_rate = -r_dot_v / m.horizontal_range;
    } else {
        m.range_rate = 0.0f;
    }

    // DO-365B Modified Tau evaluated against baseline Warning D_MOD
    const float d_mod = config_.warning.d_mod_m;
    if (m.horizontal_range <= d_mod) {
        m.tau_mod = 0.0f; // Inside D_MOD boundary
    } else if (m.is_converging_horiz) {
        // tau_mod = - (r_horiz^2 - D_MOD^2) / (r_h . v_rel,h)
        m.tau_mod = (m.horizontal_range * m.horizontal_range - d_mod * d_mod) / (-r_dot_v);
    } else {
        m.tau_mod = std::numeric_limits<float>::infinity();
    }

    // Horizontal Miss Distance (HMD) at CPA
    const float v_rel_sq = v_rel_h.squaredNorm();
    if (v_rel_sq < 1e-4f) {
        m.time_to_cpa = 0.0f;
        m.hmd = m.horizontal_range;
    } else {
        const float t_cpa = -r_dot_v / v_rel_sq;
        m.time_to_cpa = std::max(0.0f, t_cpa);

        if (t_cpa < 0.0f) {
            // Diverging: closest point was in the past
            m.hmd = m.horizontal_range;
        } else {
            const Eigen::Vector2f r_cpa = r_h + v_rel_h * t_cpa;
            m.hmd = r_cpa.norm();
        }
    }

    // Vertical convergence and time to co-altitude (t_coa)
    m.is_converging_vert = (delta_z * v_rel_z < -1e-4f);
    if (m.vertical_separation < 1.0f) {
        m.time_to_coaltitude = 0.0f; // Already co-altitude
    } else if (m.is_converging_vert && std::abs(v_rel_z) > 1e-3f) {
        m.time_to_coaltitude = m.vertical_separation / std::abs(v_rel_z);
    } else {
        m.time_to_coaltitude = std::numeric_limits<float>::infinity();
    }

    // Helper lambda to compute rung-specific tau_mod
    auto compute_rung_tau = [&](float rung_d_mod) -> float {
        if (m.horizontal_range <= rung_d_mod) {
            return 0.0f;
        }
        if (m.is_converging_horiz) {
            return (m.horizontal_range * m.horizontal_range - rung_d_mod * rung_d_mod) / (-r_dot_v);
        }
        return std::numeric_limits<float>::infinity();
    };

    // Evaluate DO-365B Alerting Rungs in decreasing priority
    if (checkRungViolation(m.horizontal_range, compute_rung_tau(config_.warning.d_mod_m), m.hmd,
                           m.vertical_separation, m.time_to_coaltitude,
                           m.is_converging_horiz, m.is_converging_vert, config_.warning)) {
        m.alert_level = DaaAlertLevel::Level4_Warning;
    } else if (checkRungViolation(m.horizontal_range, compute_rung_tau(config_.corrective.d_mod_m), m.hmd,
                                  m.vertical_separation, m.time_to_coaltitude,
                                  m.is_converging_horiz, m.is_converging_vert, config_.corrective)) {
        m.alert_level = DaaAlertLevel::Level3_Corrective;
    } else if (checkRungViolation(m.horizontal_range, compute_rung_tau(config_.preventive.d_mod_m), m.hmd,
                                  m.vertical_separation, m.time_to_coaltitude,
                                  m.is_converging_horiz, m.is_converging_vert, config_.preventive)) {
        m.alert_level = DaaAlertLevel::Level2_Preventive;
    } else if (checkRungViolation(m.horizontal_range, compute_rung_tau(config_.advisory.d_mod_m), m.hmd,
                                  m.vertical_separation, m.time_to_coaltitude,
                                  m.is_converging_horiz, m.is_converging_vert, config_.advisory)) {
        m.alert_level = DaaAlertLevel::Level1_TrafficAdvisory;
    } else {
        m.alert_level = DaaAlertLevel::Level0_Normal;
    }

    return m;
}

Eigen::Vector3f AdsbDeconfliction::computeEvasiveCommand(
    const OwnshipState& ownship,
    const AdsbTarget& threat,
    const IntruderMetrics& metrics) const noexcept {
    // 1. Determine Vertical Evasive Rate
    float vz_cmd = 0.0f;
    const bool altitude_above_floor = (ownship.position.z() >= config_.min_safe_agl_for_dive);
    const bool threat_above_or_level = (threat.position.z() >= (ownship.position.z() - 10.0f));

    if (metrics.alert_level == DaaAlertLevel::Level4_Warning) {
        if (altitude_above_floor && threat_above_or_level) {
            // Mandated emergency descent dive rate of -4.0 m/s
            vz_cmd = config_.emergency_dive_rate;
        } else {
            // Safe emergency climb rate of +3.0 m/s
            vz_cmd = config_.emergency_climb_rate;
        }
    } else if (metrics.alert_level == DaaAlertLevel::Level3_Corrective) {
        if (altitude_above_floor && threat_above_or_level) {
            vz_cmd = config_.corrective_dive_rate;
        } else {
            vz_cmd = 1.5f;
        }
    } else {
        vz_cmd = ownship.velocity.z();
    }

    // 2. Determine Horizontal Evasive Vector
    // Break perpendicular to line-of-sight vector toward intruder
    const Eigen::Vector2f r_h = threat.position.head<2>() - ownship.position.head<2>();
    const float dist_h = r_h.norm();

    Eigen::Vector2f v_horiz_cmd = ownship.velocity.head<2>();

    if (dist_h > 0.1f) {
        const Eigen::Vector2f u_los = r_h / dist_h;
        // Two orthogonal escape directions
        const Eigen::Vector2f n1(-u_los.y(), u_los.x());
        const Eigen::Vector2f n2(u_los.y(), -u_los.x());

        // Steer away from intruder velocity track
        const Eigen::Vector2f v_int_h = threat.velocity.head<2>();
        const Eigen::Vector2f best_escape = (n1.dot(v_int_h) < n2.dot(v_int_h)) ? n1 : n2;

        v_horiz_cmd = best_escape * config_.evasive_cruise_speed;
    }

    return Eigen::Vector3f(v_horiz_cmd.x(), v_horiz_cmd.y(), vz_cmd);
}

DeconflictionResult AdsbDeconfliction::evaluateTraffic(
    const OwnshipState& ownship,
    const std::vector<AdsbTarget>& traffic) const {
    DeconflictionResult result;
    result.max_alert_level = DaaAlertLevel::Level0_Normal;
    result.evasive_action_required = false;
    result.recommended_velocity_cmd = ownship.velocity;

    if (traffic.empty()) {
        result.advisory_message = "[DAA LEVEL 0: NORMAL] Airspace clear. No ADS-B traffic detected.";
        return result;
    }

    int best_threat_idx = -1;
    float min_tau_mod = std::numeric_limits<float>::infinity();
    float min_range = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < traffic.size(); ++i) {
        if (!traffic[i].is_valid) {
            continue;
        }

        const auto metrics = evaluateIntruder(ownship, traffic[i]);
        result.all_intruder_metrics.push_back(metrics);

        // Priority 1: Higher alert level takes precedence
        if (static_cast<uint8_t>(metrics.alert_level) > static_cast<uint8_t>(result.max_alert_level)) {
            result.max_alert_level = metrics.alert_level;
            best_threat_idx = static_cast<int>(i);
            min_tau_mod = metrics.tau_mod;
            min_range = metrics.horizontal_range;
        }
        // Priority 2: Tie-break with smallest tau_mod
        else if (metrics.alert_level == result.max_alert_level && result.max_alert_level != DaaAlertLevel::Level0_Normal) {
            if (metrics.tau_mod < min_tau_mod || (std::abs(metrics.tau_mod - min_tau_mod) < 0.5f && metrics.horizontal_range < min_range)) {
                best_threat_idx = static_cast<int>(i);
                min_tau_mod = metrics.tau_mod;
                min_range = metrics.horizontal_range;
            }
        }
    }

    if (best_threat_idx >= 0) {
        const auto& threat = traffic[static_cast<size_t>(best_threat_idx)];
        result.primary_threat_icao = threat.icao_address;
        result.primary_threat_callsign = threat.callsign;
        result.primary_threat_metrics = evaluateIntruder(ownship, threat);

        // Generate evasive commands for active advisory levels
        if (result.max_alert_level == DaaAlertLevel::Level4_Warning ||
            result.max_alert_level == DaaAlertLevel::Level3_Corrective) {
            result.evasive_action_required = true;
            result.recommended_velocity_cmd = computeEvasiveCommand(ownship, threat, result.primary_threat_metrics);
        }
    }

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1);

    switch (result.max_alert_level) {
        case DaaAlertLevel::Level4_Warning:
            ss << "[DAA LEVEL 4: WARNING] Critical Well Clear Violation! Threat ICAO 0x"
               << std::hex << result.primary_threat_icao << std::dec << " ("
               << result.primary_threat_callsign << "). Range: "
               << result.primary_threat_metrics.horizontal_range << "m, TauMod: "
               << result.primary_threat_metrics.tau_mod << "s, HMD: "
               << result.primary_threat_metrics.hmd << "m. Mandating emergency dive rate "
               << result.recommended_velocity_cmd.z() << " m/s.";
            break;
        case DaaAlertLevel::Level3_Corrective:
            ss << "[DAA LEVEL 3: CORRECTIVE] Corrective resolution required! Threat ICAO 0x"
               << std::hex << result.primary_threat_icao << std::dec << " ("
               << result.primary_threat_callsign << "). TauMod: "
               << result.primary_threat_metrics.tau_mod << "s, HMD: "
               << result.primary_threat_metrics.hmd << "m. Commanded avoidance vz: "
               << result.recommended_velocity_cmd.z() << " m/s.";
            break;
        case DaaAlertLevel::Level2_Preventive:
            ss << "[DAA LEVEL 2: PREVENTIVE] Preventive advisory active for traffic 0x"
               << std::hex << result.primary_threat_icao << std::dec
               << ". Maintain current trajectory; do not reduce separation.";
            break;
        case DaaAlertLevel::Level1_TrafficAdvisory:
            ss << "[DAA LEVEL 1: TRAFFIC ADVISORY] Traffic nearby 0x"
               << std::hex << result.primary_threat_icao << std::dec << " ("
               << result.primary_threat_callsign << "). Range: "
               << result.primary_threat_metrics.horizontal_range << "m. Situational monitoring.";
            break;
        case DaaAlertLevel::Level0_Normal:
        default:
            ss << "[DAA LEVEL 0: NORMAL] Airspace clear. All traffic outside DAA alert thresholds.";
            break;
    }

    result.advisory_message = ss.str();
    return result;
}

std::string AdsbDeconfliction::alertLevelToString(DaaAlertLevel level) noexcept {
    switch (level) {
        case DaaAlertLevel::Level0_Normal:
            return "Level0_Normal";
        case DaaAlertLevel::Level1_TrafficAdvisory:
            return "Level1_TrafficAdvisory";
        case DaaAlertLevel::Level2_Preventive:
            return "Level2_Preventive";
        case DaaAlertLevel::Level3_Corrective:
            return "Level3_Corrective";
        case DaaAlertLevel::Level4_Warning:
            return "Level4_Warning";
        default:
            return "Unknown";
    }
}

} // namespace px4_airsim_autonomy::production
