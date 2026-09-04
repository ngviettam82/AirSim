#include "px4_airsim_autonomy/production/mission_continuity.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace px4_airsim_autonomy::production {

// -----------------------------------------------------------------------------
// BreakpointState JSON Serialization & Deserialization
// -----------------------------------------------------------------------------

std::string BreakpointState::toJson() const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "{\n";
    ss << "  \"mission_id\": \"" << mission_id << "\",\n";
    ss << "  \"current_waypoint_index\": " << current_waypoint_index << ",\n";
    ss << "  \"strip_index\": " << strip_index << ",\n";
    ss << "  \"abort_position\": ["
       << abort_position.x() << ", "
       << abort_position.y() << ", "
       << abort_position.z() << "],\n";
    ss << "  \"progress_ratio\": " << progress_ratio << ",\n";
    ss << "  \"heading_rad\": " << heading_rad << ",\n";
    ss << "  \"flight_speed_m_s\": " << flight_speed_m_s << ",\n";
    ss << "  \"battery_percentage\": " << battery_percentage << ",\n";
    ss << "  \"timestamp_utc_ms\": " << timestamp_utc_ms << ",\n";
    ss << "  \"abort_reason\": \"" << abort_reason << "\"\n";
    ss << "}";
    return ss.str();
}

namespace {

// Helper to extract a string value for a given key from JSON
std::optional<std::string> extractJsonString(const std::string& json, const std::string& key) {
    const std::string search = "\"" + key + "\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return std::nullopt;

    pos = json.find(':', pos);
    if (pos == std::string::npos) return std::nullopt;

    size_t start_quote = json.find('\"', pos);
    if (start_quote == std::string::npos) return std::nullopt;

    size_t end_quote = json.find('\"', start_quote + 1);
    if (end_quote == std::string::npos) return std::nullopt;

    return json.substr(start_quote + 1, end_quote - start_quote - 1);
}

// Helper to extract a numeric double value for a given key from JSON
std::optional<double> extractJsonDouble(const std::string& json, const std::string& key) {
    const std::string search = "\"" + key + "\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return std::nullopt;

    pos = json.find(':', pos);
    if (pos == std::string::npos) return std::nullopt;

    size_t val_start = json.find_first_of("-+0123456789", pos);
    if (val_start == std::string::npos) return std::nullopt;

    size_t val_end = json.find_first_not_of("-+0123456789.eE", val_start);
    std::string token = (val_end == std::string::npos) ? json.substr(val_start) : json.substr(val_start, val_end - val_start);

    try {
        return std::stod(token);
    } catch (...) {
        return std::nullopt;
    }
}

// Helper to extract a 3-element vector [x, y, z] from JSON
std::optional<Eigen::Vector3d> extractJsonVector3d(const std::string& json, const std::string& key) {
    const std::string search = "\"" + key + "\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return std::nullopt;

    size_t bracket_open = json.find('[', pos);
    size_t bracket_close = json.find(']', bracket_open);
    if (bracket_open == std::string::npos || bracket_close == std::string::npos) return std::nullopt;

    std::string array_str = json.substr(bracket_open + 1, bracket_close - bracket_open - 1);
    std::stringstream ss(array_str);
    std::string token;
    std::vector<double> vals;

    while (std::getline(ss, token, ',')) {
        size_t start = token.find_first_of("-+0123456789");
        if (start != std::string::npos) {
            size_t end = token.find_first_not_of("-+0123456789.eE", start);
            std::string num = (end == std::string::npos) ? token.substr(start) : token.substr(start, end - start);
            try {
                vals.push_back(std::stod(num));
            } catch (...) {}
        }
    }

    if (vals.size() >= 3) {
        return Eigen::Vector3d(vals[0], vals[1], vals[2]);
    }
    return std::nullopt;
}

} // anonymous namespace

std::optional<BreakpointState> BreakpointState::fromJson(const std::string& json_str) {
    BreakpointState state;

    if (auto mid = extractJsonString(json_str, "mission_id")) {
        state.mission_id = *mid;
    }
    if (auto r = extractJsonString(json_str, "abort_reason")) {
        state.abort_reason = *r;
    }
    if (auto wp = extractJsonDouble(json_str, "current_waypoint_index")) {
        state.current_waypoint_index = static_cast<size_t>(*wp);
    }
    if (auto st = extractJsonDouble(json_str, "strip_index")) {
        state.strip_index = static_cast<int>(*st);
    }
    if (auto pos = extractJsonVector3d(json_str, "abort_position")) {
        state.abort_position = *pos;
    }
    if (auto prog = extractJsonDouble(json_str, "progress_ratio")) {
        state.progress_ratio = *prog;
    }
    if (auto yaw = extractJsonDouble(json_str, "heading_rad")) {
        state.heading_rad = *yaw;
    }
    if (auto spd = extractJsonDouble(json_str, "flight_speed_m_s")) {
        state.flight_speed_m_s = *spd;
    }
    if (auto bat = extractJsonDouble(json_str, "battery_percentage")) {
        state.battery_percentage = *bat;
    }
    if (auto ts = extractJsonDouble(json_str, "timestamp_utc_ms")) {
        state.timestamp_utc_ms = static_cast<int64_t>(*ts);
    }

    return state;
}

bool BreakpointState::saveToFile(const std::string& filepath) const {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) return false;
    ofs << toJson();
    return ofs.good();
}

std::optional<BreakpointState> BreakpointState::loadFromFile(const std::string& filepath) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) return std::nullopt;

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    return fromJson(buffer.str());
}

// -----------------------------------------------------------------------------
// Backtrack Calculations
// -----------------------------------------------------------------------------

double BacktrackParameters::computeBacktrackDistance() const {
    return MissionContinuity::computeBacktrackDistance(*this);
}

double MissionContinuity::computeBacktrackDistance(const BacktrackParameters& params) {
    // Photogrammetric overlap buffer: 2 * D_trigger guarantees at least 2 overlapping photo frames
    const double d_photo = 2.0 * std::max(0.0, params.trigger_distance_m);

    // Kinematic buffer: (v^2 / (2 * a)) + v * t_settle
    // Accelerates from stop to v, plus stabilization duration
    double d_kinematic = 0.0;
    if (params.max_acceleration_m_s2 > 1e-4 && params.nominal_flight_speed_m_s > 0.0) {
        const double v = params.nominal_flight_speed_m_s;
        const double a = params.max_acceleration_m_s2;
        const double t_settle = std::max(0.0, params.settling_time_s);

        const double d_accel = (v * v) / (2.0 * a);
        const double d_settle = v * t_settle;
        d_kinematic = d_accel + d_settle;
    }

    // Dynamic buffer selection
    const double d_backtrack = std::max({d_photo, d_kinematic, params.min_backtrack_buffer_m});
    return d_backtrack;
}

// -----------------------------------------------------------------------------
// Trajectory Synthesis
// -----------------------------------------------------------------------------

ResumptionPlan MissionContinuity::synthesizeResumptionTrajectory(
    const std::vector<SweepWaypoint>& original_mission,
    const BreakpointState& breakpoint,
    const Eigen::Vector3d& drone_current_position,
    const BacktrackParameters& backtrack_params,
    const IngressConfig& ingress_config) {

    ResumptionPlan plan;
    plan.breakpoint = breakpoint;

    if (original_mission.empty()) {
        return plan;
    }

    // 1. Calculate dynamic backtrack buffer
    const double backtrack_target_m = computeBacktrackDistance(backtrack_params);

    // 2. Identify the mission segment containing the abort position
    size_t abort_seg_idx = 1;
    double min_proj_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector3d abort_proj_pt = original_mission.front().position;

    // Search window centered around recorded waypoint index if valid
    size_t search_start = 1;
    size_t search_end = original_mission.size();
    if (breakpoint.current_waypoint_index > 0 && breakpoint.current_waypoint_index < original_mission.size()) {
        search_start = (breakpoint.current_waypoint_index > 2) ? breakpoint.current_waypoint_index - 2 : 1;
        search_end = std::min(original_mission.size(), breakpoint.current_waypoint_index + 3);
    }

    for (size_t i = search_start; i < search_end; ++i) {
        const Eigen::Vector3d& p1 = original_mission[i - 1].position;
        const Eigen::Vector3d& p2 = original_mission[i].position;
        const Eigen::Vector3d v = p2 - p1;
        const double len = v.norm();

        if (len > 1e-4) {
            const double t = std::clamp((breakpoint.abort_position - p1).dot(v) / (len * len), 0.0, 1.0);
            const Eigen::Vector3d proj = p1 + t * v;
            const double dist = (breakpoint.abort_position - proj).norm();
            if (dist < min_proj_dist) {
                min_proj_dist = dist;
                abort_seg_idx = i;
                abort_proj_pt = proj;
            }
        }
    }

    // 3. Backtrack along the polyline path by backtrack_target_m
    Eigen::Vector3d reengagement_pt = abort_proj_pt;
    size_t resume_from_waypoint = abort_seg_idx;
    double accumulated_backtrack = 0.0;
    double remaining_backtrack = backtrack_target_m;

    // First, walk back on the abort segment from abort_proj_pt to p1
    const Eigen::Vector3d seg_start = original_mission[abort_seg_idx - 1].position;
    const double dist_to_start = (abort_proj_pt - seg_start).norm();

    if (remaining_backtrack <= dist_to_start) {
        const Eigen::Vector3d seg_dir = (abort_proj_pt - seg_start).normalized();
        reengagement_pt = abort_proj_pt - remaining_backtrack * seg_dir;
        accumulated_backtrack = remaining_backtrack;
        remaining_backtrack = 0.0;
        resume_from_waypoint = abort_seg_idx;
    } else {
        accumulated_backtrack += dist_to_start;
        remaining_backtrack -= dist_to_start;
        reengagement_pt = seg_start;
        resume_from_waypoint = abort_seg_idx - 1;

        // Step back through earlier segments
        for (int k = static_cast<int>(abort_seg_idx) - 2; k >= 0; --k) {
            const Eigen::Vector3d& p_next = original_mission[k + 1].position;
            const Eigen::Vector3d& p_curr = original_mission[k].position;
            const double seg_len = (p_next - p_curr).norm();

            if (remaining_backtrack <= seg_len) {
                const Eigen::Vector3d dir = (p_next - p_curr).normalized();
                reengagement_pt = p_next - remaining_backtrack * dir;
                accumulated_backtrack += remaining_backtrack;
                remaining_backtrack = 0.0;
                resume_from_waypoint = k + 1;
                break;
            } else {
                accumulated_backtrack += seg_len;
                remaining_backtrack -= seg_len;
                reengagement_pt = p_curr;
                resume_from_waypoint = k;
            }
        }
    }

    plan.applied_backtrack_distance_m = accumulated_backtrack;
    plan.reengagement_point = reengagement_pt;
    plan.resumed_waypoint_index = resume_from_waypoint;

    // 4. Determine track heading vector at re-engagement point
    size_t ref_wp = std::min(resume_from_waypoint, original_mission.size() - 1);
    if (ref_wp == 0 && original_mission.size() > 1) ref_wp = 1;

    Eigen::Vector3d track_dir = (original_mission[ref_wp].position - original_mission[ref_wp - 1].position);
    if (track_dir.norm() > 1e-4) {
        track_dir.normalize();
    } else {
        track_dir = Eigen::Vector3d(1, 0, 0);
    }
    const double track_heading = std::atan2(track_dir.y(), track_dir.x());

    // 5. Synthesize smooth ingress corridor
    // Safe transit altitude above terrain and obstacles
    const double transit_z = std::max(ingress_config.safe_transit_altitude_agl_m,
                                     reengagement_pt.z() + 5.0);

    size_t g_idx = 0;
    std::vector<SweepWaypoint> res_wps;

    // Point 1: Ingress launch / climb to safe transit altitude
    {
        SweepWaypoint wp_climb;
        wp_climb.position = Eigen::Vector3d(drone_current_position.x(), drone_current_position.y(), transit_z);
        wp_climb.yaw_rad = track_heading;
        wp_climb.target_speed_m_s = ingress_config.transit_speed_m_s;
        wp_climb.type = WaypointType::SurveyEntry;
        wp_climb.strip_index = original_mission[ref_wp].strip_index;
        wp_climb.is_photo_trigger_zone = false;
        wp_climb.global_index = g_idx++;
        res_wps.push_back(wp_climb);
    }

    // Point 2: Alignment ingress point (elevated at transit altitude)
    const double lead_dist = std::max(5.0, ingress_config.alignment_lead_distance_m);
    const Eigen::Vector3d align_xy = reengagement_pt - lead_dist * track_dir;

    {
        SweepWaypoint wp_transit_align;
        wp_transit_align.position = Eigen::Vector3d(align_xy.x(), align_xy.y(), transit_z);
        wp_transit_align.yaw_rad = track_heading;
        wp_transit_align.target_speed_m_s = ingress_config.transit_speed_m_s;
        wp_transit_align.type = WaypointType::TurnEntry;
        wp_transit_align.strip_index = original_mission[ref_wp].strip_index;
        wp_transit_align.is_photo_trigger_zone = false;
        wp_transit_align.global_index = g_idx++;
        res_wps.push_back(wp_transit_align);
    }

    // Point 3: Descend onto track line at survey altitude (aligned with track heading)
    {
        SweepWaypoint wp_approach;
        wp_approach.position = Eigen::Vector3d(align_xy.x(), align_xy.y(), reengagement_pt.z());
        wp_approach.yaw_rad = track_heading;
        wp_approach.target_speed_m_s = ingress_config.approach_speed_m_s;
        wp_approach.type = WaypointType::TurnEntry;
        wp_approach.strip_index = original_mission[ref_wp].strip_index;
        wp_approach.is_photo_trigger_zone = false;
        wp_approach.global_index = g_idx++;
        res_wps.push_back(wp_approach);
    }

    // Point 4: Re-engagement point (photogrammetry triggering ACTIVATES here)
    {
        SweepWaypoint wp_resume;
        wp_resume.position = reengagement_pt;
        wp_resume.yaw_rad = track_heading;
        wp_resume.target_speed_m_s = backtrack_params.nominal_flight_speed_m_s;
        wp_resume.type = WaypointType::SweepStart;
        wp_resume.strip_index = original_mission[ref_wp].strip_index;
        wp_resume.is_photo_trigger_zone = true; // Photo capture seamlessly resumes
        wp_resume.global_index = g_idx++;
        res_wps.push_back(wp_resume);
    }

    // 6. Splice in remaining original mission waypoints
    for (size_t i = resume_from_waypoint; i < original_mission.size(); ++i) {
        SweepWaypoint wp = original_mission[i];
        wp.global_index = g_idx++;
        res_wps.push_back(wp);
    }

    // 7. Calculate distance metrics
    double ingress_dist = 0.0;
    for (size_t i = 0; i < 3; ++i) {
        ingress_dist += (res_wps[i + 1].position - res_wps[i].position).norm();
    }

    double survey_dist = 0.0;
    for (size_t i = 3; i + 1 < res_wps.size(); ++i) {
        survey_dist += (res_wps[i + 1].position - res_wps[i].position).norm();
    }

    plan.resumed_waypoints = std::move(res_wps);
    plan.total_ingress_distance_m = ingress_dist;
    plan.remaining_survey_distance_m = survey_dist;

    return plan;
}

} // namespace px4_airsim_autonomy::production
