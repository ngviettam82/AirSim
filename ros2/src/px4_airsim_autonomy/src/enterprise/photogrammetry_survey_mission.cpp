#include "px4_airsim_autonomy/enterprise/photogrammetry_survey_mission.hpp"
#include <rclcpp/logging.hpp>
#include <cmath>

namespace px4_airsim_autonomy {
namespace enterprise {

PhotogrammetrySurveyMission::PhotogrammetrySurveyMission()
    : camera_spec_(production::CameraSpec::AirSim_Default_1080p())
{
    // Default 120m x 80m survey polygon
    survey_polygon_ = {
        {  0.0,   0.0},
        {120.0,   0.0},
        {120.0,  80.0},
        {  0.0,  80.0}
    };
}

void PhotogrammetrySurveyMission::init(rclcpp::Node& node)
{
    target_altitude_agl_ = node.has_parameter("survey.altitude") ? node.get_parameter("survey.altitude").as_double() : node.declare_parameter<double>("survey.altitude", 30.0);
    forward_overlap_ = node.has_parameter("survey.forward_overlap") ? node.get_parameter("survey.forward_overlap").as_double() : node.declare_parameter<double>("survey.forward_overlap", 0.75);
    side_overlap_ = node.has_parameter("survey.side_overlap") ? node.get_parameter("survey.side_overlap").as_double() : node.declare_parameter<double>("survey.side_overlap", 0.65);
    survey_speed_m_s_ = node.has_parameter("survey.speed") ? node.get_parameter("survey.speed").as_double() : node.declare_parameter<double>("survey.speed", 4.0);
    acceptance_radius_ = node.has_parameter("survey.acceptance_radius") ? node.get_parameter("survey.acceptance_radius").as_double() : node.declare_parameter<double>("survey.acceptance_radius", 2.5);

    // Compute optical parameters using production PhotogrammetryCalc
    production::SurveyParameters survey_params;
    survey_params.forward_overlap = forward_overlap_;
    survey_params.side_overlap = side_overlap_;
    survey_params.shutter_speed_s = 0.001; // 1/1000s
    survey_params.blur_budget_px = 0.5;

    grid_metrics_ = production::PhotogrammetryCalc::computeSurveyGridMetrics(
        camera_spec_, target_altitude_agl_, survey_params, survey_speed_m_s_
    );
    target_gsd_cm_ = grid_metrics_.gsd.gsd_mean_cm();

    generateSurveyPlan();

    RCLCPP_INFO(node.get_logger(),
                "[PhotogrammetrySurveyMission] Initialized: GSD=%.2f cm, StripSpacing=%.1f m, TriggerSpacing=%.1f m, Speed=%.1f m/s",
                grid_metrics_.gsd.gsd_mean_cm(), grid_metrics_.strip_spacing_m,
                grid_metrics_.trigger_distance_m, survey_speed_m_s_);
}

void PhotogrammetrySurveyMission::generateSurveyPlan()
{
    production::PlannerConfig cfg;
    cfg.strip_spacing_m = grid_metrics_.strip_spacing_m > 1.0 ? grid_metrics_.strip_spacing_m : 20.0;
    cfg.flight_altitude_agl_m = target_altitude_agl_;
    cfg.survey_speed_m_s = survey_speed_m_s_;
    cfg.turn_speed_m_s = 2.0;
    cfg.turn_overshoot_m = 6.0;

    auto opt = production::BoustrophedonPlanner::findOptimalOrientation(
        survey_polygon_, cfg.strip_spacing_m
    );
    optimal_angle_deg_ = opt.optimal_angle_deg;

    mission_waypoints_ = production::BoustrophedonPlanner::planMission(survey_polygon_, cfg);
    current_wp_idx_ = 0;
    plan_generated_ = true;
}

void PhotogrammetrySurveyMission::onActivate()
{
    // Resume mission execution
}

void PhotogrammetrySurveyMission::onDeactivate()
{
    // Pause mission execution
}

void PhotogrammetrySurveyMission::reset()
{
    current_wp_idx_ = 0;
    is_resuming_ = false;
}

bool PhotogrammetrySurveyMission::saveBreakpoint(const std::string& filepath, const SensorSnapshot& sensors)
{
    production::BreakpointState bp;
    bp.mission_id = "survey_mission";
    bp.current_waypoint_index = current_wp_idx_;
    bp.abort_position = sensors.position.cast<double>();
    bp.heading_rad = sensors.yaw;
    bp.flight_speed_m_s = sensors.velocity.norm();
    bp.progress_ratio = mission_waypoints_.empty() ? 0.0 :
                        static_cast<double>(current_wp_idx_) / static_cast<double>(mission_waypoints_.size());
    bp.abort_reason = "operator_pause_or_rtl";

    return bp.saveToFile(filepath);
}

bool PhotogrammetrySurveyMission::resumeFromBreakpoint(const std::string& filepath)
{
    auto bp_opt = production::BreakpointState::loadFromFile(filepath);
    if (!bp_opt.has_value()) {
        return false;
    }

    // Dynamic backtrack buffer: ensures SfM tie points are retained
    production::BacktrackParameters bt_params;
    bt_params.trigger_distance_m = grid_metrics_.trigger_distance_m > 1.0 ? grid_metrics_.trigger_distance_m : 10.0;
    bt_params.nominal_flight_speed_m_s = survey_speed_m_s_;
    bt_params.max_acceleration_m_s2 = 2.0;
    bt_params.settling_time_s = 1.5;

    backtrack_distance_m_ = bt_params.computeBacktrackDistance();
    current_wp_idx_ = bp_opt->current_waypoint_index;
    is_resuming_ = true;
    return true;
}

AutonomyCommand PhotogrammetrySurveyMission::update(const SensorSnapshot& sensors, float /*dt*/)
{
    AutonomyCommand cmd;

    if (mission_waypoints_.empty() || !plan_generated_) {
        generateSurveyPlan();
    }

    if (current_wp_idx_ >= mission_waypoints_.size()) {
        cmd.type = ControlType::Velocity;
        cmd.vector = Eigen::Vector3f::Zero();
        cmd.yaw_or_yaw_rate = 0.0f;
        cmd.status_message = "Survey mission complete. Holding station.";
        return cmd;
    }

    const auto& target_wp = mission_waypoints_[current_wp_idx_];
    Eigen::Vector3f target_pos = target_wp.position.cast<float>();

    // Distance to target waypoint in ENU
    Eigen::Vector3f delta = target_pos - sensors.position;
    float dist_xy = delta.head<2>().norm();
    float dist_z = std::abs(delta.z());

    // Check if waypoint reached
    if (dist_xy < static_cast<float>(acceptance_radius_) && dist_z < 2.0f) {
        current_wp_idx_++;
        if (current_wp_idx_ >= mission_waypoints_.size()) {
            cmd.type = ControlType::Velocity;
            cmd.vector = Eigen::Vector3f::Zero();
            cmd.yaw_or_yaw_rate = 0.0f;
            cmd.status_message = "Final survey waypoint reached. Mission complete.";
            return cmd;
        }
    }

    // Commanded guidance: Position goto or trajectory
    cmd.type = ControlType::Position;
    cmd.vector = target_pos;
    cmd.yaw_or_yaw_rate = static_cast<float>(target_wp.yaw_rad);
    cmd.status_message = "Survey Track: WP " + std::to_string(current_wp_idx_ + 1) + "/" +
                         std::to_string(mission_waypoints_.size()) +
                         " [" + target_wp.typeString() + "]" +
                         (target_wp.is_photo_trigger_zone ? " [TRIGGER ACTIVE]" : " [TRANSITION]");

    return cmd;
}

} // namespace enterprise
} // namespace px4_airsim_autonomy
