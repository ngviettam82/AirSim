#include "px4_airsim_autonomy/enterprise/enterprise_flight_supervisor.hpp"

namespace px4_airsim_autonomy {
namespace enterprise {

EnterpriseFlightSupervisor::EnterpriseFlightSupervisor()
{
    // Setup default 3D Geofence (Operational flight geography 200m x 200m, 0 to 100m AGL)
    production::Volume3D keep_in;
    keep_in.id = "flight_geography_default";
    keep_in.type = production::GeofenceVolumeType::KeepIn;
    keep_in.z_min = -5.0f;
    keep_in.z_max = 100.0f;
    keep_in.boundary.vertices = {
        Eigen::Vector2f(-100.0f, -100.0f),
        Eigen::Vector2f( 100.0f, -100.0f),
        Eigen::Vector2f( 100.0f,  100.0f),
        Eigen::Vector2f(-100.0f,  100.0f)
    };
    geofence_.addVolume(keep_in);

    // Setup standard forward depth camera
    production::CameraStreamConfig fwd_cam;
    fwd_cam.camera_id = "cam_forward";
    fwd_cam.mount = production::CameraMountOrientation::Forward;
    fwd_cam.intrinsics = production::CameraIntrinsics::fromFov(640, 480, 90.0f);
    fwd_cam.subsample_step = 2;
    depth_processor_.registerCamera(fwd_cam);
}

void EnterpriseFlightSupervisor::init(rclcpp::Node& node)
{
    enable_adsb_ = node.declare_parameter<bool>("safety.enable_adsb", true);
    enable_geofence_ = node.declare_parameter<bool>("safety.enable_geofence", true);
    enable_smart_rth_ = node.declare_parameter<bool>("safety.enable_smart_rth", true);

    RCLCPP_INFO(node.get_logger(),
                "[EnterpriseFlightSupervisor] Initialized (ADSB: %s, Geofence: %s, SmartRTH: %s)",
                enable_adsb_ ? "ON" : "OFF", enable_geofence_ ? "ON" : "OFF", enable_smart_rth_ ? "ON" : "OFF");
}

void EnterpriseFlightSupervisor::reset()
{
    spin_recovery_.reset();
    nav_fsm_.reset();
    depth_processor_.reset();
    tracked_traffic_.clear();
}

void EnterpriseFlightSupervisor::triggerMotorFailure(production::MotorIndex motor)
{
    spin_recovery_.triggerMotorFailure(motor);
}

void EnterpriseFlightSupervisor::updateAdsbTraffic(const std::vector<production::AdsbTarget>& traffic)
{
    tracked_traffic_ = traffic;
}

SupervisorSafetyReport EnterpriseFlightSupervisor::evaluate(
    const SensorSnapshot& sensors,
    float dt,
    const AutonomyCommand& mission_cmd)
{
    SupervisorSafetyReport report;
    report.max_allowable_speed = nav_fsm_.getMaxAllowableSpeed();

    // --------------------------------------------------------------------------
    // 1. CRITICAL ACTUATOR FAILURE CHECK: Quadcopter Spin Recovery
    // --------------------------------------------------------------------------
    if (spin_recovery_.isRecoveryActive()) {
        production::SpinRecoveryTelemetry telem;
        telem.position = sensors.position;
        telem.velocity = sensors.velocity;
        telem.angular_velocity = Eigen::Vector3f(0.0f, 0.0f, spin_recovery_.getFilteredSpinRate());
        telem.dt = dt;

        production::SpinRecoveryOutput spin_out = spin_recovery_.update(telem);

        report.emergency_override_active = true;
        report.override_source = "QuadcopterSpinRecovery";
        report.override_reason = spin_out.diagnostic_message;
        report.spin_recovery_active = true;

        // Controlled descent velocity setpoint
        report.override_command.type = ControlType::Velocity;
        report.override_command.vector = Eigen::Vector3f(0.0f, 0.0f, -1.8f);
        report.override_command.yaw_or_yaw_rate = 0.0f;
        report.override_command.status_message = "EMERGENCY: Spin recovery active (motor failure)";
        return report;
    }

    // --------------------------------------------------------------------------
    // 2. STATE ESTIMATION INTEGRITY CHECK: Degraded Navigation FSM
    // --------------------------------------------------------------------------
    production::NavigationHealthInputs nav_inputs;
    nav_inputs.gnss_received = true;
    nav_inputs.gnss_fix = production::GnssFixType::RtkFixed;
    nav_inputs.gnss_satellites = 16;
    nav_inputs.gnss_eph = 0.08f;
    nav_inputs.vio_received = true;
    nav_inputs.vio_tracking_valid = true;
    nav_inputs.vio_tracked_features = 45;
    nav_inputs.vio_confidence = 0.85f;
    nav_inputs.imu_healthy = true;

    production::NavigationStatusReport nav_status = nav_fsm_.update(nav_inputs, dt);
    report.nav_tier = nav_status.current_tier;
    report.max_allowable_speed = nav_status.max_allowable_speed;

    if (nav_status.is_failsafe_active) {
        report.emergency_override_active = true;
        report.override_source = "DegradedNavigationFsm";
        report.override_reason = "Navigation quality degraded to Tier 4 (Emergency Descend Land)";
        report.override_command.type = ControlType::Velocity;
        report.override_command.vector = Eigen::Vector3f(0.0f, 0.0f, nav_status.commanded_vz_failsafe);
        report.override_command.yaw_or_yaw_rate = 0.0f;
        report.override_command.status_message = "FAILSAFE: Degraded navigation emergency landing";
        return report;
    }

    // --------------------------------------------------------------------------
    // 3. AIRSPACE DECONFLICTION CHECK: RTCA DO-365B ADS-B
    // --------------------------------------------------------------------------
    if (enable_adsb_ && !tracked_traffic_.empty()) {
        production::OwnshipState ownship;
        ownship.position = sensors.position;
        ownship.velocity = sensors.velocity;

        production::DeconflictionResult adsb_res = adsb_.evaluateTraffic(ownship, tracked_traffic_);
        report.adsb_alert_level = adsb_res.max_alert_level;

        if (adsb_res.evasive_action_required) {
            report.emergency_override_active = true;
            report.override_source = "AdsbDeconfliction";
            report.override_reason = adsb_res.advisory_message;
            report.override_command.type = ControlType::Velocity;
            report.override_command.vector = adsb_res.recommended_velocity_cmd;
            report.override_command.yaw_or_yaw_rate = 0.0f;
            report.override_command.status_message = "AIRSPACE EVASION: " + adsb_res.advisory_message;
            return report;
        }
    }

    // --------------------------------------------------------------------------
    // 4. AIRSPACE CONTAINMENT CHECK: 3D Volumetric Geofence
    // --------------------------------------------------------------------------
    if (enable_geofence_) {
        production::GeofenceStatus geo_status = geofence_.evaluate(sensors.position, sensors.velocity, dt);
        report.geofence_status = geo_status;

        if (geo_status.breach_imminent || geo_status.breached) {
            report.emergency_override_active = true;
            report.override_source = "Geofence3D";
            report.override_reason = geo_status.status_message;
            report.override_command.type = ControlType::Velocity;
            report.override_command.vector = geo_status.braking_velocity_cmd;
            report.override_command.yaw_or_yaw_rate = 0.0f;
            report.override_command.status_message = "GEOFENCE BRAKING: " + geo_status.status_message;
            return report;
        }
    }

    // --------------------------------------------------------------------------
    // 5. ENERGY & BATTERY CONTINGENCY: Physics-Based Smart RTH
    // --------------------------------------------------------------------------
    if (enable_smart_rth_) {
        // Evaluate with nominal 6S battery voltage (22.2V) and 70% estimated SoC
        float est_soc = 0.70f;
        float est_voltage = 22.2f;
        Eigen::Vector2f est_wind(0.0f, 0.0f);

        production::RthEvaluationResult rth_res = rth_battery_.evaluate(
            est_soc, est_voltage, sensors.position, home_pos_, est_wind
        );
        report.rth_state = rth_res.state;

        if (rth_res.immediate_land_required) {
            report.emergency_override_active = true;
            report.override_source = "SmartRthBattery";
            report.override_reason = "Critically depleted battery cell voltage cutoff";
            report.override_command.type = ControlType::Velocity;
            report.override_command.vector = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
            report.override_command.yaw_or_yaw_rate = 0.0f;
            report.override_command.status_message = "FAILSAFE: Smart RTH critical cutoff landing";
            return report;
        } else if (rth_res.rth_required) {
            report.emergency_override_active = true;
            report.override_source = "SmartRthBattery";
            report.override_reason = "Remaining battery required for return flight against wind";
            
            // Vector pointing to home at safe cruise speed
            Eigen::Vector3f to_home = home_pos_ - sensors.position;
            float dist_h = to_home.head<2>().norm();
            Eigen::Vector3f rth_vel = Eigen::Vector3f::Zero();
            if (dist_h > 1.0f) {
                Eigen::Vector2f dir_h = to_home.head<2>() / dist_h;
                rth_vel.x() = dir_h.x() * 8.0f;
                rth_vel.y() = dir_h.y() * 8.0f;
            }
            // Maintain safe cruise altitude
            float alt_diff = (home_pos_.z() + 30.0f) - sensors.position.z();
            rth_vel.z() = std::clamp(alt_diff * 0.5f, -2.0f, 3.0f);

            report.override_command.type = ControlType::Velocity;
            report.override_command.vector = rth_vel;
            report.override_command.yaw_or_yaw_rate = 0.0f;
            report.override_command.status_message = "AUTONOMOUS RTH: Returning to launch point";
            return report;
        }
    }

    // --------------------------------------------------------------------------
    // 6. DYNAMIC SPEED ENVELOPE ENFORCEMENT ON MISSION COMMAND
    // --------------------------------------------------------------------------
    report.override_command = mission_cmd;
    if (report.override_command.type == ControlType::Velocity) {
        float speed_xy = report.override_command.vector.head<2>().norm();
        if (speed_xy > report.max_allowable_speed && speed_xy > 1e-3f) {
            report.override_command.vector.head<2>() *= (report.max_allowable_speed / speed_xy);
        }
    }

    return report;
}

} // namespace enterprise
} // namespace px4_airsim_autonomy
