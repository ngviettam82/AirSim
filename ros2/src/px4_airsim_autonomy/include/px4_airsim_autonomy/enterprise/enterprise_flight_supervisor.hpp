#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include "px4_airsim_autonomy/types.hpp"
#include "px4_airsim_autonomy/production/smart_rth_battery.hpp"
#include "px4_airsim_autonomy/production/geofence_3d.hpp"
#include "px4_airsim_autonomy/production/adsb_deconfliction.hpp"
#include "px4_airsim_autonomy/production/degraded_navigation_fsm.hpp"
#include "px4_airsim_autonomy/production/quadcopter_spin_recovery.hpp"
#include "px4_airsim_autonomy/production/multi_camera_depth.hpp"

namespace px4_airsim_autonomy {
namespace enterprise {

/**
 * @brief Consolidated safety status from all enterprise flight supervisors.
 */
struct SupervisorSafetyReport {
    bool emergency_override_active{false};
    std::string override_source;
    std::string override_reason;
    AutonomyCommand override_command;

    // Subsystem status summaries
    production::RthBatteryState rth_state{production::RthBatteryState::Nominal};
    production::GeofenceStatus geofence_status;
    production::DaaAlertLevel adsb_alert_level{production::DaaAlertLevel::Level0_Normal};
    production::NavigationQualityTier nav_tier{production::NavigationQualityTier::Tier0_RtkFixed};
    bool spin_recovery_active{false};
    float max_allowable_speed{15.0f};
};

/**
 * @brief Enterprise Flight Supervisor.
 * 
 * Integrates 3D Volumetric Geofencing, Physics-Based Smart RTH, RTCA DO-365B ADS-B
 * Deconfliction, 5-Tier Degraded Navigation FSM, and Mueller & D'Andrea Spin Recovery.
 * Runs on every 50 Hz control loop cycle to provide real-time preemptive safety overrides.
 */
class EnterpriseFlightSupervisor {
public:
    EnterpriseFlightSupervisor();
    ~EnterpriseFlightSupervisor() = default;

    void init(rclcpp::Node& node);
    void reset();

    /**
     * @brief Ingest sensor telemetry and evaluate real-time safety constraints.
     * @param sensors Current vehicle state and environment perception
     * @param dt Elapsed seconds since last control step
     * @param mission_cmd Planned command from the active mission
     * @return SupervisorSafetyReport Full status report with preemptive override if triggered
     */
    SupervisorSafetyReport evaluate(
        const SensorSnapshot& sensors,
        float dt,
        const AutonomyCommand& mission_cmd
    );

    // Subsystem accessors
    production::Geofence3D& getGeofence() { return geofence_; }
    production::SmartRthBattery& getSmartRth() { return rth_battery_; }
    production::AdsbDeconfliction& getAdsb() { return adsb_; }
    production::DegradedNavigationFsm& getNavFsm() { return nav_fsm_; }
    production::QuadcopterSpinRecovery& getSpinRecovery() { return spin_recovery_; }
    production::MultiCameraDepthProcessor& getDepthProcessor() { return depth_processor_; }

    void triggerMotorFailure(production::MotorIndex motor);
    void updateAdsbTraffic(const std::vector<production::AdsbTarget>& traffic);
    void setHomePosition(const Eigen::Vector3f& home_pos) { home_pos_ = home_pos; }

private:
    production::Geofence3D geofence_;
    production::SmartRthBattery rth_battery_;
    production::AdsbDeconfliction adsb_;
    production::DegradedNavigationFsm nav_fsm_;
    production::QuadcopterSpinRecovery spin_recovery_;
    production::MultiCameraDepthProcessor depth_processor_;

    Eigen::Vector3f home_pos_{Eigen::Vector3f::Zero()};
    std::vector<production::AdsbTarget> tracked_traffic_;

    // Safety thresholds
    float default_speed_limit_{15.0f};
    bool enable_adsb_{true};
    bool enable_geofence_{true};
    bool enable_smart_rth_{true};
};

} // namespace enterprise
} // namespace px4_airsim_autonomy
