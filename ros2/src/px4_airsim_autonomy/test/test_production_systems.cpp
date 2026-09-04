#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "px4_airsim_autonomy/production/smart_rth_battery.hpp"
#include "px4_airsim_autonomy/production/geofence_3d.hpp"
#include "px4_airsim_autonomy/production/adsb_deconfliction.hpp"
#include "px4_airsim_autonomy/production/photogrammetry_calc.hpp"
#include "px4_airsim_autonomy/production/boustrophedon_planner.hpp"
#include "px4_airsim_autonomy/production/mission_continuity.hpp"
#include "px4_airsim_autonomy/production/dynamic_stopping_bubble.hpp"
#include "px4_airsim_autonomy/production/safe_flight_corridor.hpp"
#include "px4_airsim_autonomy/production/quadcopter_spin_recovery.hpp"
#include "px4_airsim_autonomy/production/multi_camera_depth.hpp"
#include "px4_airsim_autonomy/production/degraded_navigation_fsm.hpp"

#include <cmath>
#include <vector>

using namespace px4_airsim_autonomy::production;

// ==============================================================================
// 1. SMART RTH BATTERY CONTINGENCY TESTS
// ==============================================================================
TEST(ProductionSystemsTest, SmartRthBatteryWindTriangleAndEnergy) {
    SmartRthBattery rth;

    // Wind triangle groundspeed calculation
    Eigen::Vector2f to_home(0.0f, 1.0f); // Due North
    Eigen::Vector2f calm_wind(0.0f, 0.0f);
    auto res_calm = rth.computeWindTriangle(to_home, calm_wind, 10.0f);
    EXPECT_NEAR(res_calm.groundspeed, 10.0f, 1e-4f);
    EXPECT_FALSE(res_calm.course_unachievable);

    // Tailwind: wind blowing North at 5 m/s -> groundspeed = 15 m/s
    Eigen::Vector2f tailwind(0.0f, 5.0f);
    auto res_tail = rth.computeWindTriangle(to_home, tailwind, 10.0f);
    EXPECT_NEAR(res_tail.groundspeed, 15.0f, 1e-4f);
    EXPECT_GT(res_tail.parallel_wind, 4.9f);

    // Headwind: wind blowing South at 5 m/s -> groundspeed = 5 m/s
    Eigen::Vector2f headwind(0.0f, -5.0f);
    auto res_head = rth.computeWindTriangle(to_home, headwind, 10.0f);
    EXPECT_NEAR(res_head.groundspeed, 5.0f, 1e-4f);
    EXPECT_LT(res_head.parallel_wind, -4.9f);

    // Crosswind: wind East 6 m/s, airspeed 10 m/s -> v_g = sqrt(10^2 - 6^2) = 8 m/s
    Eigen::Vector2f crosswind(6.0f, 0.0f);
    auto res_cross = rth.computeWindTriangle(to_home, crosswind, 10.0f);
    EXPECT_NEAR(res_cross.groundspeed, 8.0f, 1e-3f);
    EXPECT_NEAR(res_cross.crosswind, 6.0f, 1e-3f);

    // Energy integration & RTH state evaluation
    Eigen::Vector3f curr_pos(0.0f, 1000.0f, 50.0f); // 1km North
    Eigen::Vector3f home_pos(0.0f, 0.0f, 0.0f);
    Eigen::Vector2f wind(0.0f, 0.0f);

    // High battery (SoC 80%, 4.0V/cell -> 24.0V pack) -> Nominal
    auto eval_high = rth.evaluate(0.80f, 24.0f, curr_pos, home_pos, wind);
    EXPECT_EQ(eval_high.state, RthBatteryState::Nominal);
    EXPECT_FALSE(eval_high.rth_required);
    EXPECT_GT(eval_high.energy_breakdown.e_cruise_joules, 0.0f);

    // Marginal battery matching required return energy -> Critical RTH
    float req_soc = eval_high.required_soc;
    auto eval_crit = rth.evaluate(req_soc, 21.6f, curr_pos, home_pos, wind);
    EXPECT_EQ(eval_crit.state, RthBatteryState::CriticalRthTrigger);
    EXPECT_TRUE(eval_crit.rth_required);

    // Depleted cell voltage (3.2V/cell -> 19.2V pack) -> Emergency cutoff land
    auto eval_land = rth.evaluate(0.03f, 19.2f, curr_pos, home_pos, wind);
    EXPECT_EQ(eval_land.state, RthBatteryState::EmergencyCutoffLand);
    EXPECT_TRUE(eval_land.immediate_land_required);
}

// ==============================================================================
// 2. 3D VOLUMETRIC GEOFENCE TESTS
// ==============================================================================
TEST(ProductionSystemsTest, Geofence3DContainmentAndBraking) {
    Geofence3D fence;

    // Create 100m x 100m Keep-In operational volume, height 0 to 50m
    Volume3D keep_in;
    keep_in.id = "flight_geography";
    keep_in.type = GeofenceVolumeType::KeepIn;
    keep_in.z_min = 0.0f;
    keep_in.z_max = 50.0f;
    keep_in.boundary.vertices = {
        Eigen::Vector2f(-50.0f, -50.0f),
        Eigen::Vector2f( 50.0f, -50.0f),
        Eigen::Vector2f( 50.0f,  50.0f),
        Eigen::Vector2f(-50.0f,  50.0f)
    };
    fence.addVolume(keep_in);

    // Inside and stationary -> Safe
    auto status_center = fence.evaluate(
        Eigen::Vector3f(0.0f, 0.0f, 20.0f),
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        0.05f
    );
    EXPECT_TRUE(status_center.is_safe);
    EXPECT_FALSE(status_center.breached);
    EXPECT_FALSE(status_center.breach_imminent);

    // Stopping distance formula: d_stop = v * t_reaction + v^2 / (2 * a_max)
    // For v = 10 m/s, t_reac = 0.3s, a_max = 2.5 m/s^2:
    // d_stop = 10 * 0.3 + 100 / 5 = 3.0 + 20.0 = 23.0 m
    float d_stop_10 = fence.computeStoppingDistance(10.0f);
    EXPECT_NEAR(d_stop_10, 23.0f, 0.05f);

    // Moving at 10 m/s towards East wall at x = 35m (15m clearance < 19.67m d_stop)
    // Must trigger breach imminent and maximum braking vector
    auto status_fast = fence.evaluate(
        Eigen::Vector3f(35.0f, 0.0f, 20.0f),
        Eigen::Vector3f(10.0f, 0.0f, 0.0f),
        0.05f
    );
    EXPECT_FALSE(status_fast.is_safe);
    EXPECT_TRUE(status_fast.breach_imminent);
    EXPECT_LT(status_fast.braking_velocity_cmd.x(), 10.0f); // Deceleration applied
}

// ==============================================================================
// 3. RTCA DO-365B ADS-B DECONFLICTION TESTS
// ==============================================================================
TEST(ProductionSystemsTest, AdsbDeconflictionDO365B) {
    AdsbDeconfliction diconf;

    OwnshipState ownship;
    ownship.position = Eigen::Vector3f(0.0f, 0.0f, 100.0f);
    ownship.velocity = Eigen::Vector3f(10.0f, 0.0f, 0.0f); // East at 10 m/s

    // Distant traffic (10km away) -> Level 0 Normal
    AdsbTarget target_distant;
    target_distant.icao_address = 0x112233;
    target_distant.position = Eigen::Vector3f(10000.0f, 0.0f, 1000.0f);
    target_distant.velocity = Eigen::Vector3f(-100.0f, 0.0f, 0.0f);
    auto eval_distant = diconf.evaluateTraffic(ownship, {target_distant});
    EXPECT_EQ(eval_distant.max_alert_level, DaaAlertLevel::Level0_Normal);

    // Head-on collision course within 20s (tau_mod < 25s, co-altitude)
    // Level 4 Warning Alert / Well Clear Violation -> Automated emergency dive
    AdsbTarget target_threat;
    target_threat.icao_address = 0xAABBCC;
    target_threat.position = Eigen::Vector3f(800.0f, 0.0f, 100.0f);
    target_threat.velocity = Eigen::Vector3f(-40.0f, 0.0f, 0.0f); // Closing rate = 50 m/s -> 16s to collision
    auto eval_threat = diconf.evaluateTraffic(ownship, {target_threat});
    EXPECT_EQ(eval_threat.max_alert_level, DaaAlertLevel::Level4_Warning);
    EXPECT_LT(eval_threat.recommended_velocity_cmd.z(), -2.0f); // Emergency dive commanded
}

// ==============================================================================
// 4. PHOTOGRAMMETRY & GSD CALCULATION TESTS
// ==============================================================================
TEST(ProductionSystemsTest, PhotogrammetryCalculations) {
    auto p4p = CameraSpec::DJI_Phantom4_Pro();
    EXPECT_TRUE(p4p.isValid());

    // 50m AGL altitude
    double alt = 50.0;
    auto gsd = PhotogrammetryCalc::computeGsd(p4p, alt);
    // GSD_h = (50 * 13.2) / (8.8 * 5472) = 1.37 cm/px
    EXPECT_NEAR(gsd.gsd_h_cm(), 1.3706, 0.05);
    EXPECT_NEAR(gsd.footprint_width_m, 75.0, 0.1);
    EXPECT_NEAR(gsd.footprint_height_m, 50.0, 0.1);

    // Overlap: 80% forward, 70% side
    SurveyParameters survey_params;
    survey_params.forward_overlap = 0.80;
    survey_params.side_overlap = 0.70;
    survey_params.shutter_speed_s = 0.001; // 1/1000s
    survey_params.blur_budget_px = 0.5;

    auto metrics = PhotogrammetryCalc::computeSurveyGridMetrics(p4p, alt, survey_params, 5.0);
    // D_trigger = 50.0 * (1 - 0.8) = 10.0m
    EXPECT_NEAR(metrics.trigger_distance_m, 10.0, 0.05);
    // S_strip = 75.0 * (1 - 0.7) = 22.5m
    EXPECT_NEAR(metrics.strip_spacing_m, 22.5, 0.05);
    // v_max = 0.5 * 0.0137 / 0.001 = 6.85 m/s
    EXPECT_NEAR(metrics.max_speed_blur_m_s, 6.85, 0.1);

    // Smart Oblique Capture (SOC) 5-way sequence
    auto soc = PhotogrammetryCalc::generateSocGimbalSequence(45.0);
    ASSERT_EQ(soc.size(), 5u);
    EXPECT_EQ(soc[0].direction, SocGimbalPose::Direction::Nadir);
    EXPECT_NEAR(soc[0].pitch_deg, 0.0, 1e-3);
    EXPECT_EQ(soc[1].direction, SocGimbalPose::Direction::Forward);
    EXPECT_NEAR(soc[1].pitch_deg, 45.0, 1e-3);
}

// ==============================================================================
// 5. BOUSTROPHEDON PLANNER TESTS
// ==============================================================================
TEST(ProductionSystemsTest, BoustrophedonPlannerCoverage) {
    // 100m x 60m rectangular survey field
    std::vector<Eigen::Vector2d> polygon = {
        {0.0, 0.0},
        {100.0, 0.0},
        {100.0, 60.0},
        {0.0, 60.0}
    };

    PlannerConfig config;
    config.strip_spacing_m = 20.0;
    config.flight_altitude_agl_m = 50.0;
    config.survey_speed_m_s = 5.0;

    auto waypoints = BoustrophedonPlanner::planMission(polygon, config);
    EXPECT_FALSE(waypoints.empty());
    EXPECT_GT(waypoints.size(), 4u);
}

// ==============================================================================
// 6. MISSION CONTINUITY & BACKTRACK BUFFER TESTS
// ==============================================================================
TEST(ProductionSystemsTest, MissionContinuitySerializationAndBacktrack) {
    BreakpointState state;
    state.mission_id = "survey_alpha";
    state.current_waypoint_index = 12;
    state.strip_index = 2;
    state.abort_position = Eigen::Vector3d(50.0, 40.0, 30.0);
    state.progress_ratio = 0.65;
    state.abort_reason = "low_battery_rtl";

    // JSON round-trip serialization
    std::string json_str = state.toJson();
    EXPECT_NE(json_str.find("survey_alpha"), std::string::npos);

    auto restored = BreakpointState::fromJson(json_str);
    ASSERT_TRUE(restored.has_value());
    EXPECT_EQ(restored->current_waypoint_index, 12u);
    EXPECT_NEAR(restored->abort_position.x(), 50.0, 1e-3);
    EXPECT_EQ(restored->abort_reason, "low_battery_rtl");

    // Dynamic backtrack buffer calculation:
    // D_backtrack = max(2 * D_trig, v^2 / (2*a) + v * t_settle)
    // For D_trig = 10m, v = 5m/s, a = 2m/s^2, t_settle = 2s:
    // 2 * 10 = 20m; 25/4 + 10 = 16.25m -> max = 20m
    BacktrackParameters bt_params;
    bt_params.trigger_distance_m = 10.0;
    bt_params.nominal_flight_speed_m_s = 5.0;
    bt_params.max_acceleration_m_s2 = 2.0;
    bt_params.settling_time_s = 2.0;

    double d_backtrack = bt_params.computeBacktrackDistance();
    EXPECT_NEAR(d_backtrack, 20.0, 1e-3);
}

// ==============================================================================
// 7. DYNAMIC STOPPING BUBBLE & BRAKING ELLIPSOID TESTS
// ==============================================================================
TEST(ProductionSystemsTest, DynamicStoppingBubblePhysics) {
    DynamicStoppingBubbleConfig cfg;
    cfg.a_max = 4.0f;
    cfg.t_latency = 0.15f;
    cfg.d_margin = 0.8f;
    DynamicStoppingBubble bubble(cfg);

    // Quadratic stopping distance: d_stop(v) = v^2 / (2 * a_max) + v * t_latency + d_margin
    // v = 4.0 m/s -> 16 / 8 + 0.6 + 0.8 = 3.4 m
    float d_stop = bubble.computeStoppingDistance(4.0f);
    EXPECT_NEAR(d_stop, 3.4f, 1e-4f);

    // Emergency braking vector: a_brake = -a_max * (v / ||v||)
    Eigen::Vector3f vel(4.0f, 0.0f, 0.0f);
    Eigen::Vector3f a_brake = bubble.computeBrakingAcceleration(vel);
    EXPECT_NEAR(a_brake.x(), -4.0f, 1e-4f);
    EXPECT_NEAR(a_brake.y(), 0.0f, 1e-4f);

    // Update bubble with vehicle at (0, 0, 5) moving at 4 m/s in +X
    bubble.update(Eigen::Vector3f(0.0f, 0.0f, 5.0f), vel);

    // Point 2m ahead is inside bubble (violation)
    Eigen::Vector3f pt_inside(2.0f, 0.0f, 5.0f);
    EXPECT_TRUE(bubble.isPointInside(pt_inside));
    EXPECT_LT(bubble.computeNormalizedDistance(pt_inside), 1.0f);

    // Point 10m ahead is outside bubble
    Eigen::Vector3f pt_outside(10.0f, 0.0f, 5.0f);
    EXPECT_FALSE(bubble.isPointInside(pt_outside));
    EXPECT_GT(bubble.computeNormalizedDistance(pt_outside), 1.0f);
}

// ==============================================================================
// 8. SAFE FLIGHT CORRIDOR & TRANSVERSE JERK TESTS
// ==============================================================================
TEST(ProductionSystemsTest, SafeFlightCorridorAndJerkBounds) {
    SafeFlightCorridor sfc;

    std::vector<Eigen::Vector3f> waypoints = {
        Eigen::Vector3f(0.0f, 0.0f, 2.0f),
        Eigen::Vector3f(5.0f, 0.0f, 2.0f)
    };

    auto slices = sfc.generateCorridor(waypoints);
    ASSERT_FALSE(slices.empty());
    EXPECT_GT(slices[0].hyperplanes.size(), 4u);
    EXPECT_TRUE(slices[0].contains(Eigen::Vector3f(1.0f, 0.0f, 2.0f)));

    // Minimum-jerk polynomial (T = 4.0s yields max jerk ~ 4.69 m/s^3 <= 10.0 m/s^3)
    auto smooth_seg = PolynomialSegment3D::solveMinimumJerkQuintic(
        waypoints[0], Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
        waypoints[1], Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
        4.0f
    );
    auto jerk_report = sfc.checkJerkLimits(smooth_seg, 10.0f);
    EXPECT_TRUE(jerk_report.satisfies_limit);
    EXPECT_LE(jerk_report.max_transverse_jerk, 10.0f);

    // Aggressive maneuver (5m in 2.0s yields jerk > 30 m/s^3) -> Must flag violation
    auto aggressive_seg = PolynomialSegment3D::solveMinimumJerkQuintic(
        waypoints[0], Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
        waypoints[1], Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
        2.0f
    );
    auto agg_report = sfc.checkJerkLimits(aggressive_seg, 10.0f);
    EXPECT_FALSE(agg_report.satisfies_limit);
}

// ==============================================================================
// 9. QUADCOPTER SPIN-DESCENT SINGLE MOTOR RECOVERY TESTS
// ==============================================================================
TEST(ProductionSystemsTest, QuadcopterSpinRecovery) {
    QuadcopterSpinRecovery recovery;
    recovery.triggerMotorFailure(QuadcopterSpinRecovery::MotorIndex::Motor1_FrontRight);
    EXPECT_TRUE(recovery.isRecoveryActive());

    // Step simulation through spin recovery controller
    Eigen::Vector3f pos(0.0f, 0.0f, 20.0f);
    Eigen::Vector3f vel(0.5f, 0.2f, -1.0f);
    float dt = 0.01f;

    for (int i = 0; i < 50; ++i) {
        recovery.update(pos, vel, 25.0f, dt);
    }

    auto thrusts = recovery.getActiveThrustCommands();
    // Failed motor 0 must receive 0 thrust
    EXPECT_NEAR(thrusts[0], 0.0f, 1e-4f);
    // Surviving motors (1, 2, 3) must receive positive thrust for controlled descent
    EXPECT_GT(thrusts[1], 0.0f);
    EXPECT_GT(thrusts[2], 0.0f);
    EXPECT_GT(thrusts[3], 0.0f);
}

// ==============================================================================
// 10. MULTI-CAMERA 360 DEPTH PROCESSOR TESTS
// ==============================================================================
TEST(ProductionSystemsTest, MultiCameraDepthProcessing) {
    MultiCameraDepthProcessor processor;

    CameraStreamConfig fwd_cfg;
    fwd_cfg.camera_id = "cam_forward";
    fwd_cfg.mount = CameraMountOrientation::Forward;
    fwd_cfg.intrinsics = CameraIntrinsics::fromFov(640, 480, 90.0f);
    fwd_cfg.subsample_step = 2;
    processor.registerCamera(fwd_cfg);

    CameraStreamConfig down_cfg;
    down_cfg.camera_id = "cam_downward";
    down_cfg.mount = CameraMountOrientation::Downward;
    down_cfg.intrinsics = CameraIntrinsics::fromFov(640, 480, 90.0f);
    down_cfg.subsample_step = 2;
    processor.registerCamera(down_cfg);

    EXPECT_EQ(processor.getRegisteredCameraCount(), 2u);

    // Ingest synthetic depth buffer (obstacle 3.5m ahead)
    std::vector<float> fwd_depth(640 * 480, 20.0f);
    fwd_depth[240 * 640 + 320] = 3.5f;
    processor.ingestDepthFrame("cam_forward", fwd_depth.data(), 640, 480, 640, 1000);

    // Ingest downward buffer (ground 1.8m down)
    std::vector<float> down_depth(640 * 480, 50.0f);
    down_depth[240 * 640 + 320] = 1.8f;
    processor.ingestDepthFrame("cam_downward", down_depth.data(), 640, 480, 640, 1000);

    auto clearance = processor.getClearance();
    EXPECT_NEAR(clearance.min_distance, 1.8f, 0.05f);
    EXPECT_NEAR(clearance.front, 3.5f, 0.05f);
    EXPECT_NEAR(clearance.lower, 1.8f, 0.05f);
}

// ==============================================================================
// 11. DEGRADED NAVIGATION FSM TESTS
// ==============================================================================
TEST(ProductionSystemsTest, DegradedNavigationFsmTransitions) {
    DegradedNavigationFsm fsm;
    EXPECT_EQ(fsm.getCurrentTier(), NavigationQualityTier::Tier0_RtkFixed);
    EXPECT_NEAR(fsm.getMaxAllowableSpeed(), 15.0f, 1e-3f);

    // Inject high GPS innovation test ratio outlier (ratio = 2.5 > 1.0)
    // Must immediately demote from RTK to VIO
    NavigationHealthInputs in1;
    in1.gnss_received = true;
    in1.gnss_fix = GnssFixType::RtkFixed;
    in1.gnss_satellites = 18;
    in1.gnss_eph = 0.05f;
    in1.gnss_innovation_test_ratio = 2.5f; // Failure
    in1.vio_received = true;
    in1.vio_tracking_valid = true;
    in1.vio_tracked_features = 80;
    in1.vio_confidence = 0.9f;
    in1.vio_innovation_test_ratio = 0.1f;
    fsm.update(in1, 0.05f);
    EXPECT_EQ(fsm.getCurrentTier(), NavigationQualityTier::Tier2_VioLioOdometry);
    EXPECT_NEAR(fsm.getMaxAllowableSpeed(), 5.0f, 1e-3f);

    // Invalidate VIO tracking -> Demote to Dead Reckoning
    NavigationHealthInputs in2 = in1;
    in2.vio_tracking_valid = false;
    in2.vio_tracked_features = 0;
    in2.vio_confidence = 0.0f;
    fsm.update(in2, 0.05f);
    EXPECT_EQ(fsm.getCurrentTier(), NavigationQualityTier::Tier3_InertialDragDeadReckoning);
    EXPECT_NEAR(fsm.getMaxAllowableSpeed(), 1.5f, 1e-3f);

    // Exceed dead reckoning time budget (16s > 15s) -> Emergency descend and land
    fsm.update(in2, 16.0f);
    EXPECT_EQ(fsm.getCurrentTier(), NavigationQualityTier::Tier4_EmergencyDescendLand);
    EXPECT_NEAR(fsm.getMaxAllowableSpeed(), 0.0f, 1e-3f);
}

