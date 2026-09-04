#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "px4_airsim_autonomy/types.hpp"
#include "px4_airsim_autonomy/algorithm_factory.hpp"
#include "px4_airsim_autonomy/enterprise/photogrammetry_survey_mission.hpp"
#include "px4_airsim_autonomy/enterprise/dynamic_avoidance_mission.hpp"
#include "px4_airsim_autonomy/enterprise/enterprise_flight_supervisor.hpp"
#include "px4_airsim_autonomy/production/dynamic_stopping_bubble.hpp"
#include "px4_airsim_autonomy/production/safe_flight_corridor.hpp"
#include "px4_airsim_autonomy/production/quadcopter_spin_recovery.hpp"

using namespace px4_airsim_autonomy;
using namespace px4_airsim_autonomy::production;
using namespace px4_airsim_autonomy::enterprise;

class AutonomyAlgorithmsTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        test_node_ = std::make_shared<rclcpp::Node>("test_autonomy_node");
    }

    void TearDown() override {
        test_node_.reset();
    }

    std::shared_ptr<rclcpp::Node> test_node_;
};

// ==============================================================================
// 1. FACTORY CREATION TESTS
// ==============================================================================
TEST_F(AutonomyAlgorithmsTest, FactoryCreation) {
    auto alg1 = AlgorithmFactory::createAlgorithm("photogrammetry_survey");
    ASSERT_NE(alg1, nullptr);
    EXPECT_EQ(alg1->getName(), "photogrammetry_survey");

    auto alg2 = AlgorithmFactory::createAlgorithm("dynamic_avoidance");
    ASSERT_NE(alg2, nullptr);
    EXPECT_EQ(alg2->getName(), "dynamic_avoidance");

    // Aliases
    auto alg3 = AlgorithmFactory::createAlgorithm("scanning_patrol");
    ASSERT_NE(alg3, nullptr);
    EXPECT_EQ(alg3->getName(), "photogrammetry_survey");

    auto alg4 = AlgorithmFactory::createAlgorithm("obstacle_avoidance");
    ASSERT_NE(alg4, nullptr);
    EXPECT_EQ(alg4->getName(), "dynamic_avoidance");

    // Blank / None returns nullptr for safe standby
    auto alg_blank = AlgorithmFactory::createAlgorithm("");
    EXPECT_EQ(alg_blank, nullptr);

    auto alg_none = AlgorithmFactory::createAlgorithm("none");
    EXPECT_EQ(alg_none, nullptr);

    auto list = AlgorithmFactory::getAvailableAlgorithms();
    EXPECT_EQ(list.size(), 2u);
}

// ==============================================================================
// 2. PHOTOGRAMMETRY SURVEY MISSION TESTS
// ==============================================================================
TEST_F(AutonomyAlgorithmsTest, PhotogrammetrySurveyMissionLogic) {
    PhotogrammetrySurveyMission survey;
    survey.init(*test_node_);
    survey.onActivate();

    EXPECT_GT(survey.getTotalWaypoints(), 4u);
    EXPECT_GT(survey.getTargetGsdCm(), 0.5);

    SensorSnapshot sensors;
    sensors.position = Eigen::Vector3f(0.0f, 0.0f, 30.0f);
    sensors.velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    sensors.dt = 0.05f;

    auto cmd = survey.update(sensors, 0.05f);
    EXPECT_EQ(cmd.type, ControlType::Position);
    EXPECT_GT(cmd.vector.z(), 20.0f); // Altitude setpoint maintained

    // Waypoint progression
    size_t initial_wp = survey.getCurrentWaypointIndex();
    sensors.position = cmd.vector; // Arrived at waypoint
    auto cmd2 = survey.update(sensors, 0.05f);
    EXPECT_GE(survey.getCurrentWaypointIndex(), initial_wp);
}

// ==============================================================================
// 3. DYNAMIC AVOIDANCE MISSION TESTS
// ==============================================================================
TEST_F(AutonomyAlgorithmsTest, DynamicAvoidanceMissionLogic) {
    DynamicAvoidanceMission avoidance;
    avoidance.init(*test_node_);
    avoidance.onActivate();

    SensorSnapshot sensors;
    sensors.dt = 0.05f;
    sensors.position = Eigen::Vector3f(0.0f, 0.0f, 10.0f);
    sensors.velocity = Eigen::Vector3f(2.0f, 0.0f, 0.0f);

    // 1. Clear corridor ahead -> Forward cruise
    sensors.min_depth_distance = 25.0f;
    sensors.center_sector_distance = 25.0f;
    sensors.left_sector_distance = 25.0f;
    sensors.right_sector_distance = 25.0f;
    sensors.upper_sector_distance = 25.0f;
    sensors.lower_sector_distance = 25.0f;

    auto cmd_clear = avoidance.update(sensors, 0.05f);
    EXPECT_EQ(cmd_clear.type, ControlType::Velocity);
    EXPECT_GT(cmd_clear.vector.x(), 0.0f); // Forward cruising
    EXPECT_FLOAT_EQ(cmd_clear.vector.y(), 0.0f);

    // 2. Obstacle penetrating stopping bubble (min depth 1.2m < stopping distance ~ 2.5m)
    sensors.min_depth_distance = 1.2f;
    sensors.left_sector_distance = 8.0f; // Left side is clear
    sensors.right_sector_distance = 1.0f; // Right side is blocked

    auto cmd_evade = avoidance.update(sensors, 0.05f);
    EXPECT_GT(cmd_evade.vector.y(), 0.0f); // Lateral evasion to left
    EXPECT_GT(cmd_evade.yaw_or_yaw_rate, 0.0f); // Yaw left
}

// ==============================================================================
// 4. ENTERPRISE FLIGHT SUPERVISOR INTEGRATION TESTS
// ==============================================================================
TEST_F(AutonomyAlgorithmsTest, EnterpriseFlightSupervisorSafetyOverride) {
    EnterpriseFlightSupervisor supervisor;
    supervisor.init(*test_node_);

    SensorSnapshot sensors;
    sensors.dt = 0.05f;
    sensors.position = Eigen::Vector3f(0.0f, 0.0f, 20.0f);
    sensors.velocity = Eigen::Vector3f(2.0f, 0.0f, 0.0f);

    AutonomyCommand nominal_cmd;
    nominal_cmd.type = ControlType::Velocity;
    nominal_cmd.vector = Eigen::Vector3f(2.0f, 0.0f, 0.0f);

    // Nominal conditions -> No override
    auto report_nom = supervisor.evaluate(sensors, 0.05f, nominal_cmd);
    EXPECT_FALSE(report_nom.emergency_override_active);

    // ADS-B Collision intruder threat -> Command Emergency Dive
    AdsbTarget intruder;
    intruder.icao_address = 0x998877;
    intruder.position = Eigen::Vector3f(600.0f, 0.0f, 20.0f);
    intruder.velocity = Eigen::Vector3f(-40.0f, 0.0f, 0.0f);
    supervisor.updateAdsbTraffic({intruder});

    auto report_adsb = supervisor.evaluate(sensors, 0.05f, nominal_cmd);
    EXPECT_TRUE(report_adsb.emergency_override_active);
    EXPECT_EQ(report_adsb.override_source, "AdsbDeconfliction");
    EXPECT_LT(report_adsb.override_command.vector.z(), -2.0f); // Emergency dive

    // Clear ADS-B traffic
    supervisor.updateAdsbTraffic({});

    // Motor failure injection -> Command Quadcopter Spin Recovery
    supervisor.triggerMotorFailure(MotorIndex::Motor1_FR);
    auto report_spin = supervisor.evaluate(sensors, 0.05f, nominal_cmd);
    EXPECT_TRUE(report_spin.emergency_override_active);
    EXPECT_EQ(report_spin.override_source, "QuadcopterSpinRecovery");
    EXPECT_NEAR(report_spin.override_command.vector.z(), -1.8f, 0.1f);
}

// ==============================================================================
// 5. KINEMATICS TRANSFORM FORMULAS TESTS
// ==============================================================================
TEST_F(AutonomyAlgorithmsTest, KinematicsTransformFormulas) {
    // 1. Body FLU to Earth NED Velocity Transform
    float heading_north = 0.0f;
    float cos_0 = std::cos(heading_north);
    float sin_0 = std::sin(heading_north);
    float v_fwd = 1.0f, v_left = 1.0f, v_up = 0.5f;

    float vx_ned =  v_fwd * cos_0 + v_left * sin_0;
    float vy_ned =  v_fwd * sin_0 - v_left * cos_0;
    float vz_ned = -v_up;

    EXPECT_FLOAT_EQ(vx_ned, 1.0f);  // North
    EXPECT_FLOAT_EQ(vy_ned, -1.0f); // West (-East)
    EXPECT_FLOAT_EQ(vz_ned, -0.5f); // Down
}

// ==============================================================================
// 6. DYNAMIC STOPPING BUBBLE PHYSICS TESTS
// ==============================================================================
TEST_F(AutonomyAlgorithmsTest, DynamicStoppingBubbleQuadraticDistanceAndBraking) {
    DynamicStoppingBubbleConfig cfg;
    cfg.a_max = 4.0f;
    cfg.t_latency = 0.15f;
    cfg.d_margin = 0.8f;
    DynamicStoppingBubble bubble(cfg);

    EXPECT_FLOAT_EQ(bubble.computeStoppingDistance(4.0f), 3.4f);

    Eigen::Vector3f vel(4.0f, 0.0f, 0.0f);
    Eigen::Vector3f a_brake = bubble.computeBrakingAcceleration(vel);
    EXPECT_FLOAT_EQ(a_brake.x(), -4.0f);
}

// ==============================================================================
// 7. SAFE FLIGHT CORRIDOR TESTS
// ==============================================================================
TEST_F(AutonomyAlgorithmsTest, SafeFlightCorridorConvexSlicesAndJerkLimit) {
    SafeFlightCorridor sfc;

    std::vector<Eigen::Vector3f> waypoints = {
        Eigen::Vector3f(0.0f, 0.0f, 2.0f),
        Eigen::Vector3f(5.0f, 0.0f, 2.0f),
        Eigen::Vector3f(10.0f, 5.0f, 3.0f)
    };
    std::vector<Eigen::Vector3f> obstacles = {
        Eigen::Vector3f(2.5f, 1.2f, 2.0f)
    };

    auto slices = sfc.generateCorridor(waypoints, obstacles);
    ASSERT_EQ(slices.size(), 2u);
    EXPECT_GE(slices[0].hyperplanes.size(), 6u);
}

// ==============================================================================
// 8. QUADCOPTER SPIN RECOVERY TESTS
// ==============================================================================
TEST_F(AutonomyAlgorithmsTest, QuadcopterSpinRecoveryMuellerDAndrea) {
    SpinRecoveryConfig cfg;
    cfg.target_spin_rate = 25.0f;
    cfg.target_descent_rate = -1.8f;
    QuadcopterSpinRecovery recovery(cfg);

    EXPECT_FALSE(recovery.isRecoveryActive());

    recovery.triggerMotorFailure(MotorIndex::Motor1_FR);
    EXPECT_TRUE(recovery.isRecoveryActive());
    EXPECT_EQ(recovery.getFailedMotor(), MotorIndex::Motor1_FR);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
