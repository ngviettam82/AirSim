#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <array>
#include <cstdint>

#include "px4_airsim_autonomy/types.hpp"

namespace px4_airsim_autonomy {
namespace production {

/**
 * @brief Identifiers for the 4 quadcopter motors (PX4 Quad X convention).
 */
enum class MotorIndex : int {
    Motor1_FrontRight = 0, ///< Front-Right (CW)
    Motor2_RearLeft = 1,   ///< Rear-Left (CW)
    Motor3_FrontLeft = 2,  ///< Front-Left (CCW)
    Motor4_RearRight = 3,  ///< Rear-Right (CCW)
    Motor1_FR = 0,         ///< Shorthand alias
    Motor2_RL = 1,         ///< Shorthand alias
    Motor3_FL = 2,         ///< Shorthand alias
    Motor4_RR = 3,         ///< Shorthand alias
    None = -1
};

/**
 * @brief High-level contingency state of the spin recovery controller.
 */
enum class RecoveryState {
    Nominal,             ///< All 4 motors operational; recovery controller dormant.
    FailureDetected,     ///< Single-motor failure detected; initiating spin-up transition.
    SpinRecoveryActive,  ///< Yaw regulation abandoned; spinning at stable ~25 rad/s with cyclic tilt control.
    StabilizedDescent,   ///< Horizontal drift arrested; descending steadily at ~ -1.8 m/s.
    Touchdown            ///< Ground touchdown detected; motors disarmed.
};

/**
 * @brief Physical and geometric parameters of the quadcopter airframe.
 */
struct AirframeParams {
    /// Total vehicle mass in kg.
    float mass = 1.5f;

    /// Arm length from center of mass to motor shaft (meters).
    float arm_length = 0.25f;

    /// Motor angular positions relative to forward body axis (+x) in radians (Quad X):
    /// FR (-pi/4), RL (3pi/4), FL (pi/4), RR (-3pi/4).
    std::array<float, 4> motor_angles = {
        -0.785398163f,  // Motor 1: FR (-45 deg)
         2.356194490f,  // Motor 2: RL (+135 deg)
         0.785398163f,  // Motor 3: FL (+45 deg)
        -2.356194490f   // Motor 4: RR (-135 deg)
    };

    /// Motor spin direction signs (+1 = CW, -1 = CCW).
    std::array<float, 4> motor_directions = {1.0f, 1.0f, -1.0f, -1.0f};

    /// Aerodynamic torque-to-thrust ratio: tau_z = c_tau * T (meters).
    float c_tau = 0.016f;

    /// Maximum continuous thrust per motor (Newtons).
    float max_thrust_per_motor = 12.0f;

    /// Minimum thrust per motor to maintain spin and control authority (Newtons).
    float min_thrust_per_motor = 0.4f;

    /// Standard gravitational acceleration (m/s^2).
    float gravity = 9.81f;
};

/**
 * @brief Tuning parameters for Mueller & D'Andrea spin recovery controller.
 */
struct SpinRecoveryConfig {
    /// Airframe physical constants.
    AirframeParams airframe;

    /// Target steady-state spin rate in rad/s (nominal ~ 25.0 rad/s).
    float target_spin_rate = 25.0f;

    /// Commanded safe vertical descent velocity in m/s (nominal -1.8 m/s).
    float target_descent_rate = -1.8f;

    /// Maximum allowed mean tilt angle of the spin axis from vertical (radians, ~20 deg).
    float max_tilt_angle = 0.35f;

    /// Horizontal position proportional gain (1/s^2).
    float kp_horizontal = 1.2f;

    /// Horizontal velocity derivative gain (1/s).
    float kd_horizontal = 1.8f;

    /// Vertical descent velocity feedback gain.
    float kd_vertical = 2.5f;

    /// Cyclic pitch/roll moment authority scaling gain.
    float k_cyclic = 1.0f;

    /// Phase lead compensation to counteract motor/ESC response lag: theta_lead = omega_z * t_delay (seconds).
    float phase_delay_comp_sec = 0.025f;

    /// Altitude threshold (m AGL) below which touchdown is triggered.
    float touchdown_altitude_m = 0.25f;

    /// Vertical velocity threshold (m/s) to confirm touchdown.
    float touchdown_velocity_threshold = 0.20f;
};

/**
 * @brief Input telemetry state fed to the spin recovery controller.
 */
struct SpinRecoveryTelemetry {
    /// Vehicle position in local ENU frame [x, y, z] (meters).
    Eigen::Vector3f position = Eigen::Vector3f::Zero();

    /// Vehicle linear velocity in local ENU frame [vx, vy, vz] (m/s).
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

    /// Vehicle angular rate in body frame [p, q, r] (rad/s, r is yaw rate).
    Eigen::Vector3f angular_velocity = Eigen::Vector3f::Zero();

    /// Current attitude orientation quaternion (world ENU to body).
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();

    /// Desired horizontal position setpoint in ENU frame [x_des, y_des] (meters).
    Eigen::Vector2f target_horizontal_position = Eigen::Vector2f::Zero();

    /// Elapsed time since last controller update in seconds.
    float dt = 0.01f;
};

/**
 * @brief Output actuator setpoints and diagnostics computed by the recovery controller.
 */
struct SpinRecoveryOutput {
    /// Current contingency state.
    RecoveryState state = RecoveryState::Nominal;

    /// Index of the failed motor (or None).
    MotorIndex failed_motor = MotorIndex::None;

    /// Commanded individual motor thrusts in Newtons [T1, T2, T3, T4].
    /// Failed motor is guaranteed to be 0.0 N.
    std::array<float, 4> motor_thrusts_n = {0.0f, 0.0f, 0.0f, 0.0f};

    /// Normalized actuator commands in range [0.0, 1.0] for PX4 actuator_motors topic.
    std::array<float, 4> normalized_motor_outputs = {0.0f, 0.0f, 0.0f, 0.0f};

    /// Current spin azimuth angle theta(t) in radians [0, 2*pi).
    float current_spin_azimuth = 0.0f;

    /// Filtered body spin rate omega_z in rad/s.
    float filtered_spin_rate = 0.0f;

    /// Commanded tilt magnitude of spin axis in radians.
    float commanded_tilt = 0.0f;

    /// Commanded total collective thrust in Newtons.
    float collective_thrust_n = 0.0f;

    /// Diagnostic status message.
    std::string diagnostic_message;
};

/**
 * @brief Production implementation of Mueller & D'Andrea Single-Motor Failure Recovery Controller.
 *
 * References:
 * - Mark W. Mueller and Raffaello D'Andrea, "Relaxed Hover Solutions for Multicopters:
 *   Application to Algorithmic Redundancy and Safe Flight after Motor Failure", IEEE T-RO 2014.
 * - Mark W. Mueller and Raffaello D'Andrea, "Stability and Control of a Quadrocopter
 *   Despite the Complete Loss of One, Two, or Three Propellers", IEEE/RSJ IROS 2014.
 *
 * Algorithmic Principles:
 * 1. Abandons yaw regulation: Quadcopter loses yaw equilibrium when one motor fails.
 *    Vehicle spins at a stable rate omega_z ~ 25 rad/s.
 * 2. Modulates cyclic thrust on the 3 active motors synchronized with spin azimuth:
 *    theta(t) = int omega_z dt.
 * 3. Stabilizes horizontal translation via periodic cyclic tilting.
 * 4. Regulates vertical descent rate to safe target v_z ~ -1.8 m/s via collective thrust.
 */
class QuadcopterSpinRecovery {
public:
    using MotorIndex = px4_airsim_autonomy::production::MotorIndex;

    explicit QuadcopterSpinRecovery(const SpinRecoveryConfig& config = SpinRecoveryConfig());

    ~QuadcopterSpinRecovery() = default;

    /**
     * @brief Trigger single-motor failure and engage spin recovery.
     * @param motor Index of the failed motor (0 to 3).
     */
    void triggerMotorFailure(MotorIndex motor);

    /**
     * @brief Reset controller back to nominal state.
     */
    void reset();

    /**
     * @brief Main periodic computation step (typically called at 50 Hz - 200 Hz).
     * @param telemetry Current vehicle kinematics and timing.
     * @return SpinRecoveryOutput Actuator setpoints and status.
     */
    SpinRecoveryOutput update(const SpinRecoveryTelemetry& telemetry);

    /**
     * @brief Convenience update overload taking position, velocity, yaw spin rate, and dt.
     * @param position Current position [x, y, z] in local ENU frame (meters).
     * @param velocity Current velocity [vx, vy, vz] in local ENU frame (m/s).
     * @param omega_z Body yaw spin rate in rad/s.
     * @param dt Loop period in seconds.
     * @return SpinRecoveryOutput Actuator setpoints and status.
     */
    SpinRecoveryOutput update(
        const Eigen::Vector3f& position,
        const Eigen::Vector3f& velocity,
        float omega_z,
        float dt);

    /**
     * @brief Get the latest commanded active motor thrusts [T1, T2, T3, T4] in Newtons.
     */
    std::array<float, 4> getActiveThrustCommands() const {
        return last_output_.motor_thrusts_n;
    }

    /**
     * @brief Check if spin recovery mode is currently active.
     */
    bool isRecoveryActive() const {
        return state_ == RecoveryState::FailureDetected ||
               state_ == RecoveryState::SpinRecoveryActive ||
               state_ == RecoveryState::StabilizedDescent;
    }

    RecoveryState getState() const { return state_; }
    MotorIndex getFailedMotor() const { return failed_motor_; }
    float getCurrentSpinAzimuth() const { return spin_azimuth_; }
    float getFilteredSpinRate() const { return filtered_spin_rate_; }
    const SpinRecoveryOutput& getLastOutput() const { return last_output_; }

    const SpinRecoveryConfig& getConfig() const { return config_; }
    void setConfig(const SpinRecoveryConfig& config) { config_ = config; }

private:
    /**
     * @brief Compute the 3x3 control allocation matrix for the 3 functioning motors.
     * [T_collective; M_x; M_y] = B_3x3 * [T_active1; T_active2; T_active3]
     */
    void updateAllocationMatrix();

    SpinRecoveryConfig config_;
    RecoveryState state_ = RecoveryState::Nominal;
    MotorIndex failed_motor_ = MotorIndex::None;
    SpinRecoveryOutput last_output_;

    /// Integrated spin azimuth angle theta(t) in radians [0, 2*pi).
    float spin_azimuth_ = 0.0f;

    /// Low-pass filtered spin rate omega_z (rad/s).
    float filtered_spin_rate_ = 0.0f;

    /// Indices of the 3 active functioning motors.
    std::array<int, 3> active_indices_ = {0, 1, 2};

    /// 3x3 allocation matrix mapping active motor thrusts to [T_coll, M_x, M_y].
    Eigen::Matrix3f B_alloc_ = Eigen::Matrix3f::Identity();

    /// Inverse allocation matrix for fast analytical solving.
    Eigen::Matrix3f B_alloc_inv_ = Eigen::Matrix3f::Identity();
};

} // namespace production
} // namespace px4_airsim_autonomy
