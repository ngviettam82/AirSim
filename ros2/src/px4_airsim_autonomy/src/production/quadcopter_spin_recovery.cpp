#include "px4_airsim_autonomy/production/quadcopter_spin_recovery.hpp"

#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace px4_airsim_autonomy {
namespace production {

namespace {
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kEpsilon = 1e-4f;
} // namespace

QuadcopterSpinRecovery::QuadcopterSpinRecovery(const SpinRecoveryConfig& config)
    : config_(config) {
    reset();
}

void QuadcopterSpinRecovery::reset() {
    state_ = RecoveryState::Nominal;
    failed_motor_ = MotorIndex::None;
    spin_azimuth_ = 0.0f;
    filtered_spin_rate_ = 0.0f;
    active_indices_ = {0, 1, 2};
    B_alloc_ = Eigen::Matrix3f::Identity();
    B_alloc_inv_ = Eigen::Matrix3f::Identity();
    last_output_ = SpinRecoveryOutput();
}

void QuadcopterSpinRecovery::triggerMotorFailure(MotorIndex motor) {
    if (motor == MotorIndex::None) {
        reset();
        return;
    }

    failed_motor_ = motor;
    state_ = RecoveryState::FailureDetected;
    updateAllocationMatrix();
}

void QuadcopterSpinRecovery::updateAllocationMatrix() {
    if (failed_motor_ == MotorIndex::None) {
        return;
    }

    // Determine the 3 surviving active motors
    int active_cnt = 0;
    const int failed_idx = static_cast<int>(failed_motor_);

    for (int m = 0; m < 4; ++m) {
        if (m != failed_idx && active_cnt < 3) {
            active_indices_[active_cnt++] = m;
        }
    }

    // Assemble 3x3 allocation matrix mapping [T1, T2, T3]_active -> [T_coll, M_x, M_y]
    // Motor position in body frame: x_m = L * cos(angle_m), y_m = L * sin(angle_m)
    // T_coll = sum(T_i)
    // M_x    = sum(y_i * T_i)
    // M_y    = sum(-x_i * T_i)
    for (int k = 0; k < 3; ++k) {
        const int m = active_indices_[k];
        const float angle = config_.airframe.motor_angles[m];
        const float x_m = config_.airframe.arm_length * std::cos(angle);
        const float y_m = config_.airframe.arm_length * std::sin(angle);

        B_alloc_(0, k) = 1.0f; // Collective thrust contribution
        B_alloc_(1, k) = y_m;  // Roll moment contribution
        B_alloc_(2, k) = -x_m; // Pitch moment contribution
    }

    // Invert the 3x3 allocation matrix
    B_alloc_inv_ = B_alloc_.inverse();
}

SpinRecoveryOutput QuadcopterSpinRecovery::update(const SpinRecoveryTelemetry& telemetry) {
    SpinRecoveryOutput output;
    output.state = state_;
    output.failed_motor = failed_motor_;

    // If nominal or touchdown, no active spin recovery thrust
    if (state_ == RecoveryState::Nominal) {
        output.diagnostic_message = "Nominal flight: 4 motors operational.";
        return output;
    }

    if (state_ == RecoveryState::Touchdown) {
        output.motor_thrusts_n.fill(0.0f);
        output.normalized_motor_outputs.fill(0.0f);
        output.diagnostic_message = "Touchdown detected: all motors disarmed.";
        return output;
    }

    // ------------------------------------------------------------------------
    // Step 1: Spin Rate Filtering & Azimuth Integration
    // Abandon yaw regulation: vehicle spins freely at omega_z ~ 25 rad/s.
    // ------------------------------------------------------------------------
    const float raw_omega_z = telemetry.angular_velocity.z();
    const float alpha_filt = std::clamp(telemetry.dt / (telemetry.dt + 0.02f), 0.05f, 1.0f);
    filtered_spin_rate_ = (1.0f - alpha_filt) * filtered_spin_rate_ + alpha_filt * raw_omega_z;

    // Azimuth integration: theta(t) = int omega_z dt
    spin_azimuth_ += raw_omega_z * telemetry.dt;
    while (spin_azimuth_ >= kTwoPi) {
        spin_azimuth_ -= kTwoPi;
    }
    while (spin_azimuth_ < 0.0f) {
        spin_azimuth_ += kTwoPi;
    }

    output.current_spin_azimuth = spin_azimuth_;
    output.filtered_spin_rate = filtered_spin_rate_;

    // Check if vehicle has spun up into stable recovery regime
    if (state_ == RecoveryState::FailureDetected && std::abs(raw_omega_z) > 10.0f) {
        state_ = RecoveryState::SpinRecoveryActive;
    }

    // ------------------------------------------------------------------------
    // Step 2: Horizontal Translation Regulation via Cyclic Tilt Modulation
    // ------------------------------------------------------------------------
    const Eigen::Vector2f pos_xy(telemetry.position.x(), telemetry.position.y());
    const Eigen::Vector2f vel_xy(telemetry.velocity.x(), telemetry.velocity.y());
    const Eigen::Vector2f pos_err = telemetry.target_horizontal_position - pos_xy;
    const Eigen::Vector2f vel_err = -vel_xy; // Target horizontal velocity is 0 m/s

    // Desired horizontal acceleration in world ENU frame
    const Eigen::Vector2f a_xy_des = config_.kp_horizontal * pos_err + config_.kd_horizontal * vel_err;

    // Desired mean tilt vector of the spin axis: alpha = a_xy / g
    Eigen::Vector2f alpha = a_xy_des / config_.airframe.gravity;
    float tilt_mag = alpha.norm();

    if (tilt_mag > config_.max_tilt_angle) {
        alpha = alpha * (config_.max_tilt_angle / tilt_mag);
        tilt_mag = config_.max_tilt_angle;
    }
    output.commanded_tilt = tilt_mag;

    // Phase compensation for motor thrust lag: theta_eff = theta + omega_z * t_delay
    const float theta_eff = spin_azimuth_ + raw_omega_z * config_.phase_delay_comp_sec;

    // Cyclic moments in body frame synchronized with rotating azimuth
    const float moment_scale = config_.k_cyclic * config_.airframe.mass * config_.airframe.gravity * config_.airframe.arm_length;
    const float delta_Mx = (alpha.x() * std::cos(theta_eff) + alpha.y() * std::sin(theta_eff)) * moment_scale;
    const float delta_My = (-alpha.x() * std::sin(theta_eff) + alpha.y() * std::cos(theta_eff)) * moment_scale;

    // ------------------------------------------------------------------------
    // Step 3: Vertical Altitude Control - Safe Descent Rate (v_z ~ -1.8 m/s)
    // ------------------------------------------------------------------------
    const float vz_current = telemetry.velocity.z();
    const float vz_err = config_.target_descent_rate - vz_current;
    const float a_z_des = config_.kd_vertical * vz_err;

    // Projected collective thrust to compensate for vehicle mean tilt
    const float cos_tilt = std::max(0.6f, std::cos(tilt_mag));
    float T_collective = (config_.airframe.mass * (config_.airframe.gravity + a_z_des)) / cos_tilt;

    // Clamp collective thrust to physical capabilities of the 3 remaining motors
    const float min_total = 3.0f * config_.airframe.min_thrust_per_motor;
    const float max_total = 3.0f * config_.airframe.max_thrust_per_motor;
    T_collective = std::clamp(T_collective, min_total, max_total);
    output.collective_thrust_n = T_collective;

    // Check transition to stabilized descent
    if (state_ == RecoveryState::SpinRecoveryActive &&
        vel_xy.norm() < 1.0f &&
        std::abs(vz_current - config_.target_descent_rate) < 0.8f) {
        state_ = RecoveryState::StabilizedDescent;
    }

    // ------------------------------------------------------------------------
    // Step 4: Actuator Allocation for 3 Surviving Motors
    // ------------------------------------------------------------------------
    const Eigen::Vector3f w_des(T_collective, delta_Mx, delta_My);
    Eigen::Vector3f T_active = B_alloc_inv_ * w_des;

    // Ensure collective thrust is prioritized if individual motor saturates
    float active_sum = T_active.sum();
    if (active_sum > kEpsilon && std::abs(active_sum - T_collective) > 0.1f) {
        T_active = T_active * (T_collective / active_sum);
    }

    // Clamp individual thrusts within physical motor saturation limits
    for (int k = 0; k < 3; ++k) {
        T_active(k) = std::clamp(
            T_active(k),
            config_.airframe.min_thrust_per_motor,
            config_.airframe.max_thrust_per_motor);
    }

    // Clear all motor outputs first
    output.motor_thrusts_n.fill(0.0f);
    output.normalized_motor_outputs.fill(0.0f);

    // Assign active motor thrusts (failed motor remains 0.0 N)
    for (int k = 0; k < 3; ++k) {
        const int m = active_indices_[k];
        output.motor_thrusts_n[m] = T_active(k);
        output.normalized_motor_outputs[m] = std::clamp(
            T_active(k) / config_.airframe.max_thrust_per_motor,
            0.0f,
            1.0f);
    }

    // ------------------------------------------------------------------------
    // Step 5: Touchdown Detection
    // ------------------------------------------------------------------------
    if (telemetry.position.z() <= config_.touchdown_altitude_m &&
        vz_current > -config_.touchdown_velocity_threshold) {
        state_ = RecoveryState::Touchdown;
        output.state = state_;
        output.motor_thrusts_n.fill(0.0f);
        output.normalized_motor_outputs.fill(0.0f);
        output.diagnostic_message = "TOUCHDOWN: Landed safely; spin recovery disarmed.";
        return output;
    }

    // Status diagnostics
    std::ostringstream oss;
    oss << (state_ == RecoveryState::StabilizedDescent ? "STABILIZED_DESCENT" : "SPIN_RECOVERY")
        << ": FailedMotor=" << static_cast<int>(failed_motor_)
        << ", SpinRate=" << std::fixed << std::setprecision(1) << filtered_spin_rate_ << " rad/s"
        << ", Azimuth=" << std::setprecision(2) << spin_azimuth_ << " rad"
        << ", Tilt=" << std::setprecision(2) << tilt_mag << " rad"
        << ", Vz=" << std::setprecision(2) << vz_current << " m/s (target " << config_.target_descent_rate << " m/s)"
        << ", Thrust=[";
    for (int i = 0; i < 4; ++i) {
        oss << std::setprecision(1) << output.motor_thrusts_n[i] << (i < 3 ? ", " : "]");
    }
    output.diagnostic_message = oss.str();

    last_output_ = output;
    return output;
}

SpinRecoveryOutput QuadcopterSpinRecovery::update(
    const Eigen::Vector3f& position,
    const Eigen::Vector3f& velocity,
    float omega_z,
    float dt) {
    SpinRecoveryTelemetry telem;
    telem.position = position;
    telem.velocity = velocity;
    telem.angular_velocity = Eigen::Vector3f(0.0f, 0.0f, omega_z);
    telem.dt = dt;
    telem.target_horizontal_position = Eigen::Vector2f(position.x(), position.y());
    return update(telem);
}

} // namespace production
} // namespace px4_airsim_autonomy
