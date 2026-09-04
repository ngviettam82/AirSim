#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

namespace px4_airsim_autonomy {
namespace production {

/**
 * @brief Navigation Quality Tiers defining operational capability in degraded/GPS-denied environments.
 */
enum class NavigationQualityTier : uint8_t {
    Tier0_RtkFixed = 0,               ///< Full RTK Fixed centimeter-level GNSS (15 m/s)
    Tier1_RtkFloat = 1,               ///< RTK Float / High-precision DGPS sub-meter level (10 m/s)
    Tier2_VioLioOdometry = 2,         ///< GPS-denied Visual-Inertial / LiDAR-Inertial Odometry (5 m/s)
    Tier3_InertialDragDeadReckoning = 3, ///< Optical/IMU drag dead reckoning, drifting (1.5 m/s)
    Tier4_EmergencyDescendLand = 4    ///< All relative position/velocity lost; failsafe hover & land (0 m/s)
};

/**
 * @brief GNSS Fix classification types.
 */
enum class GnssFixType : uint8_t {
    NoFix = 0,
    Fix2D = 1,
    Fix3D = 2,
    RtkFloat = 3,
    RtkFixed = 4
};

/**
 * @brief Comprehensive sensor health and innovation test metrics input for FSM evaluation.
 */
struct NavigationHealthInputs {
    uint64_t timestamp_us = 0;

    // --- GNSS Signals ---
    bool gnss_received = false;                     ///< Set true when a GNSS message was received this cycle
    GnssFixType gnss_fix = GnssFixType::NoFix;      ///< Receiver fix type
    int gnss_satellites = 0;                        ///< Number of satellites tracked & used
    float gnss_eph = 999.0f;                        ///< Horizontal dilution of precision / position error (m)
    float gnss_epv = 999.0f;                        ///< Vertical position error (m)
    float gnss_innovation_test_ratio = 0.0f;        ///< Normalized innovation squared: chi^2 / gamma_gate

    // --- VIO / LIO Odometry Signals ---
    bool vio_received = false;                      ///< Set true when a VIO/LIO message was received this cycle
    bool vio_tracking_valid = false;                ///< Visual tracking status / pose estimator valid
    int vio_tracked_features = 0;                   ///< Number of visual features actively tracked
    float vio_confidence = 0.0f;                    ///< Estimator confidence metric [0.0, 1.0]
    float vio_innovation_test_ratio = 0.0f;         ///< Normalized innovation squared: chi^2 / gamma_gate

    // --- Inertial / Core State Estimation ---
    bool imu_healthy = true;                        ///< Accelerometer & gyroscope integrity check
    bool ekf_diverged = false;                      ///< EKF filter divergence flag
    float ekf_innovation_test_ratio = 0.0f;         ///< Composite EKF innovation ratio
};

/**
 * @brief Tunable configuration parameters for the degradation FSM.
 */
struct DegradedFsmConfig {
    // Watchdog dropout timers (seconds)
    float gnss_timeout_sec = 0.5f;                  ///< Time without GNSS before declaring dropout
    float vio_timeout_sec = 0.3f;                   ///< Time without VIO before declaring dropout
    float imu_timeout_sec = 0.1f;                   ///< Time without IMU before emergency failsafe

    // Innovation gating threshold: ratio > 1.0 indicates outlier / measurement rejection
    float innovation_gate_ratio = 1.0f;

    // Promotion hysteresis requirement (seconds)
    float promotion_hysteresis_sec = 2.0f;          ///< Health must be continuously sustained for > 2.0s

    // Dead reckoning duration limit in Tier 3 before triggering emergency landing
    float max_dead_reckoning_sec = 15.0f;

    // GNSS quality thresholds
    int min_satellites_rtk_fixed = 8;
    int min_satellites_rtk_float = 6;
    float max_eph_rtk_fixed = 0.15f;                ///< Max allowed horizontal uncertainty for RTK fixed (m)
    float max_eph_rtk_float = 0.80f;                ///< Max allowed horizontal uncertainty for RTK float (m)

    // VIO quality thresholds
    int min_vio_features = 12;
    float min_vio_confidence = 0.40f;

    // Speed limits per tier (m/s)
    float speed_limit_tier0 = 15.0f;
    float speed_limit_tier1 = 10.0f;
    float speed_limit_tier2 = 5.0f;
    float speed_limit_tier3 = 1.5f;
    float speed_limit_tier4 = 0.0f;

    // Emergency descent velocity for Tier 4 (m/s, negative for downward)
    float emergency_descent_vel_z = -0.5f;
};

/**
 * @brief Real-time status report from the Degraded Navigation FSM.
 */
struct NavigationStatusReport {
    NavigationQualityTier current_tier = NavigationQualityTier::Tier0_RtkFixed;
    NavigationQualityTier previous_tier = NavigationQualityTier::Tier0_RtkFixed;

    // Operational status flags
    bool is_gps_denied = false;                     ///< True when operating in Tier 2, 3, or 4
    bool is_dead_reckoning = false;                 ///< True when operating in Tier 3
    bool is_failsafe_active = false;                ///< True when operating in Tier 4
    bool is_rtk_fixed = false;                      ///< True in Tier 0
    bool is_rtk_float = false;                      ///< True in Tier 1
    bool is_vio_active = false;                     ///< True in Tier 2

    // Dynamic flight envelopes
    float max_allowable_speed = 15.0f;              ///< Maximum safe horizontal velocity command (m/s)
    float commanded_vz_failsafe = 0.0f;             ///< Downward descent rate if in Tier 4 (m/s)

    // Diagnostic & telemetry details
    std::string tier_name = "Tier0_RtkFixed";
    std::string transition_reason;
    float time_in_current_tier_sec = 0.0f;
    float dead_reckoning_elapsed_sec = 0.0f;
    float promotion_dwell_time_sec = 0.0f;

    // Active sensor health flags
    bool gnss_healthy = false;
    bool vio_healthy = false;
    bool imu_healthy = true;
    float latest_gnss_innovation_ratio = 0.0f;
    float latest_vio_innovation_ratio = 0.0f;
};

/**
 * @brief Transition event callback signature: (old_tier, new_tier, reason).
 */
using TierTransitionCallback = std::function<void(NavigationQualityTier, NavigationQualityTier, const std::string&)>;

/**
 * @brief Production GPS-denied Degraded Navigation State Machine.
 * 
 * Monitors sensor freshness (watchdog dropout timers) and innovation test ratios
 * (chi^2 / gamma_gate > 1.0). Automatically demotes navigation tier upon degradation
 * or failure, and promotes with strict 2.0-second hysteresis once sensors stabilize.
 * Dynamically enforces safe flight speed limits and failsafe triggers.
 * 
 * Fully thread-safe.
 */
class DegradedNavigationFsm {
public:
    explicit DegradedNavigationFsm(const DegradedFsmConfig& config = DegradedFsmConfig{});
    ~DegradedNavigationFsm() = default;

    /**
     * @brief Update the state machine with the latest sensor health telemetry.
     * 
     * @param inputs Current sensor health and innovation test metrics
     * @param dt Elapsed time since last update call in seconds
     * @return Current NavigationStatusReport
     */
    NavigationStatusReport update(const NavigationHealthInputs& inputs, float dt);

    /**
     * @brief Retrieve the latest status report without advancing state time.
     */
    NavigationStatusReport getStatus() const;

    /**
     * @brief Retrieve maximum allowable flight speed for active tier in m/s.
     */
    float getMaxAllowableSpeed() const;

    /**
     * @brief Retrieve maximum allowable flight speed for a specified tier in m/s.
     */
    float getMaxSpeedForTier(NavigationQualityTier tier) const;

    /**
     * @brief Get active navigation quality tier.
     */
    NavigationQualityTier getCurrentTier() const;

    /**
     * @brief Check if vehicle is currently in GPS-denied navigation mode (Tier 2, 3, or 4).
     */
    bool isGpsDenied() const;

    /**
     * @brief Check if emergency failsafe landing is active (Tier 4).
     */
    bool isFailsafeActive() const;

    /**
     * @brief Human-readable string identifier for a navigation tier.
     */
    static std::string tierToString(NavigationQualityTier tier);

    /**
     * @brief Register an optional callback invoked whenever a tier transition occurs.
     */
    void registerTransitionCallback(TierTransitionCallback callback);

    /**
     * @brief Update state machine configuration parameters.
     */
    void setConfig(const DegradedFsmConfig& config);

    /**
     * @brief Retrieve current configuration parameters.
     */
    DegradedFsmConfig getConfig() const;

    /**
     * @brief Reset state machine back to Tier0 with zeroed timers.
     */
    void reset(NavigationQualityTier initial_tier = NavigationQualityTier::Tier0_RtkFixed);

private:
    // Health evaluation helpers
    bool checkTier0Health(const NavigationHealthInputs& inputs) const;
    bool checkTier1Health(const NavigationHealthInputs& inputs) const;
    bool checkTier2Health(const NavigationHealthInputs& inputs) const;
    bool checkTier3Health(const NavigationHealthInputs& inputs) const;

    NavigationQualityTier evaluateHighestHealthyTier(const NavigationHealthInputs& inputs) const;
    void executeTransition(NavigationQualityTier target_tier, const std::string& reason);

    mutable std::mutex mutex_;

    DegradedFsmConfig config_;
    NavigationQualityTier current_tier_ = NavigationQualityTier::Tier0_RtkFixed;
    NavigationQualityTier previous_tier_ = NavigationQualityTier::Tier0_RtkFixed;

    // Watchdog dropout timers (seconds elapsed since last valid packet)
    float gnss_dropout_timer_sec_ = 0.0f;
    float vio_dropout_timer_sec_ = 0.0f;
    float imu_dropout_timer_sec_ = 0.0f;

    // Hysteresis promotion tracking
    NavigationQualityTier candidate_promotion_tier_ = NavigationQualityTier::Tier0_RtkFixed;
    float promotion_dwell_timer_sec_ = 0.0f;

    // Dead reckoning elapsed time in Tier 3
    float dead_reckoning_elapsed_sec_ = 0.0f;

    // Uptime in current tier
    float time_in_current_tier_sec_ = 0.0f;

    std::string latest_transition_reason_ = "Initial state";
    TierTransitionCallback transition_callback_ = nullptr;

    NavigationStatusReport latest_report_;
};

} // namespace production
} // namespace px4_airsim_autonomy
