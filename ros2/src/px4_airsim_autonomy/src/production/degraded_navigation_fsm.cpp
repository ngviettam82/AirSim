#include "px4_airsim_autonomy/production/degraded_navigation_fsm.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace px4_airsim_autonomy {
namespace production {

DegradedNavigationFsm::DegradedNavigationFsm(const DegradedFsmConfig& config)
    : config_(config)
{
    reset(NavigationQualityTier::Tier0_RtkFixed);
}

std::string DegradedNavigationFsm::tierToString(NavigationQualityTier tier) {
    switch (tier) {
        case NavigationQualityTier::Tier0_RtkFixed:
            return "Tier0_RtkFixed";
        case NavigationQualityTier::Tier1_RtkFloat:
            return "Tier1_RtkFloat";
        case NavigationQualityTier::Tier2_VioLioOdometry:
            return "Tier2_VioLioOdometry";
        case NavigationQualityTier::Tier3_InertialDragDeadReckoning:
            return "Tier3_InertialDragDeadReckoning";
        case NavigationQualityTier::Tier4_EmergencyDescendLand:
            return "Tier4_EmergencyDescendLand";
        default:
            return "UnknownTier";
    }
}

float DegradedNavigationFsm::getMaxSpeedForTier(NavigationQualityTier tier) const {
    switch (tier) {
        case NavigationQualityTier::Tier0_RtkFixed:
            return config_.speed_limit_tier0;
        case NavigationQualityTier::Tier1_RtkFloat:
            return config_.speed_limit_tier1;
        case NavigationQualityTier::Tier2_VioLioOdometry:
            return config_.speed_limit_tier2;
        case NavigationQualityTier::Tier3_InertialDragDeadReckoning:
            return config_.speed_limit_tier3;
        case NavigationQualityTier::Tier4_EmergencyDescendLand:
            return config_.speed_limit_tier4;
        default:
            return 0.0f;
    }
}

float DegradedNavigationFsm::getMaxAllowableSpeed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return getMaxSpeedForTier(current_tier_);
}

NavigationQualityTier DegradedNavigationFsm::getCurrentTier() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_tier_;
}

bool DegradedNavigationFsm::isGpsDenied() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_tier_ >= NavigationQualityTier::Tier2_VioLioOdometry;
}

bool DegradedNavigationFsm::isFailsafeActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_tier_ == NavigationQualityTier::Tier4_EmergencyDescendLand;
}

void DegradedNavigationFsm::registerTransitionCallback(TierTransitionCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    transition_callback_ = std::move(callback);
}

void DegradedNavigationFsm::setConfig(const DegradedFsmConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
}

DegradedFsmConfig DegradedNavigationFsm::getConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

void DegradedNavigationFsm::reset(NavigationQualityTier initial_tier) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_tier_ = initial_tier;
    previous_tier_ = initial_tier;
    gnss_dropout_timer_sec_ = 0.0f;
    vio_dropout_timer_sec_ = 0.0f;
    imu_dropout_timer_sec_ = 0.0f;
    candidate_promotion_tier_ = initial_tier;
    promotion_dwell_timer_sec_ = 0.0f;
    dead_reckoning_elapsed_sec_ = 0.0f;
    time_in_current_tier_sec_ = 0.0f;
    latest_transition_reason_ = "FSM Initialized";

    latest_report_ = NavigationStatusReport();
    latest_report_.current_tier = initial_tier;
    latest_report_.previous_tier = initial_tier;
    latest_report_.tier_name = tierToString(initial_tier);
    latest_report_.max_allowable_speed = getMaxSpeedForTier(initial_tier);
    latest_report_.transition_reason = latest_transition_reason_;
}

bool DegradedNavigationFsm::checkTier0Health(const NavigationHealthInputs& inputs) const {
    // Watchdogs & EKF health
    if (gnss_dropout_timer_sec_ > config_.gnss_timeout_sec ||
        imu_dropout_timer_sec_ > config_.imu_timeout_sec ||
        !inputs.imu_healthy || inputs.ekf_diverged) {
        return false;
    }

    // Innovation test ratio gating
    if (inputs.gnss_innovation_test_ratio > config_.innovation_gate_ratio ||
        inputs.ekf_innovation_test_ratio > config_.innovation_gate_ratio) {
        return false;
    }

    // GNSS quality metrics for RTK Fixed
    if (inputs.gnss_fix != GnssFixType::RtkFixed) {
        return false;
    }
    if (inputs.gnss_satellites < config_.min_satellites_rtk_fixed) {
        return false;
    }
    if (inputs.gnss_eph > config_.max_eph_rtk_fixed) {
        return false;
    }

    return true;
}

bool DegradedNavigationFsm::checkTier1Health(const NavigationHealthInputs& inputs) const {
    if (gnss_dropout_timer_sec_ > config_.gnss_timeout_sec ||
        imu_dropout_timer_sec_ > config_.imu_timeout_sec ||
        !inputs.imu_healthy || inputs.ekf_diverged) {
        return false;
    }

    if (inputs.gnss_innovation_test_ratio > config_.innovation_gate_ratio ||
        inputs.ekf_innovation_test_ratio > config_.innovation_gate_ratio) {
        return false;
    }

    // RTK Float or RTK Fixed with slightly relaxed accuracy bounds
    if (inputs.gnss_fix != GnssFixType::RtkFloat && inputs.gnss_fix != GnssFixType::RtkFixed) {
        return false;
    }
    if (inputs.gnss_satellites < config_.min_satellites_rtk_float) {
        return false;
    }
    if (inputs.gnss_eph > config_.max_eph_rtk_float) {
        return false;
    }

    return true;
}

bool DegradedNavigationFsm::checkTier2Health(const NavigationHealthInputs& inputs) const {
    // VIO / LIO Odometry
    if (vio_dropout_timer_sec_ > config_.vio_timeout_sec ||
        imu_dropout_timer_sec_ > config_.imu_timeout_sec ||
        !inputs.imu_healthy || inputs.ekf_diverged) {
        return false;
    }

    if (inputs.vio_innovation_test_ratio > config_.innovation_gate_ratio ||
        inputs.ekf_innovation_test_ratio > config_.innovation_gate_ratio) {
        return false;
    }

    if (!inputs.vio_tracking_valid) {
        return false;
    }
    if (inputs.vio_tracked_features < config_.min_vio_features) {
        return false;
    }
    if (inputs.vio_confidence < config_.min_vio_confidence) {
        return false;
    }

    return true;
}

bool DegradedNavigationFsm::checkTier3Health(const NavigationHealthInputs& inputs) const {
    // Dead reckoning requires working IMU and accumulated duration within budget
    if (imu_dropout_timer_sec_ > config_.imu_timeout_sec ||
        !inputs.imu_healthy || inputs.ekf_diverged) {
        return false;
    }

    if (dead_reckoning_elapsed_sec_ >= config_.max_dead_reckoning_sec) {
        return false; // Dead reckoning time budget exhausted
    }

    return true;
}

NavigationQualityTier DegradedNavigationFsm::evaluateHighestHealthyTier(
    const NavigationHealthInputs& inputs) const
{
    if (checkTier0Health(inputs)) {
        return NavigationQualityTier::Tier0_RtkFixed;
    }
    if (checkTier1Health(inputs)) {
        return NavigationQualityTier::Tier1_RtkFloat;
    }
    if (checkTier2Health(inputs)) {
        return NavigationQualityTier::Tier2_VioLioOdometry;
    }
    if (current_tier_ != NavigationQualityTier::Tier4_EmergencyDescendLand && checkTier3Health(inputs)) {
        return NavigationQualityTier::Tier3_InertialDragDeadReckoning;
    }
    return NavigationQualityTier::Tier4_EmergencyDescendLand;
}

void DegradedNavigationFsm::executeTransition(NavigationQualityTier target_tier, const std::string& reason) {
    previous_tier_ = current_tier_;
    current_tier_ = target_tier;
    latest_transition_reason_ = reason;
    time_in_current_tier_sec_ = 0.0f;
    candidate_promotion_tier_ = target_tier;
    promotion_dwell_timer_sec_ = 0.0f;

    if (current_tier_ != NavigationQualityTier::Tier3_InertialDragDeadReckoning &&
        current_tier_ != NavigationQualityTier::Tier4_EmergencyDescendLand) {
        dead_reckoning_elapsed_sec_ = 0.0f;
    }
}

NavigationStatusReport DegradedNavigationFsm::update(const NavigationHealthInputs& inputs, float dt) {
    const float dt_clamped = std::max(0.0001f, dt);
    TierTransitionCallback cb_to_invoke = nullptr;
    NavigationQualityTier old_tier = NavigationQualityTier::Tier0_RtkFixed;
    NavigationQualityTier new_tier = NavigationQualityTier::Tier0_RtkFixed;
    std::string cb_reason;

    {
        std::lock_guard<std::mutex> lock(mutex_);

        // 1. Update sensor dropout watchdog timers
        if (inputs.gnss_received) {
            gnss_dropout_timer_sec_ = 0.0f;
        } else {
            gnss_dropout_timer_sec_ += dt_clamped;
        }

        if (inputs.vio_received) {
            vio_dropout_timer_sec_ = 0.0f;
        } else {
            vio_dropout_timer_sec_ += dt_clamped;
        }

        if (inputs.imu_healthy) {
            imu_dropout_timer_sec_ = 0.0f;
        } else {
            imu_dropout_timer_sec_ += dt_clamped;
        }

        time_in_current_tier_sec_ += dt_clamped;

        // 2. Evaluate active tier health
        bool current_tier_healthy = false;
        std::string demotion_cause;

        switch (current_tier_) {
            case NavigationQualityTier::Tier0_RtkFixed:
                current_tier_healthy = checkTier0Health(inputs);
                if (!current_tier_healthy) {
                    if (inputs.gnss_innovation_test_ratio > config_.innovation_gate_ratio) {
                        demotion_cause = "GNSS innovation test ratio rejected (> " +
                                         std::to_string(config_.innovation_gate_ratio) + ")";
                    } else if (gnss_dropout_timer_sec_ > config_.gnss_timeout_sec) {
                        demotion_cause = "GNSS signal dropout watchdog timeout (" +
                                         std::to_string(gnss_dropout_timer_sec_) + "s)";
                    } else if (inputs.gnss_fix != GnssFixType::RtkFixed) {
                        demotion_cause = "GNSS lost RTK Fixed fix (fix=" +
                                         std::to_string(static_cast<int>(inputs.gnss_fix)) + ")";
                    } else if (inputs.gnss_eph > config_.max_eph_rtk_fixed) {
                        demotion_cause = "GNSS EPH degraded (" + std::to_string(inputs.gnss_eph) + "m)";
                    } else {
                        demotion_cause = "Tier 0 health criteria failed";
                    }
                }
                break;

            case NavigationQualityTier::Tier1_RtkFloat:
                current_tier_healthy = checkTier1Health(inputs);
                if (!current_tier_healthy) {
                    if (inputs.gnss_innovation_test_ratio > config_.innovation_gate_ratio) {
                        demotion_cause = "GNSS innovation test ratio rejected in Tier 1";
                    } else if (gnss_dropout_timer_sec_ > config_.gnss_timeout_sec) {
                        demotion_cause = "GNSS dropout timeout in Tier 1";
                    } else {
                        demotion_cause = "Tier 1 RTK Float degraded/lost";
                    }
                }
                break;

            case NavigationQualityTier::Tier2_VioLioOdometry:
                current_tier_healthy = checkTier2Health(inputs);
                if (!current_tier_healthy) {
                    if (inputs.vio_innovation_test_ratio > config_.innovation_gate_ratio) {
                        demotion_cause = "VIO innovation test ratio rejected (> " +
                                         std::to_string(config_.innovation_gate_ratio) + ")";
                    } else if (vio_dropout_timer_sec_ > config_.vio_timeout_sec) {
                        demotion_cause = "VIO odometry stream dropout timeout";
                    } else if (!inputs.vio_tracking_valid) {
                        demotion_cause = "VIO visual tracking lost / invalidated";
                    } else {
                        demotion_cause = "Tier 2 VIO/LIO quality criteria failed";
                    }
                }
                break;

            case NavigationQualityTier::Tier3_InertialDragDeadReckoning:
                dead_reckoning_elapsed_sec_ += dt_clamped;
                current_tier_healthy = checkTier3Health(inputs);
                if (!current_tier_healthy) {
                    if (dead_reckoning_elapsed_sec_ >= config_.max_dead_reckoning_sec) {
                        demotion_cause = "Dead reckoning time budget exceeded (" +
                                         std::to_string(dead_reckoning_elapsed_sec_) + "s >= " +
                                         std::to_string(config_.max_dead_reckoning_sec) + "s)";
                    } else {
                        demotion_cause = "IMU failure / divergence during dead reckoning";
                    }
                }
                break;

            case NavigationQualityTier::Tier4_EmergencyDescendLand:
                // Already in terminal failsafe; remains in failsafe unless manually reset
                // or highest healthy tier promotes via sustained hysteresis
                current_tier_healthy = true;
                break;
        }

        // 3. Catastrophic override: EKF divergence or IMU failure immediately triggers Tier 4
        if (inputs.ekf_diverged || imu_dropout_timer_sec_ > config_.imu_timeout_sec || !inputs.imu_healthy) {
            if (current_tier_ != NavigationQualityTier::Tier4_EmergencyDescendLand) {
                old_tier = current_tier_;
                executeTransition(NavigationQualityTier::Tier4_EmergencyDescendLand,
                                  "EMERGENCY: EKF divergence or IMU failure detected!");
                new_tier = current_tier_;
                cb_reason = latest_transition_reason_;
                cb_to_invoke = transition_callback_;
            }
        }
        // 4. Automatic Demotion (Immediate on degradation)
        else if (!current_tier_healthy) {
            const NavigationQualityTier fallback = evaluateHighestHealthyTier(inputs);
            old_tier = current_tier_;
            executeTransition(fallback, "Demotion: " + demotion_cause + " -> " + tierToString(fallback));
            new_tier = current_tier_;
            cb_reason = latest_transition_reason_;
            cb_to_invoke = transition_callback_;
        }
        // 5. Automatic Promotion with Hysteresis (> 2.0s continuous healthy criteria)
        else {
            const NavigationQualityTier best_available = evaluateHighestHealthyTier(inputs);

            // Check if a superior tier is available (numerically lower tier enum)
            if (static_cast<uint8_t>(best_available) < static_cast<uint8_t>(current_tier_)) {
                if (best_available == candidate_promotion_tier_) {
                    promotion_dwell_timer_sec_ += dt_clamped;

                    if (promotion_dwell_timer_sec_ >= config_.promotion_hysteresis_sec) {
                        old_tier = current_tier_;
                        std::ostringstream ss;
                        ss << "Promoted: Sustained " << tierToString(best_available)
                           << " health for " << std::fixed << std::setprecision(2)
                           << promotion_dwell_timer_sec_ << "s (hysteresis passed)";
                        executeTransition(best_available, ss.str());
                        new_tier = current_tier_;
                        cb_reason = latest_transition_reason_;
                        cb_to_invoke = transition_callback_;
                    }
                } else {
                    candidate_promotion_tier_ = best_available;
                    promotion_dwell_timer_sec_ = 0.0f;
                }
            } else {
                candidate_promotion_tier_ = current_tier_;
                promotion_dwell_timer_sec_ = 0.0f;
            }
        }

        // 6. Build updated status report
        latest_report_.current_tier = current_tier_;
        latest_report_.previous_tier = previous_tier_;
        latest_report_.tier_name = tierToString(current_tier_);
        latest_report_.transition_reason = latest_transition_reason_;
        latest_report_.time_in_current_tier_sec = time_in_current_tier_sec_;
        latest_report_.dead_reckoning_elapsed_sec = dead_reckoning_elapsed_sec_;
        latest_report_.promotion_dwell_time_sec = promotion_dwell_timer_sec_;

        latest_report_.is_rtk_fixed = (current_tier_ == NavigationQualityTier::Tier0_RtkFixed);
        latest_report_.is_rtk_float = (current_tier_ == NavigationQualityTier::Tier1_RtkFloat);
        latest_report_.is_vio_active = (current_tier_ == NavigationQualityTier::Tier2_VioLioOdometry);
        latest_report_.is_dead_reckoning = (current_tier_ == NavigationQualityTier::Tier3_InertialDragDeadReckoning);
        latest_report_.is_failsafe_active = (current_tier_ == NavigationQualityTier::Tier4_EmergencyDescendLand);
        latest_report_.is_gps_denied = (current_tier_ >= NavigationQualityTier::Tier2_VioLioOdometry);

        latest_report_.max_allowable_speed = getMaxSpeedForTier(current_tier_);
        latest_report_.commanded_vz_failsafe = (current_tier_ == NavigationQualityTier::Tier4_EmergencyDescendLand)
                                                   ? config_.emergency_descent_vel_z
                                                   : 0.0f;

        latest_report_.gnss_healthy = checkTier0Health(inputs) || checkTier1Health(inputs);
        latest_report_.vio_healthy = checkTier2Health(inputs);
        latest_report_.imu_healthy = inputs.imu_healthy && (imu_dropout_timer_sec_ <= config_.imu_timeout_sec);
        latest_report_.latest_gnss_innovation_ratio = inputs.gnss_innovation_test_ratio;
        latest_report_.latest_vio_innovation_ratio = inputs.vio_innovation_test_ratio;
    }

    // Invoke transition callback outside mutex to prevent deadlocks
    if (cb_to_invoke != nullptr) {
        cb_to_invoke(old_tier, new_tier, cb_reason);
    }

    return latest_report_;
}

NavigationStatusReport DegradedNavigationFsm::getStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_report_;
}

} // namespace production
} // namespace px4_airsim_autonomy
