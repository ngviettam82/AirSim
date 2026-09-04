#include "px4_airsim_autonomy/production/dynamic_stopping_bubble.hpp"

#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace px4_airsim_autonomy {
namespace production {

namespace {
constexpr float kEpsilon = 1e-4f;
} // namespace

DynamicStoppingBubble::DynamicStoppingBubble(const DynamicStoppingBubbleConfig& config)
    : config_(config) {
    // Initialize default static bubble at origin
    update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
}

float DynamicStoppingBubble::computeStoppingDistance(float speed) const {
    const float safe_a_max = std::max(kEpsilon, config_.a_max);
    const float non_neg_speed = std::max(0.0f, speed);
    // Quadratic stopping formula: d_stop(v) = v^2 / (2 * a_max) + v * t_latency + d_margin
    return (non_neg_speed * non_neg_speed) / (2.0f * safe_a_max) +
           (non_neg_speed * config_.t_latency) +
           config_.d_margin;
}

float DynamicStoppingBubble::computeStoppingDistance(const Eigen::Vector3f& velocity) const {
    return computeStoppingDistance(velocity.norm());
}

float DynamicStoppingBubble::computeTimeToStop(float speed) const {
    const float safe_a_max = std::max(kEpsilon, config_.a_max);
    const float non_neg_speed = std::max(0.0f, speed);
    return (non_neg_speed / safe_a_max) + config_.t_latency;
}

Eigen::Matrix3f DynamicStoppingBubble::buildOrthonormalBasis(const Eigen::Vector3f& unit_v) {
    Eigen::Matrix3f R;
    const Eigen::Vector3f u_x = unit_v;

    // Select reference vector not parallel to u_x
    Eigen::Vector3f ref = Eigen::Vector3f::UnitZ();
    if (std::abs(u_x.z()) > 0.90f) {
        ref = Eigen::Vector3f::UnitY();
    }

    // Gram-Schmidt orthogonalization
    Eigen::Vector3f u_y = (u_x.cross(ref)).normalized();
    Eigen::Vector3f u_z = (u_x.cross(u_y)).normalized();

    R.col(0) = u_x;
    R.col(1) = u_y;
    R.col(2) = u_z;
    return R;
}

void DynamicStoppingBubble::update(const Eigen::Vector3f& vehicle_position, const Eigen::Vector3f& vehicle_velocity) {
    vehicle_position_ = vehicle_position;
    vehicle_velocity_ = vehicle_velocity;
    current_speed_ = vehicle_velocity.norm();

    if (current_speed_ > config_.min_velocity_threshold) {
        const Eigen::Vector3f v_hat = vehicle_velocity_ / current_speed_;
        const float d_stop = computeStoppingDistance(current_speed_);

        // Longitudinal semi-axis encompasses vehicle footprint behind and d_stop ahead
        const float a = 0.5f * (d_stop + config_.d_margin);
        // Center offset along velocity vector
        const float center_offset = 0.5f * (d_stop - config_.d_margin);
        center_ = vehicle_position_ + center_offset * v_hat;

        // Dynamic expansion of lateral and vertical clearances with speed
        const float b = config_.r_lateral + config_.k_lateral_vel * current_speed_;
        const float c = config_.r_vertical + config_.k_vertical_vel * std::abs(vehicle_velocity_.z());

        semi_axes_ = Eigen::Vector3f(a, b, c);
        orientation_ = buildOrthonormalBasis(v_hat);
    } else {
        // Stationary or near-zero velocity: symmetric safety bubble centered at vehicle
        center_ = vehicle_position_;
        semi_axes_ = Eigen::Vector3f(config_.d_margin, config_.r_lateral, config_.r_vertical);
        orientation_ = Eigen::Matrix3f::Identity();
    }

    // Precompute ellipsoid inverse metric tensor M = R * diag(1/a^2, 1/b^2, 1/c^2) * R^T
    const float inv_a2 = 1.0f / (semi_axes_.x() * semi_axes_.x());
    const float inv_b2 = 1.0f / (semi_axes_.y() * semi_axes_.y());
    const float inv_c2 = 1.0f / (semi_axes_.z() * semi_axes_.z());

    const Eigen::Vector3f inv_diag(inv_a2, inv_b2, inv_c2);
    shape_matrix_m_ = orientation_ * inv_diag.asDiagonal() * orientation_.transpose();
}

bool DynamicStoppingBubble::isPointInside(const Eigen::Vector3f& point) const {
    const Eigen::Vector3f delta = point - center_;
    const float quadric_val = delta.transpose() * shape_matrix_m_ * delta;
    return quadric_val <= 1.0f;
}

float DynamicStoppingBubble::computeNormalizedDistance(const Eigen::Vector3f& point) const {
    const Eigen::Vector3f delta = point - center_;
    const float quadric_val = delta.transpose() * shape_matrix_m_ * delta;
    return std::sqrt(std::max(0.0f, quadric_val));
}

Eigen::Vector3f DynamicStoppingBubble::computeBrakingAcceleration() const {
    return computeBrakingAcceleration(vehicle_velocity_);
}

Eigen::Vector3f DynamicStoppingBubble::computeBrakingAcceleration(const Eigen::Vector3f& velocity) const {
    const float speed = velocity.norm();
    if (speed < config_.min_velocity_threshold) {
        return Eigen::Vector3f::Zero();
    }
    // Emergency braking deceleration opposite to instantaneous velocity vector
    return -config_.a_max * (velocity / speed);
}

Eigen::Vector3f DynamicStoppingBubble::computeBrakingVelocityStep(float dt) const {
    if (current_speed_ < config_.min_velocity_threshold) {
        return Eigen::Vector3f::Zero();
    }

    const Eigen::Vector3f a_brake = computeBrakingAcceleration();
    Eigen::Vector3f next_vel = vehicle_velocity_ + a_brake * dt;

    // Check if braking overshoots zero velocity (dot product reverses sign)
    if (next_vel.dot(vehicle_velocity_) <= 0.0f) {
        return Eigen::Vector3f::Zero();
    }

    return next_vel;
}

BubbleViolation DynamicStoppingBubble::checkObstacles(const std::vector<Eigen::Vector3f>& obstacle_points) const {
    BubbleViolation violation;

    if (obstacle_points.empty()) {
        violation.description = "No obstacles in range.";
        return violation;
    }

    for (const auto& pt : obstacle_points) {
        const float norm_dist = computeNormalizedDistance(pt);
        if (norm_dist < violation.min_normalized_distance) {
            violation.min_normalized_distance = norm_dist;
            violation.critical_obstacle_point = pt;
            violation.relative_obstacle_vector = pt - vehicle_position_;
            violation.obstacle_distance = violation.relative_obstacle_vector.norm();

            if (current_speed_ > config_.min_velocity_threshold) {
                const float speed_along_rel = violation.relative_obstacle_vector.dot(vehicle_velocity_) / violation.obstacle_distance;
                if (speed_along_rel > kEpsilon) {
                    violation.time_to_impact = violation.obstacle_distance / speed_along_rel;
                } else {
                    violation.time_to_impact = std::numeric_limits<float>::infinity();
                }
            } else {
                violation.time_to_impact = std::numeric_limits<float>::infinity();
            }
        }
    }

    if (violation.min_normalized_distance < 1.0f) {
        violation.is_violated = true;
        // Estimate geometric penetration depth along vector from ellipsoid center
        const Eigen::Vector3f delta = violation.critical_obstacle_point - center_;
        const float dist_from_center = delta.norm();
        if (violation.min_normalized_distance > kEpsilon && dist_from_center > kEpsilon) {
            const float boundary_dist = dist_from_center / violation.min_normalized_distance;
            violation.max_penetration_depth = std::max(0.0f, boundary_dist - dist_from_center);
        } else {
            violation.max_penetration_depth = semi_axes_.x();
        }

        std::ostringstream oss;
        oss << "EMERGENCY: Dynamic stopping bubble penetrated by obstacle at "
            << "[" << std::fixed << std::setprecision(2)
            << violation.critical_obstacle_point.x() << ", "
            << violation.critical_obstacle_point.y() << ", "
            << violation.critical_obstacle_point.z() << "] m. "
            << "NormDist=" << std::setprecision(3) << violation.min_normalized_distance
            << ", PenDepth=" << std::setprecision(2) << violation.max_penetration_depth << " m.";
        violation.description = oss.str();
    } else {
        violation.is_violated = false;
        std::ostringstream oss;
        oss << "CLEAR: Dynamic stopping bubble clear. MinNormDist="
            << std::fixed << std::setprecision(3) << violation.min_normalized_distance;
        violation.description = oss.str();
    }

    return violation;
}

BubbleViolation DynamicStoppingBubble::checkSensorSnapshot(const SensorSnapshot& snapshot) const {
    // Synthesize 3D obstacle query points from sensor snapshot sectors in world ENU frame
    std::vector<Eigen::Vector3f> candidate_points;
    candidate_points.reserve(10);

    const float yaw = snapshot.yaw;
    const Eigen::Vector3f forward_dir(std::cos(yaw), std::sin(yaw), 0.0f);
    const Eigen::Vector3f left_dir(-std::sin(yaw), std::cos(yaw), 0.0f);
    const Eigen::Vector3f up_dir(0.0f, 0.0f, 1.0f);

    // Forward center sector
    if (snapshot.center_sector_distance < 50.0f) {
        candidate_points.push_back(vehicle_position_ + forward_dir * snapshot.center_sector_distance);
    }

    // Left sector
    if (snapshot.left_sector_distance < 50.0f) {
        const Eigen::Vector3f left_45 = (forward_dir + left_dir).normalized();
        candidate_points.push_back(vehicle_position_ + left_45 * snapshot.left_sector_distance);
    }

    // Right sector
    if (snapshot.right_sector_distance < 50.0f) {
        const Eigen::Vector3f right_45 = (forward_dir - left_dir).normalized();
        candidate_points.push_back(vehicle_position_ + right_45 * snapshot.right_sector_distance);
    }

    // Upper sector
    if (snapshot.upper_sector_distance < 50.0f) {
        const Eigen::Vector3f up_forward = (forward_dir + 0.5f * up_dir).normalized();
        candidate_points.push_back(vehicle_position_ + up_forward * snapshot.upper_sector_distance);
    }

    // Lower sector
    if (snapshot.lower_sector_distance < 50.0f) {
        const Eigen::Vector3f down_forward = (forward_dir - 0.5f * up_dir).normalized();
        candidate_points.push_back(vehicle_position_ + down_forward * snapshot.lower_sector_distance);
    }

    // Nearest obstacle in overall depth field
    if (snapshot.min_depth_distance < 50.0f) {
        candidate_points.push_back(vehicle_position_ + forward_dir * snapshot.min_depth_distance);
    }

    return checkObstacles(candidate_points);
}

bool DynamicStoppingBubble::isSphereColliding(const Eigen::Vector3f& sphere_center, float sphere_radius) const {
    // Minkowski sum approximation: expand semi-axes by sphere radius
    const Eigen::Vector3f expanded_axes = semi_axes_ + Eigen::Vector3f::Constant(std::max(0.0f, sphere_radius));
    const float inv_a2 = 1.0f / (expanded_axes.x() * expanded_axes.x());
    const float inv_b2 = 1.0f / (expanded_axes.y() * expanded_axes.y());
    const float inv_c2 = 1.0f / (expanded_axes.z() * expanded_axes.z());

    const Eigen::Vector3f inv_diag(inv_a2, inv_b2, inv_c2);
    const Eigen::Matrix3f expanded_m = orientation_ * inv_diag.asDiagonal() * orientation_.transpose();

    const Eigen::Vector3f delta = sphere_center - center_;
    const float quadric_val = delta.transpose() * expanded_m * delta;
    return quadric_val <= 1.0f;
}

} // namespace production
} // namespace px4_airsim_autonomy
