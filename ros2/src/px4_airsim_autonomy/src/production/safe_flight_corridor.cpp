#include "px4_airsim_autonomy/production/safe_flight_corridor.hpp"

#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace px4_airsim_autonomy {
namespace production {

namespace {
constexpr float kEpsilon = 1e-4f;
constexpr float kPi = 3.14159265358979323846f;
} // namespace

// ============================================================================
// CorridorSlice Implementation
// ============================================================================

void CorridorSlice::assembleMatrixRepresentation() {
    const size_t num_planes = hyperplanes.size();
    A_k.resize(static_cast<Eigen::Index>(num_planes), 3);
    b_k.resize(static_cast<Eigen::Index>(num_planes));

    for (size_t i = 0; i < num_planes; ++i) {
        A_k.row(static_cast<Eigen::Index>(i)) = hyperplanes[i].normal.transpose();
        b_k(static_cast<Eigen::Index>(i)) = hyperplanes[i].offset;
    }
}

bool CorridorSlice::contains(const Eigen::Vector3f& point, float tolerance) const {
    if (A_k.rows() > 0) {
        const Eigen::VectorXf residual = A_k * point - b_k;
        return (residual.array() <= tolerance).all();
    }

    for (const auto& plane : hyperplanes) {
        if (!plane.contains(point, tolerance)) {
            return false;
        }
    }
    return true;
}

float CorridorSlice::computeMaxViolation(const Eigen::Vector3f& point) const {
    if (hyperplanes.empty()) {
        return 0.0f;
    }

    float max_viol = -std::numeric_limits<float>::infinity();
    for (const auto& plane : hyperplanes) {
        max_viol = std::max(max_viol, plane.signedDistance(point));
    }
    return max_viol;
}

// ============================================================================
// PolynomialSegment3D Implementation
// ============================================================================

PolynomialSegment3D::PolynomialSegment3D(float duration, const Eigen::Matrix<float, 3, 6>& coeffs)
    : duration_(std::max(kEpsilon, duration)), coeffs_(coeffs) {}

PolynomialSegment3D PolynomialSegment3D::solveMinimumJerkQuintic(
    const Eigen::Vector3f& p0, const Eigen::Vector3f& v0, const Eigen::Vector3f& a0,
    const Eigen::Vector3f& p1, const Eigen::Vector3f& v1, const Eigen::Vector3f& a1,
    float duration) {
    const float T = std::max(kEpsilon, duration);
    const float T2 = T * T;
    const float T3 = T2 * T;
    const float T4 = T3 * T;
    const float T5 = T4 * T;

    Eigen::Matrix<float, 3, 6> C = Eigen::Matrix<float, 3, 6>::Zero();

    // Boundary conditions at t = 0:
    // p(0) = c0 => c0 = p0
    // v(0) = c1 => c1 = v0
    // a(0) = 2*c2 => c2 = 0.5 * a0
    C.col(0) = p0;
    C.col(1) = v0;
    C.col(2) = 0.5f * a0;

    // Remaining boundary discrepancies at t = T
    const Eigen::Vector3f delta_p = p1 - (p0 + v0 * T + 0.5f * a0 * T2);
    const Eigen::Vector3f delta_v = v1 - (v0 + a0 * T);
    const Eigen::Vector3f delta_a = a1 - a0;

    // Analytical closed-form solution to linear system for [c3, c4, c5]
    C.col(3) = (10.0f * delta_p) / T3 - (4.0f * delta_v) / T2 + (0.5f * delta_a) / T;
    C.col(4) = (-15.0f * delta_p) / T4 + (7.0f * delta_v) / T3 - (1.0f * delta_a) / T2;
    C.col(5) = (6.0f * delta_p) / T5 - (3.0f * delta_v) / T4 + (0.5f * delta_a) / T3;

    return PolynomialSegment3D(T, C);
}

Eigen::Vector3f PolynomialSegment3D::evaluatePosition(float t) const {
    const float clamped_t = std::clamp(t, 0.0f, duration_);
    const float t2 = clamped_t * clamped_t;
    const float t3 = t2 * clamped_t;
    const float t4 = t3 * clamped_t;
    const float t5 = t4 * clamped_t;

    return coeffs_.col(0) +
           coeffs_.col(1) * clamped_t +
           coeffs_.col(2) * t2 +
           coeffs_.col(3) * t3 +
           coeffs_.col(4) * t4 +
           coeffs_.col(5) * t5;
}

Eigen::Vector3f PolynomialSegment3D::evaluateVelocity(float t) const {
    const float clamped_t = std::clamp(t, 0.0f, duration_);
    const float t2 = clamped_t * clamped_t;
    const float t3 = t2 * clamped_t;
    const float t4 = t3 * clamped_t;

    return coeffs_.col(1) +
           2.0f * coeffs_.col(2) * clamped_t +
           3.0f * coeffs_.col(3) * t2 +
           4.0f * coeffs_.col(4) * t3 +
           5.0f * coeffs_.col(5) * t4;
}

Eigen::Vector3f PolynomialSegment3D::evaluateAcceleration(float t) const {
    const float clamped_t = std::clamp(t, 0.0f, duration_);
    const float t2 = clamped_t * clamped_t;
    const float t3 = t2 * clamped_t;

    return 2.0f * coeffs_.col(2) +
           6.0f * coeffs_.col(3) * clamped_t +
           12.0f * coeffs_.col(4) * t2 +
           20.0f * coeffs_.col(5) * t3;
}

Eigen::Vector3f PolynomialSegment3D::evaluateJerk(float t) const {
    const float clamped_t = std::clamp(t, 0.0f, duration_);
    const float t2 = clamped_t * clamped_t;

    return 6.0f * coeffs_.col(3) +
           24.0f * coeffs_.col(4) * clamped_t +
           60.0f * coeffs_.col(5) * t2;
}

Eigen::Vector3f PolynomialSegment3D::evaluateSnap(float t) const {
    const float clamped_t = std::clamp(t, 0.0f, duration_);

    return 24.0f * coeffs_.col(4) +
           120.0f * coeffs_.col(5) * clamped_t;
}

Eigen::Vector3f PolynomialSegment3D::evaluateTransverseJerk(
    float t, const Eigen::Vector3f& gravity) const {
    const Eigen::Vector3f a = evaluateAcceleration(t);
    const Eigen::Vector3f j = evaluateJerk(t);

    // Multirotor total thrust vector in inertial ENU frame: f_thrust = m * (a - g)
    const Eigen::Vector3f f_dir = a - gravity;
    const float f_norm = f_dir.norm();

    if (f_norm > kEpsilon) {
        const Eigen::Vector3f z_B = f_dir / f_norm;
        // Transverse component is orthogonal to thrust axis: j_perp = j - (j . z_B) * z_B
        const float j_parallel = j.dot(z_B);
        return j - j_parallel * z_B;
    }

    return j;
}

// ============================================================================
// SafeFlightCorridor Implementation
// ============================================================================

SafeFlightCorridor::SafeFlightCorridor(const SafeFlightCorridorConfig& config)
    : config_(config) {}

std::vector<CorridorSlice> SafeFlightCorridor::generateCorridor(
    const std::vector<Eigen::Vector3f>& waypoints,
    const std::vector<Eigen::Vector3f>& obstacle_points) const {
    std::vector<CorridorSlice> slices;

    if (waypoints.size() < 2) {
        return slices;
    }

    const size_t num_segments = waypoints.size() - 1;
    slices.reserve(num_segments);

    for (size_t k = 0; k < num_segments; ++k) {
        CorridorSlice slice;
        slice.slice_index = static_cast<int>(k);
        slice.start_waypoint = waypoints[k];
        slice.end_waypoint = waypoints[k + 1];

        const Eigen::Vector3f seg_vec = slice.end_waypoint - slice.start_waypoint;
        const float seg_len = seg_vec.norm();
        if (seg_len < kEpsilon) {
            continue;
        }

        const Eigen::Vector3f t_hat = seg_vec / seg_len;

        // Build transverse orthonormal frame {u_hat, v_hat} perpendicular to t_hat
        Eigen::Vector3f ref = Eigen::Vector3f::UnitZ();
        if (std::abs(t_hat.z()) > 0.90f) {
            ref = Eigen::Vector3f::UnitY();
        }
        const Eigen::Vector3f u_hat = (t_hat.cross(ref)).normalized();
        const Eigen::Vector3f v_hat = (t_hat.cross(u_hat)).normalized();

        // 1. Start endcap plane: -t_hat^T (x - (w_k - overlap * t_hat)) <= 0
        const Eigen::Vector3f p_start = slice.start_waypoint - config_.overlap_margin * t_hat;
        slice.hyperplanes.emplace_back(p_start, -t_hat);

        // 2. End endcap plane: +t_hat^T (x - (w_{k+1} + overlap * t_hat)) <= 0
        const Eigen::Vector3f p_end = slice.end_waypoint + config_.overlap_margin * t_hat;
        slice.hyperplanes.emplace_back(p_end, t_hat);

        // 3. Radial bounding planes around segment axis
        const int K = std::max(4, config_.num_radial_planes);
        const float angle_step = (2.0f * kPi) / static_cast<float>(K);

        for (int m = 0; m < K; ++m) {
            const float angle = static_cast<float>(m) * angle_step;
            const Eigen::Vector3f n_radial = (std::cos(angle) * u_hat + std::sin(angle) * v_hat).normalized();

            float allowed_clearance = config_.default_clearance;

            // Constrain by nearby obstacles within segment longitudinal projection
            for (const auto& obs : obstacle_points) {
                const Eigen::Vector3f diff = obs - slice.start_waypoint;
                const float s = diff.dot(t_hat);

                // Check if obstacle is along this corridor segment
                if (s >= -config_.overlap_margin && s <= seg_len + config_.overlap_margin) {
                    const Eigen::Vector3f radial_disp = diff - s * t_hat;
                    const float obs_radial_proj = radial_disp.dot(n_radial);

                    if (obs_radial_proj > 0.0f) {
                        const float obs_clearance = std::max(
                            config_.min_clearance,
                            obs_radial_proj - config_.obstacle_safety_margin);
                        allowed_clearance = std::min(allowed_clearance, obs_clearance);
                    }
                }
            }

            // Reference point on boundary plane: midway along segment + allowed_clearance * n_radial
            const Eigen::Vector3f p_plane = slice.start_waypoint + 0.5f * seg_len * t_hat + allowed_clearance * n_radial;
            slice.hyperplanes.emplace_back(p_plane, n_radial);
        }

        slice.assembleMatrixRepresentation();
        slices.push_back(slice);
    }

    return slices;
}

JerkCheckResult SafeFlightCorridor::checkJerkLimits(
    const PolynomialSegment3D& segment,
    float j_max,
    int num_samples) const {
    JerkCheckResult result;
    result.enforced_j_max = std::min(j_max, config_.max_transverse_jerk);
    result.satisfies_limit = true;

    const float duration = segment.getDuration();
    const int N = std::max(10, num_samples);
    const float dt = duration / static_cast<float>(N - 1);

    for (int i = 0; i < N; ++i) {
        const float t = static_cast<float>(i) * dt;
        const Eigen::Vector3f a = segment.evaluateAcceleration(t);
        const Eigen::Vector3f j_total = segment.evaluateJerk(t);
        const Eigen::Vector3f j_perp = segment.evaluateTransverseJerk(t);

        const float a_norm = a.norm();
        const float j_total_norm = j_total.norm();
        const float j_perp_norm = j_perp.norm();

        if (a_norm > result.max_acceleration) {
            result.max_acceleration = a_norm;
        }
        if (j_total_norm > result.max_total_jerk) {
            result.max_total_jerk = j_total_norm;
        }
        if (j_perp_norm > result.max_transverse_jerk) {
            result.max_transverse_jerk = j_perp_norm;
            result.time_of_peak_transverse_jerk = t;
        }

        // Enforce transverse jerk limit: prevents rate-gyro saturation & motor desync
        if (j_perp_norm > result.enforced_j_max) {
            result.satisfies_limit = false;
        }
        if (j_total_norm > config_.max_total_jerk) {
            result.satisfies_limit = false;
        }
    }

    std::ostringstream oss;
    if (result.satisfies_limit) {
        oss << "PASS: Transverse jerk " << std::fixed << std::setprecision(2)
            << result.max_transverse_jerk << " m/s^3 <= " << result.enforced_j_max
            << " m/s^3 limit. Total jerk: " << result.max_total_jerk
            << " m/s^3, Max Acc: " << result.max_acceleration << " m/s^2.";
    } else {
        oss << "VIOLATION: Transverse jerk " << std::fixed << std::setprecision(2)
            << result.max_transverse_jerk << " m/s^3 exceeds limit "
            << result.enforced_j_max << " m/s^3 at t=" << result.time_of_peak_transverse_jerk
            << " s! Risk of rate-gyro saturation and motor desync.";
    }
    result.diagnostics = oss.str();

    return result;
}

bool SafeFlightCorridor::verifyTrajectoryInCorridor(
    const PolynomialSegment3D& segment,
    const CorridorSlice& slice,
    int num_samples,
    float tolerance) const {
    const float duration = segment.getDuration();
    const int N = std::max(5, num_samples);
    const float dt = duration / static_cast<float>(N - 1);

    for (int i = 0; i < N; ++i) {
        const float t = static_cast<float>(i) * dt;
        const Eigen::Vector3f p = segment.evaluatePosition(t);
        if (!slice.contains(p, tolerance)) {
            return false;
        }
    }
    return true;
}

} // namespace production
} // namespace px4_airsim_autonomy
