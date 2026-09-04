#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <limits>
#include <memory>

namespace px4_airsim_autonomy {
namespace production {

/**
 * @brief Representation of an outward-pointing bounding halfspace plane: n^T x <= b.
 */
struct Hyperplane3D {
    /// Unit outward normal vector (||normal|| = 1.0).
    Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();

    /// Scalar offset such that normal.dot(x) <= offset is safe / inside.
    float offset = 0.0f;

    Hyperplane3D() = default;
    Hyperplane3D(const Eigen::Vector3f& n, float b)
        : normal(n.normalized()), offset(b) {}

    Hyperplane3D(const Eigen::Vector3f& plane_point, const Eigen::Vector3f& outward_normal)
        : normal(outward_normal.normalized()), offset(normal.dot(plane_point)) {}

    /**
     * @brief Compute signed distance to plane: positive is outside, negative is inside.
     */
    float signedDistance(const Eigen::Vector3f& point) const {
        return normal.dot(point) - offset;
    }

    /**
     * @brief Check if point lies inside the halfspace (n^T x <= b + tolerance).
     */
    bool contains(const Eigen::Vector3f& point, float tolerance = 1e-4f) const {
        return signedDistance(point) <= tolerance;
    }
};

/**
 * @brief Convex polyhedral corridor slice represented as C_k = { x in R^3 | A_k x <= b_k }.
 */
struct CorridorSlice {
    /// Slice index along trajectory.
    int slice_index = 0;

    /// Starting waypoint of this segment.
    Eigen::Vector3f start_waypoint = Eigen::Vector3f::Zero();

    /// Ending waypoint of this segment.
    Eigen::Vector3f end_waypoint = Eigen::Vector3f::Zero();

    /// Individual bounding halfspace planes.
    std::vector<Hyperplane3D> hyperplanes;

    /// Polyhedral matrix representation A_k (M x 3).
    Eigen::MatrixX3f A_k;

    /// Polyhedral vector representation b_k (M x 1).
    Eigen::VectorXf b_k;

    /**
     * @brief Assemble the Eigen matrix A_k and vector b_k from hyperplanes list.
     */
    void assembleMatrixRepresentation();

    /**
     * @brief Check whether point x satisfies A_k x <= b_k.
     */
    bool contains(const Eigen::Vector3f& point, float tolerance = 1e-4f) const;

    /**
     * @brief Compute maximum halfspace violation: max_i (a_i^T x - b_i).
     * Returns <= 0 if point is completely inside.
     */
    float computeMaxViolation(const Eigen::Vector3f& point) const;
};

/**
 * @brief 3D Quintic (5th-order) Polynomial Segment: p(t) = sum_{i=0}^5 c_i t^i for t in [0, T].
 */
class PolynomialSegment3D {
public:
    PolynomialSegment3D() = default;

    /**
     * @brief Construct polynomial segment with duration T and coefficient matrices.
     * @param duration Time interval T in seconds (must be > 0).
     * @param coeffs 3x6 coefficient matrix where row 0 is X, row 1 is Y, row 2 is Z:
     *               c_0 + c_1*t + c_2*t^2 + c_3*t^3 + c_4*t^4 + c_5*t^5
     */
    PolynomialSegment3D(float duration, const Eigen::Matrix<float, 3, 6>& coeffs);

    /**
     * @brief Solve closed-form minimum-jerk boundary value problem between two states.
     * @param p0 Initial position at t=0.
     * @param v0 Initial velocity at t=0.
     * @param a0 Initial acceleration at t=0.
     * @param p1 Final position at t=T.
     * @param v1 Final velocity at t=T.
     * @param a1 Final acceleration at t=T.
     * @param duration Total segment duration T > 0.
     * @return PolynomialSegment3D Solved quintic polynomial.
     */
    static PolynomialSegment3D solveMinimumJerkQuintic(
        const Eigen::Vector3f& p0, const Eigen::Vector3f& v0, const Eigen::Vector3f& a0,
        const Eigen::Vector3f& p1, const Eigen::Vector3f& v1, const Eigen::Vector3f& a1,
        float duration);

    float getDuration() const { return duration_; }
    const Eigen::Matrix<float, 3, 6>& getCoefficients() const { return coeffs_; }

    Eigen::Vector3f evaluatePosition(float t) const;
    Eigen::Vector3f evaluateVelocity(float t) const;
    Eigen::Vector3f evaluateAcceleration(float t) const;
    Eigen::Vector3f evaluateJerk(float t) const;
    Eigen::Vector3f evaluateSnap(float t) const;

    /**
     * @brief Evaluate transverse jerk orthogonal to the multirotor thrust vector z_B.
     * Prevents rate-gyro saturation and motor RPM desynchronization.
     * @param t Time in seconds [0, T].
     * @param gravity Gravity vector in ENU (default [0, 0, -9.81] m/s^2).
     * @return Eigen::Vector3f Transverse jerk component j_perp(t).
     */
    Eigen::Vector3f evaluateTransverseJerk(
        float t, const Eigen::Vector3f& gravity = Eigen::Vector3f(0.0f, 0.0f, -9.81f)) const;

private:
    float duration_ = 0.0f;
    // Columns: [c0, c1, c2, c3, c4, c5] for [X; Y; Z]
    Eigen::Matrix<float, 3, 6> coeffs_ = Eigen::Matrix<float, 3, 6>::Zero();
};

/**
 * @brief Diagnostic report of jerk and dynamic feasibility limits.
 */
struct JerkCheckResult {
    /// True if trajectory satisfies transverse jerk <= j_max and total jerk limits.
    bool satisfies_limit = false;

    /// Peak transverse jerk magnitude observed along segment (m/s^3).
    float max_transverse_jerk = 0.0f;

    /// Peak total jerk magnitude observed along segment (m/s^3).
    float max_total_jerk = 0.0f;

    /// Peak acceleration magnitude observed along segment (m/s^2).
    float max_acceleration = 0.0f;

    /// Time of peak transverse jerk (seconds).
    float time_of_peak_transverse_jerk = 0.0f;

    /// Enforced maximum transverse jerk threshold (m/s^3).
    float enforced_j_max = 10.0f;

    /// Diagnostic text.
    std::string diagnostics;
};

/**
 * @brief Configuration parameters for Safe Flight Corridor generation and verification.
 */
struct SafeFlightCorridorConfig {
    /// Default corridor radius / half-width when unobstructed (meters).
    float default_clearance = 2.0f;

    /// Minimum allowable corridor clearance (meters, vehicle safety bubble).
    float min_clearance = 0.5f;

    /// Longitudinal overlap between consecutive slices (meters) to guarantee feasible handoff.
    float overlap_margin = 0.3f;

    /// Clearance buffer between corridor boundary and obstacle points (meters).
    float obstacle_safety_margin = 0.4f;

    /// Number of radial angular sectors around trajectory segment for bounding planes.
    int num_radial_planes = 8;

    /// Maximum allowed transverse jerk (m/s^3, enforced <= 10.0 m/s^3).
    float max_transverse_jerk = 10.0f;

    /// Maximum allowed total jerk (m/s^3).
    float max_total_jerk = 15.0f;

    /// Maximum allowed total acceleration (m/s^2).
    float max_acceleration = 6.0f;
};

/**
 * @brief Safe Flight Corridor (SFC) generator and polynomial trajectory validator.
 *
 * Constructs a sequence of 3D convex polyhedral corridor slices A_k x <= b_k
 * along a piecewise-linear path given obstacle clearances.
 *
 * Verifies polynomial trajectories inside corridor slices and enforces:
 *   |p^(3)(t)|_perp <= j_max <= 10.0 m/s^3
 * to prevent rate-gyro saturation and motor desync during dynamic maneuvers.
 */
class SafeFlightCorridor {
public:
    explicit SafeFlightCorridor(const SafeFlightCorridorConfig& config = SafeFlightCorridorConfig());

    ~SafeFlightCorridor() = default;

    /**
     * @brief Generate 3D convex corridor slices along a piecewise-linear path.
     * @param waypoints Sequence of 3D waypoints [w_0, w_1, ..., w_N].
     * @param obstacle_points Point cloud of obstacles in the environment.
     * @return std::vector<CorridorSlice> Slices satisfying A_k x <= b_k.
     */
    std::vector<CorridorSlice> generateCorridor(
        const std::vector<Eigen::Vector3f>& waypoints,
        const std::vector<Eigen::Vector3f>& obstacle_points = {}) const;

    /**
     * @brief Verify that a polynomial trajectory segment satisfies transverse jerk limits.
     * Enforces ||j_perp(t)|| <= j_max <= 10.0 m/s^3.
     * @param segment Polynomial trajectory segment.
     * @param j_max Maximum allowed transverse jerk (default from config, <= 10.0 m/s^3).
     * @param num_samples Number of evaluation samples along the interval [0, T].
     * @return JerkCheckResult Diagnostic report.
     */
    JerkCheckResult checkJerkLimits(
        const PolynomialSegment3D& segment,
        float j_max = 10.0f,
        int num_samples = 100) const;

    /**
     * @brief Verify that a polynomial trajectory segment remains entirely within a corridor slice.
     * @param segment Polynomial segment.
     * @param slice Corridor slice A_k x <= b_k.
     * @param num_samples Number of sample points.
     * @param tolerance Margin for boundary containment.
     * @return true if all sample points satisfy A_k x <= b_k + tolerance.
     */
    bool verifyTrajectoryInCorridor(
        const PolynomialSegment3D& segment,
        const CorridorSlice& slice,
        int num_samples = 50,
        float tolerance = 1e-3f) const;

    const SafeFlightCorridorConfig& getConfig() const { return config_; }
    void setConfig(const SafeFlightCorridorConfig& config) { config_ = config; }

private:
    SafeFlightCorridorConfig config_;
};

} // namespace production
} // namespace px4_airsim_autonomy
