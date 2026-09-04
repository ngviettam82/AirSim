#include "px4_airsim_autonomy/production/photogrammetry_calc.hpp"
#include <algorithm>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace px4_airsim_autonomy::production {

// -----------------------------------------------------------------------------
// CameraSpec Implementations
// -----------------------------------------------------------------------------

CameraSpec CameraSpec::DJI_Phantom4_Pro() {
    CameraSpec spec;
    spec.sensor_width_mm = 13.2;
    spec.sensor_height_mm = 8.8;
    spec.focal_length_mm = 8.8;
    spec.image_width_px = 5472;
    spec.image_height_px = 3648;
    return spec;
}

CameraSpec CameraSpec::DJI_Zenmuse_P1_35mm() {
    CameraSpec spec;
    spec.sensor_width_mm = 35.9;
    spec.sensor_height_mm = 24.0;
    spec.focal_length_mm = 35.0;
    spec.image_width_px = 8192;
    spec.image_height_px = 5460;
    return spec;
}

CameraSpec CameraSpec::Sony_A7R_IV_35mm() {
    CameraSpec spec;
    spec.sensor_width_mm = 35.7;
    spec.sensor_height_mm = 23.8;
    spec.focal_length_mm = 35.0;
    spec.image_width_px = 9504;
    spec.image_height_px = 6336;
    return spec;
}

CameraSpec CameraSpec::AirSim_Default_1080p() {
    CameraSpec spec;
    spec.sensor_width_mm = 19.2;
    spec.sensor_height_mm = 10.8;
    spec.focal_length_mm = 24.0;
    spec.image_width_px = 1920;
    spec.image_height_px = 1080;
    return spec;
}

bool CameraSpec::isValid() const {
    return sensor_width_mm > 0.0 &&
           sensor_height_mm > 0.0 &&
           focal_length_mm > 0.0 &&
           image_width_px > 0 &&
           image_height_px > 0;
}

double CameraSpec::pixelPitchHorizontalMm() const {
    return (image_width_px > 0) ? (sensor_width_mm / static_cast<double>(image_width_px)) : 0.0;
}

double CameraSpec::pixelPitchVerticalMm() const {
    return (image_height_px > 0) ? (sensor_height_mm / static_cast<double>(image_height_px)) : 0.0;
}

double CameraSpec::hfovRad() const {
    if (focal_length_mm <= 0.0) return 0.0;
    return 2.0 * std::atan(sensor_width_mm / (2.0 * focal_length_mm));
}

double CameraSpec::hfovDeg() const {
    return hfovRad() * (180.0 / M_PI);
}

double CameraSpec::vfovRad() const {
    if (focal_length_mm <= 0.0) return 0.0;
    return 2.0 * std::atan(sensor_height_mm / (2.0 * focal_length_mm));
}

double CameraSpec::vfovDeg() const {
    return vfovRad() * (180.0 / M_PI);
}

double CameraSpec::aspectRatio() const {
    return (image_height_px > 0) ? (static_cast<double>(image_width_px) / static_cast<double>(image_height_px)) : 0.0;
}

// -----------------------------------------------------------------------------
// SurveyParameters Implementations
// -----------------------------------------------------------------------------

bool SurveyParameters::isValid() const {
    return forward_overlap >= 0.0 && forward_overlap < 1.0 &&
           side_overlap >= 0.0 && side_overlap < 1.0 &&
           shutter_speed_s > 0.0 &&
           blur_budget_px > 0.0;
}

// -----------------------------------------------------------------------------
// SocGimbalPose Implementations
// -----------------------------------------------------------------------------

Eigen::Quaterniond SocGimbalPose::toQuaternion() const {
    constexpr double deg2rad = M_PI / 180.0;
    const double r = roll_deg * deg2rad;
    const double p = pitch_deg * deg2rad;
    const double y = yaw_deg * deg2rad;

    // Body FLU: Yaw around Z, Pitch around Y, Roll around X
    Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());

    return yawAngle * pitchAngle * rollAngle;
}

// -----------------------------------------------------------------------------
// PhotogrammetryCalc Core Implementations
// -----------------------------------------------------------------------------

GsdResult PhotogrammetryCalc::computeGsd(const CameraSpec& spec, double altitude_agl_m) {
    GsdResult result;
    if (!spec.isValid() || altitude_agl_m <= 0.0) {
        return result;
    }

    result.altitude_agl_m = altitude_agl_m;

    // GSD_h = (H * S_w) / (f * I_w)
    result.gsd_h_m_per_px = (altitude_agl_m * spec.sensor_width_mm) /
                            (spec.focal_length_mm * static_cast<double>(spec.image_width_px));

    // GSD_v = (H * S_h) / (f * I_h)
    result.gsd_v_m_per_px = (altitude_agl_m * spec.sensor_height_mm) /
                            (spec.focal_length_mm * static_cast<double>(spec.image_height_px));

    result.gsd_mean_m_per_px = 0.5 * (result.gsd_h_m_per_px + result.gsd_v_m_per_px);

    // Ground footprint in meters
    result.footprint_width_m = static_cast<double>(spec.image_width_px) * result.gsd_h_m_per_px;
    result.footprint_height_m = static_cast<double>(spec.image_height_px) * result.gsd_v_m_per_px;

    return result;
}

double PhotogrammetryCalc::computeRequiredAltitude(const CameraSpec& spec, double target_gsd_m_per_px) {
    if (!spec.isValid() || target_gsd_m_per_px <= 0.0) {
        return 0.0;
    }
    // H = (GSD_target * f * I_w) / S_w
    return (target_gsd_m_per_px * spec.focal_length_mm * static_cast<double>(spec.image_width_px)) /
           spec.sensor_width_mm;
}

double PhotogrammetryCalc::computeTriggerDistance(double footprint_height_m, double forward_overlap) {
    if (footprint_height_m <= 0.0 || forward_overlap < 0.0 || forward_overlap >= 1.0) {
        return 0.0;
    }
    // D_trigger = I_h * GSD_v * (1 - O_f) = H_footprint * (1 - O_f)
    return footprint_height_m * (1.0 - forward_overlap);
}

double PhotogrammetryCalc::computeStripSpacing(double footprint_width_m, double side_overlap) {
    if (footprint_width_m <= 0.0 || side_overlap < 0.0 || side_overlap >= 1.0) {
        return 0.0;
    }
    // S_strip = I_w * GSD_h * (1 - O_s) = W_footprint * (1 - O_s)
    return footprint_width_m * (1.0 - side_overlap);
}

double PhotogrammetryCalc::computeMaxSpeedForMotionBlur(double gsd_m_per_px,
                                                       double shutter_speed_s,
                                                       double blur_budget_px) {
    if (gsd_m_per_px <= 0.0 || shutter_speed_s <= 0.0 || blur_budget_px <= 0.0) {
        return 0.0;
    }
    // v_max = (k_blur * GSD) / t_shutter
    return (blur_budget_px * gsd_m_per_px) / shutter_speed_s;
}

SurveyGridMetrics PhotogrammetryCalc::computeSurveyGridMetrics(const CameraSpec& spec,
                                                               double altitude_agl_m,
                                                               const SurveyParameters& params,
                                                               double nominal_speed_m_s) {
    SurveyGridMetrics metrics;
    metrics.camera = spec;
    metrics.altitude_agl_m = altitude_agl_m;
    metrics.gsd = computeGsd(spec, altitude_agl_m);

    if (!spec.isValid() || !params.isValid() || altitude_agl_m <= 0.0) {
        return metrics;
    }

    metrics.trigger_distance_m = computeTriggerDistance(metrics.gsd.footprint_height_m, params.forward_overlap);
    metrics.strip_spacing_m = computeStripSpacing(metrics.gsd.footprint_width_m, params.side_overlap);

    const double limiting_gsd = std::min(metrics.gsd.gsd_h_m_per_px, metrics.gsd.gsd_v_m_per_px);
    metrics.max_speed_blur_m_s = computeMaxSpeedForMotionBlur(limiting_gsd, params.shutter_speed_s, params.blur_budget_px);

    const double effective_speed = (nominal_speed_m_s > 0.0) ? nominal_speed_m_s : metrics.max_speed_blur_m_s;
    if (effective_speed > 0.0 && metrics.trigger_distance_m > 0.0) {
        metrics.trigger_time_interval_s = metrics.trigger_distance_m / effective_speed;
    }

    if (metrics.trigger_distance_m > 0.0) {
        metrics.photos_per_km = 1000.0 / metrics.trigger_distance_m;
    }
    if (metrics.strip_spacing_m > 0.0) {
        metrics.strips_per_km = 1000.0 / metrics.strip_spacing_m;
    }

    return metrics;
}

std::vector<SocGimbalPose> PhotogrammetryCalc::generateSocGimbalSequence(double oblique_angle_deg) {
    std::vector<SocGimbalPose> sequence;
    sequence.reserve(5);

    const double obl = std::abs(oblique_angle_deg);

    // 0: Nadir (0°)
    {
        SocGimbalPose pose;
        pose.direction = SocGimbalPose::Direction::Nadir;
        pose.direction_name = "Nadir";
        pose.pitch_deg = 0.0;
        pose.roll_deg = 0.0;
        pose.yaw_deg = 0.0;
        pose.sequence_index = 0;
        sequence.push_back(pose);
    }

    // 1: Forward (+45°)
    {
        SocGimbalPose pose;
        pose.direction = SocGimbalPose::Direction::Forward;
        pose.direction_name = "Forward";
        pose.pitch_deg = obl;
        pose.roll_deg = 0.0;
        pose.yaw_deg = 0.0;
        pose.sequence_index = 1;
        sequence.push_back(pose);
    }

    // 2: Right (+45°)
    {
        SocGimbalPose pose;
        pose.direction = SocGimbalPose::Direction::Right;
        pose.direction_name = "Right";
        pose.pitch_deg = obl;
        pose.roll_deg = 0.0;
        pose.yaw_deg = 90.0;
        pose.sequence_index = 2;
        sequence.push_back(pose);
    }

    // 3: Aft (-45°)
    {
        SocGimbalPose pose;
        pose.direction = SocGimbalPose::Direction::Aft;
        pose.direction_name = "Aft";
        pose.pitch_deg = -obl;
        pose.roll_deg = 0.0;
        pose.yaw_deg = 0.0;
        pose.sequence_index = 3;
        sequence.push_back(pose);
    }

    // 4: Left (-45°)
    {
        SocGimbalPose pose;
        pose.direction = SocGimbalPose::Direction::Left;
        pose.direction_name = "Left";
        pose.pitch_deg = -obl;
        pose.roll_deg = 0.0;
        pose.yaw_deg = -90.0;
        pose.sequence_index = 4;
        sequence.push_back(pose);
    }

    return sequence;
}

} // namespace px4_airsim_autonomy::production
