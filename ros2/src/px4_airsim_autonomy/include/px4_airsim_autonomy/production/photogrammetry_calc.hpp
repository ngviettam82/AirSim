#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include <cmath>

namespace px4_airsim_autonomy::production {

/**
 * @brief Optical and digital sensor parameters for aerial photogrammetry cameras.
 */
struct CameraSpec {
    double sensor_width_mm{35.9};    ///< Physical sensor width in millimeters
    double sensor_height_mm{24.0};   ///< Physical sensor height in millimeters
    double focal_length_mm{35.0};    ///< Lens focal length in millimeters
    uint32_t image_width_px{8192};   ///< Image horizontal resolution in pixels
    uint32_t image_height_px{5460};  ///< Image vertical resolution in pixels

    // Factory presets for industry-standard aerial survey cameras
    static CameraSpec DJI_Phantom4_Pro();
    static CameraSpec DJI_Zenmuse_P1_35mm();
    static CameraSpec Sony_A7R_IV_35mm();
    static CameraSpec AirSim_Default_1080p();

    /**
     * @brief Validate that optical and sensor parameters are positive and non-zero.
     */
    bool isValid() const;

    /**
     * @brief Horizontal pixel pitch (mm/px).
     */
    double pixelPitchHorizontalMm() const;

    /**
     * @brief Vertical pixel pitch (mm/px).
     */
    double pixelPitchVerticalMm() const;

    /**
     * @brief Horizontal Field of View in radians: 2 * atan(S_w / (2 * f))
     */
    double hfovRad() const;

    /**
     * @brief Horizontal Field of View in degrees.
     */
    double hfovDeg() const;

    /**
     * @brief Vertical Field of View in radians: 2 * atan(S_h / (2 * f))
     */
    double vfovRad() const;

    /**
     * @brief Vertical Field of View in degrees.
     */
    double vfovDeg() const;

    /**
     * @brief Sensor aspect ratio (width / height).
     */
    double aspectRatio() const;
};

/**
 * @brief Ground Sample Distance (GSD) and camera ground footprint metrics at given AGL altitude.
 */
struct GsdResult {
    double altitude_agl_m{0.0};       ///< Above Ground Level altitude in meters
    double gsd_h_m_per_px{0.0};      ///< Horizontal GSD in meters/pixel: (H * S_w) / (f * I_w)
    double gsd_v_m_per_px{0.0};      ///< Vertical GSD in meters/pixel: (H * S_h) / (f * I_h)
    double gsd_mean_m_per_px{0.0};   ///< Mean GSD in meters/pixel: 0.5 * (gsd_h + gsd_v)
    double footprint_width_m{0.0};   ///< Ground footprint width in meters: I_w * GSD_h = (H * S_w) / f
    double footprint_height_m{0.0};  ///< Ground footprint height in meters: I_h * GSD_v = (H * S_h) / f

    // Utility accessors in centimeters
    double gsd_h_cm() const { return gsd_h_m_per_px * 100.0; }
    double gsd_v_cm() const { return gsd_v_m_per_px * 100.0; }
    double gsd_mean_cm() const { return gsd_mean_m_per_px * 100.0; }
};

/**
 * @brief Aerial survey planning parameters including overlap ratios and shutter speed.
 */
struct SurveyParameters {
    double forward_overlap{0.80};    ///< Front/along-track overlap ratio O_f in [0.0, 1.0)
    double side_overlap{0.70};       ///< Side/cross-track overlap ratio O_s in [0.0, 1.0)
    double shutter_speed_s{0.001};   ///< Exposure duration t_shutter in seconds (e.g. 1/1000 s = 0.001 s)
    double blur_budget_px{0.5};      ///< Allowable motion blur in pixels (typically 0.5 px)

    bool isValid() const;
};

/**
 * @brief Photogrammetric survey grid metrics (trigger distance, strip spacing, blur limit).
 */
struct SurveyGridMetrics {
    CameraSpec camera;
    double altitude_agl_m{0.0};
    GsdResult gsd;
    double trigger_distance_m{0.0};  ///< Along-track trigger spacing: D_trigger = I_h * GSD_v * (1 - O_f)
    double strip_spacing_m{0.0};     ///< Cross-track strip spacing: S_strip = I_w * GSD_h * (1 - O_s)
    double max_speed_blur_m_s{0.0};  ///< Dynamic blur speed limit: v_max = (k_blur * GSD) / t_shutter
    double trigger_time_interval_s{0.0}; ///< Shutter interval at nominal cruising speed
    double photos_per_km{0.0};       ///< Trigger count per kilometer along flight line
    double strips_per_km{0.0};       ///< Strip count per kilometer cross-track
};

/**
 * @brief Smart Oblique Capture (SOC) 5-way gimbal orientation.
 */
struct SocGimbalPose {
    enum class Direction {
        Nadir = 0,
        Forward = 1,
        Right = 2,
        Aft = 3,
        Left = 4
    };

    Direction direction{Direction::Nadir};
    std::string direction_name;      ///< Human-readable direction ("Nadir", "Forward", "Right", "Aft", "Left")
    double pitch_deg{0.0};           ///< Gimbal pitch angle in degrees (Nadir=0°, Fwd=+45°, Right=+45°, Aft=-45°, Left=-45°)
    double roll_deg{0.0};            ///< Gimbal roll angle in degrees
    double yaw_deg{0.0};             ///< Gimbal azimuth/yaw relative to aircraft flight direction in degrees
    int sequence_index{0};           ///< 0 to 4 in 5-way sequence

    /**
     * @brief Convert Euler angles (roll, pitch, yaw) in Body FLU frame to Eigen Quaternion.
     */
    Eigen::Quaterniond toQuaternion() const;
};

/**
 * @brief Core photogrammetry calculations for aerial survey and mapping.
 */
class PhotogrammetryCalc {
public:
    PhotogrammetryCalc() = delete;

    /**
     * @brief Calculate Ground Sample Distance (GSD) and ground footprint dimensions at AGL altitude.
     * Formula:
     *   GSD_h = (H * S_w) / (f * I_w)
     *   GSD_v = (H * S_h) / (f * I_h)
     * @param spec Camera optical and sensor specifications.
     * @param altitude_agl_m Altitude Above Ground Level in meters.
     * @return GsdResult Struct containing horizontal, vertical, and mean GSD, plus ground footprint.
     */
    static GsdResult computeGsd(const CameraSpec& spec, double altitude_agl_m);

    /**
     * @brief Compute the required flight altitude AGL to achieve a target GSD.
     * Formula:
     *   H = (GSD_target * f * I_w) / S_w
     * @param spec Camera optical and sensor specifications.
     * @param target_gsd_m_per_px Target GSD in meters per pixel.
     * @return Altitude AGL in meters. Returns 0.0 if parameters are invalid.
     */
    static double computeRequiredAltitude(const CameraSpec& spec, double target_gsd_m_per_px);

    /**
     * @brief Compute forward trigger distance along flight line.
     * Formula:
     *   D_trigger = I_h * GSD_v * (1 - O_f) = H_footprint * (1 - O_f)
     * @param footprint_height_m Ground footprint height in meters.
     * @param forward_overlap Forward overlap ratio O_f in [0.0, 1.0).
     * @return Trigger distance in meters.
     */
    static double computeTriggerDistance(double footprint_height_m, double forward_overlap);

    /**
     * @brief Compute side strip spacing between adjacent flight lines.
     * Formula:
     *   S_strip = I_w * GSD_h * (1 - O_s) = W_footprint * (1 - O_s)
     * @param footprint_width_m Ground footprint width in meters.
     * @param side_overlap Side overlap ratio O_s in [0.0, 1.0).
     * @return Strip spacing in meters.
     */
    static double computeStripSpacing(double footprint_width_m, double side_overlap);

    /**
     * @brief Compute maximum permissible flight speed to avoid motion blur.
     * Motion blur occurs when ground displacement during shutter exposure exceeds
     * allowable blur fraction (typically 0.5 pixel).
     * Formula:
     *   v_max = (blur_budget_px * GSD) / t_shutter
     * @param gsd_m_per_px Ground sample distance in meters per pixel.
     * @param shutter_speed_s Camera shutter exposure duration in seconds.
     * @param blur_budget_px Maximum allowable pixel smear (default 0.5 px).
     * @return Maximum safe flight speed in m/s.
     */
    static double computeMaxSpeedForMotionBlur(double gsd_m_per_px,
                                              double shutter_speed_s,
                                              double blur_budget_px = 0.5);

    /**
     * @brief Compute comprehensive survey grid metrics at given altitude and speed.
     * @param spec Camera specification.
     * @param altitude_agl_m Flight altitude AGL in meters.
     * @param params Survey parameters (overlaps, shutter speed, blur budget).
     * @param nominal_speed_m_s Planned flight cruising speed in m/s.
     * @return SurveyGridMetrics Full photogrammetry parameters.
     */
    static SurveyGridMetrics computeSurveyGridMetrics(const CameraSpec& spec,
                                                      double altitude_agl_m,
                                                      const SurveyParameters& params,
                                                      double nominal_speed_m_s = 5.0);

    /**
     * @brief Generate Smart Oblique Capture (SOC) 5-way gimbal sequence poses.
     * Poses:
     *   0: Nadir (0°)
     *   1: Forward (+45°)
     *   2: Right (+45°)
     *   3: Aft (-45°)
     *   4: Left (-45°)
     * @param oblique_angle_deg Oblique pitch angle in degrees (default 45.0°).
     * @return std::vector<SocGimbalPose> 5 sequential gimbal poses.
     */
    static std::vector<SocGimbalPose> generateSocGimbalSequence(double oblique_angle_deg = 45.0);
};

} // namespace px4_airsim_autonomy::production
