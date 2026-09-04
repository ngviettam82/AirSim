#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

// Forward declaration or include for ROS 2 Image msg
#include <sensor_msgs/msg/image.hpp>
#include "px4_airsim_autonomy/types.hpp"

namespace px4_airsim_autonomy {
namespace production {

/**
 * @brief Predefined standard camera mounting orientations relative to Body FLU frame.
 * 
 * Standard conventions:
 * - Body FLU: +X Forward, +Y Left, +Z Up.
 * - Camera Optical Frame: +X Right, +Y Down, +Z Forward (optical axis).
 */
enum class CameraMountOrientation {
    Forward,   ///< Optical axis along +X_body, right along -Y_body, down along -Z_body
    Left,      ///< Optical axis along +Y_body, right along +X_body, down along -Z_body
    Right,     ///< Optical axis along -Y_body, right along -X_body, down along -Z_body
    Downward,  ///< Optical axis along -Z_body, right along -Y_body, down along -X_body
    Backward,  ///< Optical axis along -X_body, right along +Y_body, down along -Z_body
    Upward,    ///< Optical axis along +Z_body, right along -Y_body, down along +X_body
    Custom     ///< User-specified rotation matrix / quaternion
};

/**
 * @brief Pinhole camera intrinsic calibration model.
 */
struct CameraIntrinsics {
    int width = 640;
    int height = 480;
    float fx = 320.0f;       ///< Focal length along X in pixels
    float fy = 320.0f;       ///< Focal length along Y in pixels
    float cx = 320.0f;       ///< Principal point X in pixels
    float cy = 240.0f;       ///< Principal point Y in pixels
    float fov_h_deg = 90.0f;  ///< Horizontal Field of View in degrees

    /**
     * @brief Construct intrinsics from horizontal FOV and image dimensions.
     * @param w Image width in pixels
     * @param h Image height in pixels
     * @param fov_deg Horizontal field of view in degrees
     */
    static CameraIntrinsics fromFov(int w, int h, float fov_deg) {
        CameraIntrinsics intr;
        intr.width = w;
        intr.height = h;
        intr.fov_h_deg = fov_deg;
        const float rad = fov_deg * (static_cast<float>(M_PI) / 180.0f);
        intr.fx = (static_cast<float>(w) * 0.5f) / std::tan(rad * 0.5f);
        intr.fy = intr.fx; // Square pixel assumption
        intr.cx = static_cast<float>(w) * 0.5f;
        intr.cy = static_cast<float>(h) * 0.5f;
        return intr;
    }
};

/**
 * @brief Configuration and calibration parameters for an individual camera stream.
 */
struct CameraStreamConfig {
    std::string camera_id;
    CameraMountOrientation mount = CameraMountOrientation::Forward;
    CameraIntrinsics intrinsics;

    /// Extrinsic rotation: R_c^b transforms vector from camera optical frame to body FLU frame
    Eigen::Matrix3f R_cam_to_body = Eigen::Matrix3f::Identity();

    /// Extrinsic translation: t_c^b position of camera optical center in body FLU frame (meters)
    Eigen::Vector3f t_cam_to_body = Eigen::Vector3f::Zero();

    float min_valid_depth_m = 0.2f;    ///< Discard points closer than this threshold
    float max_valid_depth_m = 50.0f;   ///< Discard points farther than this threshold
    int subsample_step = 2;            ///< Pixel subsampling stride (2 = sample every 2nd pixel = 4x speedup)
    bool enabled = true;               ///< Enable/disable this stream in aggregation
};

/**
 * @brief Directional obstacle clearance aggregated across all active cameras in Body FLU.
 */
struct DirectionalObstacleClearance {
    float min_distance = 100.0f;  ///< Global minimum obstacle distance across all directions (m)
    float front = 100.0f;         ///< Clearance in forward sector (+X_body, m)
    float left = 100.0f;          ///< Clearance in left sector (+Y_body, m)
    float right = 100.0f;         ///< Clearance in right sector (-Y_body, m)
    float back = 100.0f;          ///< Clearance in backward sector (-X_body, m)
    float upper = 100.0f;         ///< Clearance in upper / ceiling sector (+Z_body, m)
    float lower = 100.0f;         ///< Clearance in lower / ground sector (-Z_body, m)

    /**
     * @brief 3x3 directional sector depth grid matching SensorSnapshot::sectors_3d:
     * Row 0: Upper [Left, Center/Front, Right]
     * Row 1: Mid   [Left, Center/Front, Right]
     * Row 2: Lower [Left, Center/Front, Right]
     */
    float sectors_3d[3][3] = {
        {100.0f, 100.0f, 100.0f},
        {100.0f, 100.0f, 100.0f},
        {100.0f, 100.0f, 100.0f}
    };

    uint64_t last_update_us = 0;     ///< Microsecond timestamp of latest ingested frame
    size_t total_points_evaluated = 0; ///< Total 3D points ingested in latest aggregation
};

/**
 * @brief Production Multi-Camera Depth Processor.
 * 
 * Ingests depth streams from multiple cameras (forward, left, right, downward, etc.),
 * unprojects 2D planar depth to 3D point clouds in camera optical frame, transforms
 * them via rigid body extrinsics into Body FLU frame, and aggregates directional
 * clearance into a spatial 3D sector grid.
 * 
 * Concurrency: Completely thread-safe using shared-mutex read/write locking.
 */
class MultiCameraDepthProcessor {
public:
    MultiCameraDepthProcessor();
    ~MultiCameraDepthProcessor() = default;

    /**
     * @brief Compute standard rotation matrix R_c^b for given mount orientation.
     * @param mount Mount orientation preset
     * @return 3x3 rotation matrix transforming camera optical frame to Body FLU
     */
    static Eigen::Matrix3f computeMountRotation(CameraMountOrientation mount);

    /**
     * @brief Register or update a camera stream configuration.
     * @param config Camera configuration parameters
     */
    void registerCamera(const CameraStreamConfig& config);

    /**
     * @brief Unregister a camera stream.
     * @param camera_id Unique identifier of the camera
     */
    void unregisterCamera(const std::string& camera_id);

    /**
     * @brief Ingest a raw planar depth buffer (zero-copy from pointer).
     * 
     * @param camera_id Registered camera identifier
     * @param depth_data Pointer to contiguous float depth values (meters, 32FC1)
     * @param width Image width in pixels
     * @param height Image height in pixels
     * @param row_stride_elements Number of float elements per row (step / sizeof(float))
     * @param timestamp_us Microsecond timestamp of depth capture
     * @return true if successfully processed, false if camera unknown or invalid data
     */
    bool ingestDepthFrame(
        const std::string& camera_id,
        const float* depth_data,
        int width,
        int height,
        int row_stride_elements,
        uint64_t timestamp_us
    );

    /**
     * @brief Ingest a ROS 2 Image message (zero-copy const reference).
     * 
     * @param camera_id Registered camera identifier
     * @param image_msg ROS 2 sensor_msgs::msg::Image message (encoding must be 32FC1)
     * @return true if successfully processed
     */
    bool ingestRosImage(
        const std::string& camera_id,
        const sensor_msgs::msg::Image& image_msg
    );

    /**
     * @brief Retrieve the aggregated directional obstacle clearance.
     * @return DirectionalObstacleClearance snapshot
     */
    DirectionalObstacleClearance getClearance() const;

    /**
     * @brief Populate a px4_airsim_autonomy::SensorSnapshot with aggregated clearances.
     * @param snapshot Target snapshot reference to populate
     */
    void populateSensorSnapshot(SensorSnapshot& snapshot) const;

    /**
     * @brief Retrieve all latest 3D points in Body FLU frame across all active cameras.
     * @param max_points Optional cap on points returned (0 = all)
     * @return Vector of 3D points in Body FLU (meters)
     */
    std::vector<Eigen::Vector3f> getAggregatedPointCloud(size_t max_points = 0) const;

    /**
     * @brief Reset clearance metrics and clear internal point caches.
     */
    void reset();

    /**
     * @brief Get count of registered cameras.
     */
    size_t getRegisteredCameraCount() const;

    /**
     * @brief Check if a specific camera has been registered.
     */
    bool hasCamera(const std::string& camera_id) const;

private:
    /// Internal per-camera state buffer
    struct CameraInternalState {
        CameraStreamConfig config;
        std::vector<Eigen::Vector3f> points_flu;
        uint64_t last_frame_timestamp_us = 0;
        bool has_data = false;
    };

    /**
     * @brief Recomputes global clearance across all camera point buffers.
     * Must be called while holding write lock on mutex_.
     */
    void recomputeClearanceInternal();

    mutable std::shared_mutex mutex_;
    std::unordered_map<std::string, CameraInternalState> cameras_;
    DirectionalObstacleClearance aggregated_clearance_;
};

} // namespace production
} // namespace px4_airsim_autonomy
