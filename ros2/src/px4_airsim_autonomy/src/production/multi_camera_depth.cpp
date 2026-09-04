#include "px4_airsim_autonomy/production/multi_camera_depth.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace px4_airsim_autonomy {
namespace production {

MultiCameraDepthProcessor::MultiCameraDepthProcessor() {
    reset();
}

Eigen::Matrix3f MultiCameraDepthProcessor::computeMountRotation(CameraMountOrientation mount) {
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();

    switch (mount) {
        case CameraMountOrientation::Forward:
            // Camera Z (forward) -> Body +X
            // Camera X (right)   -> Body -Y
            // Camera Y (down)    -> Body -Z
            R <<  0.0f,  0.0f,  1.0f,
                 -1.0f,  0.0f,  0.0f,
                  0.0f, -1.0f,  0.0f;
            break;

        case CameraMountOrientation::Left:
            // Camera Z (forward) -> Body +Y
            // Camera X (right)   -> Body +X
            // Camera Y (down)    -> Body -Z
            R <<  1.0f,  0.0f,  0.0f,
                  0.0f,  0.0f,  1.0f,
                  0.0f, -1.0f,  0.0f;
            break;

        case CameraMountOrientation::Right:
            // Camera Z (forward) -> Body -Y
            // Camera X (right)   -> Body -X
            // Camera Y (down)    -> Body -Z
            R << -1.0f,  0.0f,  0.0f,
                  0.0f,  0.0f, -1.0f,
                  0.0f, -1.0f,  0.0f;
            break;

        case CameraMountOrientation::Downward:
            // Camera Z (forward) -> Body -Z
            // Camera X (right)   -> Body -Y
            // Camera Y (down)    -> Body -X
            R <<  0.0f, -1.0f,  0.0f,
                 -1.0f,  0.0f,  0.0f,
                  0.0f,  0.0f, -1.0f;
            break;

        case CameraMountOrientation::Backward:
            // Camera Z (forward) -> Body -X
            // Camera X (right)   -> Body +Y
            // Camera Y (down)    -> Body -Z
            R <<  0.0f,  0.0f, -1.0f,
                  1.0f,  0.0f,  0.0f,
                  0.0f, -1.0f,  0.0f;
            break;

        case CameraMountOrientation::Upward:
            // Camera Z (forward) -> Body +Z
            // Camera X (right)   -> Body -Y
            // Camera Y (down)    -> Body +X
            R <<  0.0f,  1.0f,  0.0f,
                 -1.0f,  0.0f,  0.0f,
                  0.0f,  0.0f,  1.0f;
            break;

        case CameraMountOrientation::Custom:
        default:
            R = Eigen::Matrix3f::Identity();
            break;
    }

    return R;
}

void MultiCameraDepthProcessor::registerCamera(const CameraStreamConfig& config) {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    CameraStreamConfig resolved_config = config;
    // Auto-populate mount rotation if custom rotation was not explicitly provided
    if (resolved_config.mount != CameraMountOrientation::Custom &&
        resolved_config.R_cam_to_body.isIdentity(1e-4f)) {
        resolved_config.R_cam_to_body = computeMountRotation(resolved_config.mount);
    }

    // Ensure valid subsample step
    if (resolved_config.subsample_step < 1) {
        resolved_config.subsample_step = 1;
    }

    CameraInternalState state;
    state.config = resolved_config;
    state.last_frame_timestamp_us = 0;
    state.has_data = false;

    // Preallocate reserve capacity to eliminate runtime heap allocations during ingest
    const int est_w = (resolved_config.intrinsics.width > 0) ? resolved_config.intrinsics.width : 640;
    const int est_h = (resolved_config.intrinsics.height > 0) ? resolved_config.intrinsics.height : 480;
    const size_t est_points = static_cast<size_t>((est_w / resolved_config.subsample_step) *
                                                  (est_h / resolved_config.subsample_step));
    state.points_flu.reserve(est_points);

    cameras_[config.camera_id] = std::move(state);
}

void MultiCameraDepthProcessor::unregisterCamera(const std::string& camera_id) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    cameras_.erase(camera_id);
    recomputeClearanceInternal();
}

bool MultiCameraDepthProcessor::hasCamera(const std::string& camera_id) const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return cameras_.find(camera_id) != cameras_.end();
}

size_t MultiCameraDepthProcessor::getRegisteredCameraCount() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return cameras_.size();
}

bool MultiCameraDepthProcessor::ingestDepthFrame(
    const std::string& camera_id,
    const float* depth_data,
    int width,
    int height,
    int row_stride_elements,
    uint64_t timestamp_us
) {
    if (depth_data == nullptr || width <= 0 || height <= 0 || row_stride_elements < width) {
        return false;
    }

    std::unique_lock<std::shared_mutex> lock(mutex_);

    auto it = cameras_.find(camera_id);
    if (it == cameras_.end()) {
        return false; // Camera not registered
    }

    CameraInternalState& cam = it->second;
    if (!cam.config.enabled) {
        return false;
    }

    // Update intrinsics resolution if changed dynamically
    if (cam.config.intrinsics.width != width || cam.config.intrinsics.height != height) {
        cam.config.intrinsics = CameraIntrinsics::fromFov(width, height, cam.config.intrinsics.fov_h_deg);
    }

    const CameraIntrinsics& intr = cam.config.intrinsics;
    const float fx = intr.fx;
    const float fy = intr.fy;
    const float cx = intr.cx;
    const float cy = intr.cy;
    const float min_d = cam.config.min_valid_depth_m;
    const float max_d = cam.config.max_valid_depth_m;
    const int step = std::max(1, cam.config.subsample_step);

    const Eigen::Matrix3f& R = cam.config.R_cam_to_body;
    const Eigen::Vector3f& t = cam.config.t_cam_to_body;

    cam.points_flu.clear();

    // Fast zero-copy unprojection into camera optical frame and transform to Body FLU
    for (int v = 0; v < height; v += step) {
        const float* row_ptr = depth_data + (v * row_stride_elements);
        const float y_norm = (static_cast<float>(v) - cy) / fy;

        for (int u = 0; u < width; u += step) {
            const float d = row_ptr[u];

            // Filter invalid / NaN / Inf / out-of-range depths
            if (d >= min_d && d <= max_d && !std::isnan(d) && !std::isinf(d)) {
                const float x_norm = (static_cast<float>(u) - cx) / fx;

                // 3D Point in camera optical frame
                const Eigen::Vector3f p_c(x_norm * d, y_norm * d, d);

                // Transform to Body FLU: p_b = R_c^b * p_c + t_c^b
                const Eigen::Vector3f p_b = R * p_c + t;

                cam.points_flu.push_back(p_b);
            }
        }
    }

    cam.last_frame_timestamp_us = timestamp_us;
    cam.has_data = true;

    // Recompute multi-camera aggregated directional clearance
    recomputeClearanceInternal();
    return true;
}

bool MultiCameraDepthProcessor::ingestRosImage(
    const std::string& camera_id,
    const sensor_msgs::msg::Image& image_msg
) {
    // AirSim DepthPlanar is published as 32FC1 (single precision floating point meters)
    if (image_msg.encoding != "32FC1") {
        return false;
    }

    if (image_msg.width == 0 || image_msg.height == 0) {
        return false;
    }

    const size_t min_bytes = static_cast<size_t>(image_msg.height * image_msg.step);
    if (image_msg.data.size() < min_bytes) {
        return false;
    }

    const int width = static_cast<int>(image_msg.width);
    const int height = static_cast<int>(image_msg.height);
    const int row_stride_elements = static_cast<int>(image_msg.step / sizeof(float));

    const float* depth_ptr = reinterpret_cast<const float*>(image_msg.data.data());

    const uint64_t timestamp_us = static_cast<uint64_t>(image_msg.header.stamp.sec) * 1000000ULL +
                                  static_cast<uint64_t>(image_msg.header.stamp.nanosec / 1000ULL);

    return ingestDepthFrame(camera_id, depth_ptr, width, height, row_stride_elements, timestamp_us);
}

void MultiCameraDepthProcessor::recomputeClearanceInternal() {
    DirectionalObstacleClearance c;
    c.min_distance = 100.0f;
    c.front = 100.0f;
    c.left = 100.0f;
    c.right = 100.0f;
    c.back = 100.0f;
    c.upper = 100.0f;
    c.lower = 100.0f;

    for (int r = 0; r < 3; ++r) {
        for (int col = 0; col < 3; ++col) {
            c.sectors_3d[r][col] = 100.0f;
        }
    }

    size_t total_points = 0;
    uint64_t max_stamp = 0;

    // Threshold constants for sector classification in Body FLU
    constexpr float kTan30 = 0.57735f;  ///< tan(30 deg)
    constexpr float kZClearanceMid = 0.35f; ///< Vertical envelope for horizontal flight (meters)

    for (const auto& kv : cameras_) {
        const CameraInternalState& state = kv.second;
        if (!state.config.enabled || !state.has_data) {
            continue;
        }

        if (state.last_frame_timestamp_us > max_stamp) {
            max_stamp = state.last_frame_timestamp_us;
        }

        total_points += state.points_flu.size();

        for (const auto& p : state.points_flu) {
            const float x = p.x(); // Forward (+X)
            const float y = p.y(); // Left (+Y)
            const float z = p.z(); // Up (+Z)
            const float dist = p.norm();

            if (dist < c.min_distance) {
                c.min_distance = dist;
            }

            // Directional clearance evaluation
            // 1. Upper / Lower clearance
            if (z > kZClearanceMid) {
                if (dist < c.upper) c.upper = dist;
            } else if (z < -kZClearanceMid) {
                if (dist < c.lower) c.lower = dist;
            }

            // 2. Front clearance: points in front cone (x > 0, azimuth within +/- 30 deg)
            if (x > 0.1f && std::abs(y) <= (x * kTan30) && std::abs(z) <= 1.5f) {
                if (dist < c.front) c.front = dist;
            }

            // 3. Left clearance: points to the left (+Y dominant)
            if (y > 0.1f && y > (std::abs(x) * kTan30) && std::abs(z) <= 1.5f) {
                if (dist < c.left) c.left = dist;
            }

            // 4. Right clearance: points to the right (-Y dominant)
            if (y < -0.1f && (-y) > (std::abs(x) * kTan30) && std::abs(z) <= 1.5f) {
                if (dist < c.right) c.right = dist;
            }

            // 5. Back clearance: points behind (-X dominant)
            if (x < -0.1f && std::abs(y) <= (-x * kTan30) && std::abs(z) <= 1.5f) {
                if (dist < c.back) c.back = dist;
            }

            // 6. Populate 3x3 directional sector depth grid
            // Elevation band: 0 = Upper, 1 = Mid, 2 = Lower
            int r_idx = 1;
            if (z > kZClearanceMid) {
                r_idx = 0;
            } else if (z < -kZClearanceMid) {
                r_idx = 2;
            }

            // Azimuth sector: 0 = Left, 1 = Center/Front, 2 = Right
            int c_idx = 1;
            if (x > 0.05f) {
                const float y_ratio = y / x;
                if (y_ratio > 0.35f) {
                    c_idx = 0; // Left
                } else if (y_ratio < -0.35f) {
                    c_idx = 2; // Right
                } else {
                    c_idx = 1; // Center
                }
            } else {
                // If lateral or behind, map directly to Left or Right
                c_idx = (y >= 0.0f) ? 0 : 2;
            }

            if (dist < c.sectors_3d[r_idx][c_idx]) {
                c.sectors_3d[r_idx][c_idx] = dist;
            }
        }
    }

    c.last_update_us = max_stamp;
    c.total_points_evaluated = total_points;
    aggregated_clearance_ = c;
}

DirectionalObstacleClearance MultiCameraDepthProcessor::getClearance() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return aggregated_clearance_;
}

void MultiCameraDepthProcessor::populateSensorSnapshot(SensorSnapshot& snapshot) const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    snapshot.min_depth_distance = aggregated_clearance_.min_distance;
    snapshot.center_sector_distance = aggregated_clearance_.front;
    snapshot.left_sector_distance = aggregated_clearance_.left;
    snapshot.right_sector_distance = aggregated_clearance_.right;
    snapshot.upper_sector_distance = aggregated_clearance_.upper;
    snapshot.lower_sector_distance = aggregated_clearance_.lower;

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            snapshot.sectors_3d[r][c] = aggregated_clearance_.sectors_3d[r][c];
        }
    }
}

std::vector<Eigen::Vector3f> MultiCameraDepthProcessor::getAggregatedPointCloud(size_t max_points) const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    std::vector<Eigen::Vector3f> combined;

    size_t total_available = 0;
    for (const auto& kv : cameras_) {
        if (kv.second.config.enabled && kv.second.has_data) {
            total_available += kv.second.points_flu.size();
        }
    }

    const size_t to_reserve = (max_points > 0 && max_points < total_available) ? max_points : total_available;
    combined.reserve(to_reserve);

    for (const auto& kv : cameras_) {
        const auto& pts = kv.second.points_flu;
        if (!kv.second.config.enabled || !kv.second.has_data) {
            continue;
        }

        for (const auto& pt : pts) {
            combined.push_back(pt);
            if (max_points > 0 && combined.size() >= max_points) {
                return combined;
            }
        }
    }

    return combined;
}

void MultiCameraDepthProcessor::reset() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    for (auto& kv : cameras_) {
        kv.second.points_flu.clear();
        kv.second.has_data = false;
        kv.second.last_frame_timestamp_us = 0;
    }

    aggregated_clearance_ = DirectionalObstacleClearance();
}

} // namespace production
} // namespace px4_airsim_autonomy
