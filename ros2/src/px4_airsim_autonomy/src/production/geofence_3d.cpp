#include "px4_airsim_autonomy/production/geofence_3d.hpp"

#include <sstream>
#include <iomanip>

namespace px4_airsim_autonomy::production {

// ============================================================================
// Polygon2D Implementation
// ============================================================================

float Polygon2D::signedArea() const noexcept {
    if (vertices.size() < 3) {
        return 0.0f;
    }
    float area2 = 0.0f;
    const size_t n = vertices.size();
    for (size_t i = 0; i < n; ++i) {
        const auto& v1 = vertices[i];
        const auto& v2 = vertices[(i + 1) % n];
        area2 += (v1.x() * v2.y() - v2.x() * v1.y());
    }
    return 0.5f * area2;
}

bool Polygon2D::contains(const Eigen::Vector2f& pt) const noexcept {
    if (vertices.size() < 3) {
        return false;
    }

    int wn = 0; // Winding number counter
    const size_t n = vertices.size();

    for (size_t i = 0; i < n; ++i) {
        const auto& v1 = vertices[i];
        const auto& v2 = vertices[(i + 1) % n];

        // Check if point lies directly on edge segment within 1mm tolerance
        const Eigen::Vector2f seg = v2 - v1;
        const float seg_len_sq = seg.squaredNorm();
        if (seg_len_sq > 1e-8f) {
            const float t = (pt - v1).dot(seg) / seg_len_sq;
            if (t >= 0.0f && t <= 1.0f) {
                const Eigen::Vector2f proj = v1 + t * seg;
                if ((pt - proj).squaredNorm() < 1e-6f) {
                    return true; // Point on perimeter
                }
            }
        }

        // Winding number ray crossing test
        if (v1.y() <= pt.y()) {
            if (v2.y() > pt.y()) { // Upward crossing
                const float is_left = (v2.x() - v1.x()) * (pt.y() - v1.y()) - (pt.x() - v1.x()) * (v2.y() - v1.y());
                if (is_left > 0.0f) {
                    ++wn;
                }
            }
        } else {
            if (v2.y() <= pt.y()) { // Downward crossing
                const float is_left = (v2.x() - v1.x()) * (pt.y() - v1.y()) - (pt.x() - v1.x()) * (v2.y() - v1.y());
                if (is_left < 0.0f) {
                    --wn;
                }
            }
        }
    }

    return wn != 0;
}

float Polygon2D::distanceToBoundary(
    const Eigen::Vector2f& pt,
    Eigen::Vector2f& out_closest_pt,
    Eigen::Vector2f& out_normal) const noexcept {
    if (vertices.size() < 2) {
        out_closest_pt = pt;
        out_normal = Eigen::Vector2f::UnitX();
        return 0.0f;
    }

    float min_dist_sq = std::numeric_limits<float>::infinity();
    Eigen::Vector2f best_closest = pt;
    Eigen::Vector2f best_normal = Eigen::Vector2f::UnitX();

    const bool ccw = isCounterClockwise();
    const size_t n = vertices.size();

    for (size_t i = 0; i < n; ++i) {
        const auto& v1 = vertices[i];
        const auto& v2 = vertices[(i + 1) % n];

        const Eigen::Vector2f seg = v2 - v1;
        const float len_sq = seg.squaredNorm();
        if (len_sq < 1e-8f) {
            continue;
        }

        const float t = std::clamp((pt - v1).dot(seg) / len_sq, 0.0f, 1.0f);
        const Eigen::Vector2f candidate = v1 + t * seg;
        const float dist_sq = (pt - candidate).squaredNorm();

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_closest = candidate;

            const float len = std::sqrt(len_sq);
            Eigen::Vector2f edge_normal;
            if (ccw) {
                edge_normal = Eigen::Vector2f(seg.y(), -seg.x()) / len;
            } else {
                edge_normal = Eigen::Vector2f(-seg.y(), seg.x()) / len;
            }
            best_normal = edge_normal;
        }
    }

    out_closest_pt = best_closest;
    out_normal = best_normal;
    return std::sqrt(min_dist_sq);
}

bool Polygon2D::rayIntersect(
    const Eigen::Vector2f& origin,
    const Eigen::Vector2f& dir,
    float& out_dist,
    Eigen::Vector2f& out_normal) const noexcept {
    if (vertices.size() < 3 || dir.squaredNorm() < 1e-6f) {
        return false;
    }

    float min_s = std::numeric_limits<float>::infinity();
    Eigen::Vector2f best_normal = Eigen::Vector2f::Zero();
    bool hit = false;

    const bool ccw = isCounterClockwise();
    const size_t n = vertices.size();

    for (size_t i = 0; i < n; ++i) {
        const auto& v1 = vertices[i];
        const auto& v2 = vertices[(i + 1) % n];

        const Eigen::Vector2f seg = v2 - v1;
        const float cross = dir.x() * seg.y() - dir.y() * seg.x();

        // Check if ray and segment are nearly parallel
        if (std::abs(cross) < 1e-7f) {
            continue;
        }

        const Eigen::Vector2f delta = v1 - origin;
        const float s = (delta.x() * seg.y() - delta.y() * seg.x()) / cross;
        const float t = (delta.x() * dir.y() - delta.y() * dir.x()) / cross;

        if (s > 1e-3f && t >= 0.0f && t <= 1.0f) {
            if (s < min_s) {
                min_s = s;
                hit = true;

                const float len = seg.norm();
                if (len > 1e-6f) {
                    if (ccw) {
                        best_normal = Eigen::Vector2f(seg.y(), -seg.x()) / len;
                    } else {
                        best_normal = Eigen::Vector2f(-seg.y(), seg.x()) / len;
                    }
                }
            }
        }
    }

    if (hit) {
        out_dist = min_s;
        out_normal = best_normal;
        return true;
    }

    return false;
}

// ============================================================================
// Volume3D Implementation
// ============================================================================

float Volume3D::distanceToBoundary(
    const Eigen::Vector3f& pt,
    Eigen::Vector3f& out_closest_pt) const noexcept {
    Eigen::Vector2f closest_2d, normal_2d;
    const float dist_2d = boundary.distanceToBoundary(pt.head<2>(), closest_2d, normal_2d);
    const bool inside_2d = boundary.contains(pt.head<2>());

    // Altitude distances
    const float d_floor = pt.z() - z_min;
    const float d_ceiling = z_max - pt.z();

    if (inside_2d && pt.z() >= z_min && pt.z() <= z_max) {
        // Point is inside the 3D volume: clearance is min to walls or caps
        const float d_z = std::min(d_floor, d_ceiling);
        if (dist_2d <= d_z) {
            out_closest_pt = Eigen::Vector3f(closest_2d.x(), closest_2d.y(), pt.z());
            return dist_2d;
        } else if (d_floor <= d_ceiling) {
            out_closest_pt = Eigen::Vector3f(pt.x(), pt.y(), z_min);
            return d_floor;
        } else {
            out_closest_pt = Eigen::Vector3f(pt.x(), pt.y(), z_max);
            return d_ceiling;
        }
    }

    // Point is outside: clamp z to [z_min, z_max]
    const float clamped_z = std::clamp(pt.z(), z_min, z_max);
    const float delta_z = pt.z() - clamped_z;

    if (inside_2d) {
        // Horizontally inside, vertically outside
        out_closest_pt = Eigen::Vector3f(pt.x(), pt.y(), clamped_z);
        return std::abs(delta_z);
    }

    // Both horizontally and vertically outside
    out_closest_pt = Eigen::Vector3f(closest_2d.x(), closest_2d.y(), clamped_z);
    const float total_dist = std::sqrt(dist_2d * dist_2d + delta_z * delta_z);
    return total_dist;
}

bool Volume3D::rayIntersect(
    const Eigen::Vector3f& origin,
    const Eigen::Vector3f& dir,
    float& out_dist,
    Eigen::Vector3f& out_normal) const noexcept {
    if (dir.squaredNorm() < 1e-6f) {
        return false;
    }

    float min_s = std::numeric_limits<float>::infinity();
    Eigen::Vector3f best_normal = Eigen::Vector3f::Zero();
    bool hit = false;

    // 1. Ray intersection with 2D vertical wall extrusion
    const Eigen::Vector2f dir_2d = dir.head<2>();
    if (dir_2d.squaredNorm() > 1e-6f) {
        const Eigen::Vector2f origin_2d = origin.head<2>();
        const size_t n = boundary.vertices.size();
        const bool ccw = boundary.isCounterClockwise();

        for (size_t i = 0; i < n; ++i) {
            const auto& v1 = boundary.vertices[i];
            const auto& v2 = boundary.vertices[(i + 1) % n];

            const Eigen::Vector2f seg = v2 - v1;
            const float cross = dir_2d.x() * seg.y() - dir_2d.y() * seg.x();
            if (std::abs(cross) < 1e-7f) {
                continue;
            }

            const Eigen::Vector2f delta = v1 - origin_2d;
            const float s = (delta.x() * seg.y() - delta.y() * seg.x()) / cross;
            const float t = (delta.x() * dir_2d.y() - delta.y() * dir_2d.x()) / cross;

            if (s > 1e-3f && t >= 0.0f && t <= 1.0f) {
                const float z_hit = origin.z() + s * dir.z();
                if (z_hit >= (z_min - 1e-3f) && z_hit <= (z_max + 1e-3f)) {
                    if (s < min_s) {
                        min_s = s;
                        hit = true;
                        const float len = seg.norm();
                        if (len > 1e-6f) {
                            if (ccw) {
                                best_normal = Eigen::Vector3f(seg.y() / len, -seg.x() / len, 0.0f);
                            } else {
                                best_normal = Eigen::Vector3f(-seg.y() / len, seg.x() / len, 0.0f);
                            }
                        }
                    }
                }
            }
        }
    }

    // 2. Ray intersection with floor cap plane (z = z_min)
    if (std::abs(dir.z()) > 1e-5f) {
        const float s_floor = (z_min - origin.z()) / dir.z();
        if (s_floor > 1e-3f && s_floor < min_s) {
            const Eigen::Vector3f hit_pt = origin + s_floor * dir;
            if (boundary.contains(hit_pt.head<2>())) {
                min_s = s_floor;
                hit = true;
                best_normal = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
            }
        }

        // 3. Ray intersection with ceiling cap plane (z = z_max)
        const float s_ceiling = (z_max - origin.z()) / dir.z();
        if (s_ceiling > 1e-3f && s_ceiling < min_s) {
            const Eigen::Vector3f hit_pt = origin + s_ceiling * dir;
            if (boundary.contains(hit_pt.head<2>())) {
                min_s = s_ceiling;
                hit = true;
                best_normal = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
            }
        }
    }

    if (hit) {
        out_dist = min_s;
        out_normal = best_normal;
        return true;
    }

    return false;
}

// ============================================================================
// Geofence3D Implementation
// ============================================================================

Geofence3D::Geofence3D()
    : params_{} {}

Geofence3D::Geofence3D(const GeofenceParams& params)
    : params_(params) {}

void Geofence3D::addVolume(const Volume3D& volume) {
    volumes_.push_back(volume);
}

void Geofence3D::clearVolumes() noexcept {
    volumes_.clear();
}

void Geofence3D::setParams(const GeofenceParams& params) noexcept {
    params_ = params;
}

float Geofence3D::computeStoppingDistance(float speed) const noexcept {
    if (speed <= 0.0f) {
        return 0.0f;
    }
    const float a_max = std::max(0.1f, params_.a_max);
    // d_stop = v * t_reaction + v^2 / (2 * a_max)
    const float d_reaction = speed * params_.t_reaction;
    const float d_braking = (speed * speed) / (2.0f * a_max);
    return d_reaction + d_braking;
}

GeofenceStatus Geofence3D::evaluate(
    const Eigen::Vector3f& position,
    const Eigen::Vector3f& velocity,
    float dt) const {
    GeofenceStatus status;
    status.is_safe = true;
    status.breach_imminent = false;
    status.breached = false;
    status.hold_point = position;

    const float speed = velocity.norm();
    status.dynamic_stopping_distance = computeStoppingDistance(speed);

    Eigen::Vector3f unit_v = Eigen::Vector3f::Zero();
    if (speed > 1e-3f) {
        unit_v = velocity / speed;
    }
    const Eigen::Vector3f predicted_pos = position + unit_v * status.dynamic_stopping_distance;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    for (const auto& vol : volumes_) {
        Eigen::Vector3f closest_pt;
        const float dist_to_bound = vol.distanceToBoundary(position, closest_pt);
        if (dist_to_bound < status.min_distance_to_boundary) {
            status.min_distance_to_boundary = dist_to_bound;
        }

        float ray_dist = std::numeric_limits<float>::infinity();
        Eigen::Vector3f ray_normal;
        const bool ray_hit = (speed > 0.05f) && vol.rayIntersect(position, unit_v, ray_dist, ray_normal);

        if (ray_hit && ray_dist < status.trajectory_distance_to_boundary) {
            status.trajectory_distance_to_boundary = ray_dist;
        }

        if (vol.type == GeofenceVolumeType::KeepIn) {
            const bool currently_inside = vol.contains(position);
            const bool predicted_inside = vol.contains(predicted_pos);

            if (!currently_inside) {
                // Breach: Vehicle is already outside KeepIn zone
                status.breached = true;
                status.is_safe = false;
                status.active_volume_id = vol.id;

                // Compute inward recovery vector with zero-length protection
                const Eigen::Vector3f delta_rec = closest_pt - position;
                const float rec_norm = delta_rec.norm();
                const Eigen::Vector3f recovery_dir = (rec_norm > 1e-3f) ? (delta_rec / rec_norm).eval() : Eigen::Vector3f::UnitZ();

                status.hold_point = closest_pt + recovery_dir * params_.buffer_dist;
                status.braking_velocity_cmd = recovery_dir * params_.recovery_speed;

                ss << "[BREACH] Vehicle outside Keep-In perimeter '" << vol.id
                   << "' (dist: " << dist_to_bound << "m). Commanding recovery velocity.";
                break;
            } else if (!predicted_inside || (ray_hit && ray_dist <= (status.dynamic_stopping_distance + params_.buffer_dist)) ||
                       dist_to_bound <= params_.buffer_dist) {
                // Predictive breach imminent
                status.breach_imminent = true;
                status.is_safe = false;
                status.active_volume_id = vol.id;

                // Command maximum braking deceleration along velocity vector
                const float new_speed = std::max(0.0f, speed - params_.a_max * dt);
                status.braking_velocity_cmd = unit_v * new_speed;

                // Hold point positioned safely before boundary buffer
                const float safe_stop_dist = std::min(status.dynamic_stopping_distance,
                    std::max(0.0f, (ray_hit ? ray_dist : dist_to_bound) - params_.buffer_dist));
                status.hold_point = position + unit_v * safe_stop_dist;

                ss << "[PROXIMITY] Keep-In boundary '" << vol.id << "' imminent (d_stop: "
                   << status.dynamic_stopping_distance << "m, clearance: " << dist_to_bound
                   << "m). Maximum braking engaged.";
            }
        } else if (vol.type == GeofenceVolumeType::KeepOut) {
            const bool currently_inside = vol.contains(position);
            const bool predicted_inside = vol.contains(predicted_pos);

            if (currently_inside) {
                // Breach: Vehicle penetrated KeepOut no-fly zone
                status.breached = true;
                status.is_safe = false;
                status.active_volume_id = vol.id;

                // Push outward along normal with zero-length protection
                const Eigen::Vector3f delta_egress = closest_pt - position;
                const float egress_norm = delta_egress.norm();
                const Eigen::Vector3f egress_dir = (egress_norm > 1e-3f) ? (delta_egress / egress_norm).eval() : (-unit_v);

                status.hold_point = closest_pt + egress_dir * params_.buffer_dist;
                status.braking_velocity_cmd = egress_dir * params_.recovery_speed;

                ss << "[BREACH] Vehicle entered Keep-Out no-fly zone '" << vol.id
                   << "'. Immediate egress commanded.";
                break;
            } else if (predicted_inside || (ray_hit && ray_dist <= (status.dynamic_stopping_distance + params_.buffer_dist)) ||
                       dist_to_bound <= params_.buffer_dist) {
                // Approaching KeepOut zone
                status.breach_imminent = true;
                status.is_safe = false;
                status.active_volume_id = vol.id;

                const float new_speed = std::max(0.0f, speed - params_.a_max * dt);
                status.braking_velocity_cmd = unit_v * new_speed;

                const float safe_stop_dist = std::min(status.dynamic_stopping_distance,
                    std::max(0.0f, (ray_hit ? ray_dist : dist_to_bound) - params_.buffer_dist));
                status.hold_point = position + unit_v * safe_stop_dist;

                ss << "[PROXIMITY] Keep-Out zone '" << vol.id << "' obstacle ahead (d_stop: "
                   << status.dynamic_stopping_distance << "m, dist: " << dist_to_bound
                   << "m). Maximum braking engaged.";
            }
        }
    }

    if (status.is_safe) {
        status.braking_velocity_cmd = velocity;
        status.hold_point = position;
        ss << "[NOMINAL] 3D airspace geofence clear (min_clearance: "
           << status.min_distance_to_boundary << "m, d_stop: "
           << status.dynamic_stopping_distance << "m).";
    }

    status.status_message = ss.str();
    return status;
}

} // namespace px4_airsim_autonomy::production
