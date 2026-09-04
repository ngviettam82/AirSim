#include "px4_airsim_autonomy/production/boustrophedon_planner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace px4_airsim_autonomy::production {

std::string SweepWaypoint::typeString() const {
    switch (type) {
        case WaypointType::SurveyEntry:   return "SurveyEntry";
        case WaypointType::SweepStart:    return "SweepStart";
        case WaypointType::SweepEnd:      return "SweepEnd";
        case WaypointType::TurnOvershoot: return "TurnOvershoot";
        case WaypointType::TurnEntry:     return "TurnEntry";
        case WaypointType::SurveyExit:    return "SurveyExit";
        default:                          return "Unknown";
    }
}

bool PlannerConfig::isValid() const {
    return strip_spacing_m > 0.0 &&
           flight_altitude_agl_m > 0.0 &&
           survey_speed_m_s > 0.0 &&
           turn_speed_m_s > 0.0 &&
           turn_overshoot_m >= 0.0 &&
           angular_resolution_deg > 0.0;
}

double BoustrophedonPlanner::computePolygonArea(const std::vector<Eigen::Vector2d>& polygon) {
    const size_t n = polygon.size();
    if (n < 3) return 0.0;

    double area2 = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const size_t next = (i + 1) % n;
        area2 += (polygon[i].x() * polygon[next].y() - polygon[next].x() * polygon[i].y());
    }
    return 0.5 * std::abs(area2);
}

std::vector<Eigen::Vector2d> BoustrophedonPlanner::computeConvexHull(
    const std::vector<Eigen::Vector2d>& vertices) {
    const size_t n = vertices.size();
    if (n < 3) return vertices;

    // Sort points lexicographically (by x, then by y)
    std::vector<Eigen::Vector2d> pts = vertices;
    std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        if (std::abs(a.x() - b.x()) > 1e-9) {
            return a.x() < b.x();
        }
        return a.y() < b.y();
    });

    // Remove duplicates
    pts.erase(std::unique(pts.begin(), pts.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return (a - b).norm() < 1e-7;
    }), pts.end());

    if (pts.size() < 3) return pts;

    // 2D cross product: (b - a) x (c - a)
    auto cross2d = [](const Eigen::Vector2d& o, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    };

    std::vector<Eigen::Vector2d> hull;
    hull.reserve(2 * pts.size());

    // Build lower hull
    for (const auto& p : pts) {
        while (hull.size() >= 2 && cross2d(hull[hull.size() - 2], hull.back(), p) <= 1e-9) {
            hull.pop_back();
        }
        hull.push_back(p);
    }

    // Build upper hull
    const size_t lower_hull_size = hull.size();
    for (int i = static_cast<int>(pts.size()) - 2; i >= 0; --i) {
        while (hull.size() > lower_hull_size && cross2d(hull[hull.size() - 2], hull.back(), pts[i]) <= 1e-9) {
            hull.pop_back();
        }
        hull.push_back(pts[i]);
    }

    // Pop the last point as it is identical to hull[0]
    hull.pop_back();

    return hull;
}

double BoustrophedonPlanner::computeProjectedWidth(const std::vector<Eigen::Vector2d>& vertices,
                                                  double sweep_angle_rad) {
    if (vertices.empty()) return 0.0;

    // Normal vector perpendicular to the sweep direction θ: n = (-sin θ, cos θ)
    const double nx = -std::sin(sweep_angle_rad);
    const double ny =  std::cos(sweep_angle_rad);

    double min_proj = std::numeric_limits<double>::infinity();
    double max_proj = -std::numeric_limits<double>::infinity();

    for (const auto& v : vertices) {
        const double proj = v.x() * nx + v.y() * ny;
        if (proj < min_proj) min_proj = proj;
        if (proj > max_proj) max_proj = proj;
    }

    return std::max(0.0, max_proj - min_proj);
}

RotatingCalipersResult BoustrophedonPlanner::findOptimalOrientation(
    const std::vector<Eigen::Vector2d>& polygon_vertices,
    double strip_spacing_m,
    double angular_step_deg) {

    RotatingCalipersResult result;
    if (polygon_vertices.size() < 3 || strip_spacing_m <= 0.0) {
        return result;
    }

    // Compute convex hull
    result.convex_hull = computeConvexHull(polygon_vertices);
    const auto& hull = result.convex_hull;
    if (hull.size() < 3) {
        return result;
    }

    // Generate candidate angles:
    // 1. All edge orientations of the convex hull (Freeman-Shapira theorem)
    std::vector<double> candidate_angles;
    candidate_angles.reserve(hull.size() * 2 + 180);

    for (size_t i = 0; i < hull.size(); ++i) {
        const size_t next = (i + 1) % hull.size();
        const Eigen::Vector2d edge = hull[next] - hull[i];
        if (edge.norm() > 1e-6) {
            double angle = std::atan2(edge.y(), edge.x());
            // Normalize to [0, pi)
            while (angle < 0.0) angle += M_PI;
            while (angle >= M_PI) angle -= M_PI;
            candidate_angles.push_back(angle);

            // Also test orthogonal orientation (calipers swapped)
            double orth_angle = angle + M_PI * 0.5;
            if (orth_angle >= M_PI) orth_angle -= M_PI;
            candidate_angles.push_back(orth_angle);
        }
    }

    // 2. Fine angular discretization for arbitrary polygon geometries
    const double step_rad = std::max(0.1, angular_step_deg) * (M_PI / 180.0);
    for (double a = 0.0; a < M_PI; a += step_rad) {
        candidate_angles.push_back(a);
    }

    // Evaluate each candidate angle to minimize strip count ceil(W_proj / S_strip)
    int min_strips = std::numeric_limits<int>::max();
    double min_width = std::numeric_limits<double>::infinity();
    double best_angle = 0.0;
    double best_edge_len = 0.0;

    for (double angle : candidate_angles) {
        const double w_proj = computeProjectedWidth(hull, angle);
        int strips = static_cast<int>(std::ceil(w_proj / strip_spacing_m));
        if (strips < 1) strips = 1;

        // Minimization criteria:
        // Priority 1: Minimum strip count (strictly minimizes turns)
        // Priority 2: Minimum projected width (tightest fit)
        bool is_better = false;
        if (strips < min_strips) {
            is_better = true;
        } else if (strips == min_strips) {
            if (w_proj < min_width - 1e-4) {
                is_better = true;
            }
        }

        if (is_better) {
            min_strips = strips;
            min_width = w_proj;
            best_angle = angle;
        }
    }

    result.optimal_angle_rad = best_angle;
    result.optimal_angle_deg = best_angle * (180.0 / M_PI);
    result.projected_width_m = min_width;
    result.strip_count = min_strips;

    // Find the longest hull edge close to the best angle
    for (size_t i = 0; i < hull.size(); ++i) {
        const size_t next = (i + 1) % hull.size();
        const Eigen::Vector2d edge = hull[next] - hull[i];
        double edge_angle = std::atan2(edge.y(), edge.x());
        while (edge_angle < 0.0) edge_angle += M_PI;
        while (edge_angle >= M_PI) edge_angle -= M_PI;

        if (std::abs(edge_angle - best_angle) < 0.05 || std::abs(edge_angle - (best_angle + M_PI * 0.5)) < 0.05) {
            if (edge.norm() > best_edge_len) {
                best_edge_len = edge.norm();
            }
        }
    }
    result.edge_length_m = best_edge_len;

    return result;
}

std::vector<std::pair<double, double>> BoustrophedonPlanner::intersectSweepLine(
    const std::vector<Eigen::Vector2d>& rotated_polygon,
    double y_line) {

    std::vector<std::pair<double, double>> intervals;
    const size_t n = rotated_polygon.size();
    if (n < 3) return intervals;

    std::vector<double> x_intersections;
    x_intersections.reserve(4);

    for (size_t i = 0; i < n; ++i) {
        const size_t next = (i + 1) % n;
        const auto& p1 = rotated_polygon[i];
        const auto& p2 = rotated_polygon[next];

        const double y_min = std::min(p1.y(), p2.y());
        const double y_max = std::max(p1.y(), p2.y());

        // Half-open interval [y_min, y_max) to prevent double counting at vertices
        if (y_line >= y_min && y_line < y_max) {
            const double dy = p2.y() - p1.y();
            if (std::abs(dy) > 1e-7) {
                const double t = (y_line - p1.y()) / dy;
                const double x_int = p1.x() + t * (p2.x() - p1.x());
                x_intersections.push_back(x_int);
            }
        }
    }

    std::sort(x_intersections.begin(), x_intersections.end());

    // Pair up intersections [x_entry, x_exit]
    for (size_t i = 0; i + 1 < x_intersections.size(); i += 2) {
        if (x_intersections[i + 1] - x_intersections[i] > 1e-4) {
            intervals.emplace_back(x_intersections[i], x_intersections[i + 1]);
        }
    }

    return intervals;
}

std::vector<Eigen::Vector2d> BoustrophedonPlanner::offsetPolygon(
    const std::vector<Eigen::Vector2d>& polygon,
    double margin_m) {

    if (std::abs(margin_m) < 1e-6 || polygon.size() < 3) {
        return polygon;
    }

    // Offset edges outward (+) or inward (-) along outward normal
    const size_t n = polygon.size();
    std::vector<Eigen::Vector2d> offset_poly;
    offset_poly.reserve(n);

    // Compute outward normals for all edges
    std::vector<Eigen::Vector2d> normals(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t next = (i + 1) % n;
        Eigen::Vector2d edge = polygon[next] - polygon[i];
        // 90° clockwise normal: (dy, -dx) assuming CCW polygon
        Eigen::Vector2d norm(edge.y(), -edge.x());
        const double len = norm.norm();
        normals[i] = (len > 1e-6) ? (norm / len) : Eigen::Vector2d(0, 1);
    }

    for (size_t i = 0; i < n; ++i) {
        const size_t prev = (i + n - 1) % n;
        // Vertex offset vector is the average of adjacent edge normals
        Eigen::Vector2d v_norm = normals[prev] + normals[i];
        const double v_len = v_norm.norm();
        if (v_len > 1e-6) {
            v_norm /= v_len;
            offset_poly.push_back(polygon[i] + margin_m * v_norm);
        } else {
            offset_poly.push_back(polygon[i]);
        }
    }

    return offset_poly;
}

std::vector<SweepWaypoint> BoustrophedonPlanner::planMission(
    const std::vector<Eigen::Vector2d>& polygon_vertices,
    const PlannerConfig& config) {

    std::vector<SweepWaypoint> waypoints;
    if (polygon_vertices.size() < 3 || !config.isValid()) {
        return waypoints;
    }

    // Apply boundary margin if requested
    const auto working_poly = (std::abs(config.boundary_margin_m) > 1e-6) ?
        offsetPolygon(polygon_vertices, config.boundary_margin_m) : polygon_vertices;

    // 1. Determine optimal orientation angle θ*
    double theta_star = 0.0;
    if (config.force_custom_angle) {
        theta_star = config.custom_angle_rad;
    } else {
        auto calipers = findOptimalOrientation(working_poly, config.strip_spacing_m, config.angular_resolution_deg);
        theta_star = calipers.optimal_angle_rad;
    }

    // 2. Rotate polygon into aligned frame by -θ*
    const double cos_t = std::cos(-theta_star);
    const double sin_t = std::sin(-theta_star);

    std::vector<Eigen::Vector2d> rotated_poly;
    rotated_poly.reserve(working_poly.size());

    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();

    for (const auto& pt : working_poly) {
        const double rx = pt.x() * cos_t - pt.y() * sin_t;
        const double ry = pt.x() * sin_t + pt.y() * cos_t;
        rotated_poly.emplace_back(rx, ry);

        if (ry < y_min) y_min = ry;
        if (ry > y_max) y_max = ry;
    }

    const double total_height = y_max - y_min;
    if (total_height <= 0.0) return waypoints;

    // 3. Compute parallel strip locations
    int strip_count = static_cast<int>(std::ceil(total_height / config.strip_spacing_m));
    if (strip_count < 1) strip_count = 1;

    // Center strips symmetrically within projected bounds
    const double span_used = static_cast<double>(strip_count - 1) * config.strip_spacing_m;
    const double y_start_offset = y_min + 0.5 * (total_height - span_used);

    // Helpers to transform from aligned frame back to world frame (+θ*)
    const double inv_cos = std::cos(theta_star);
    const double inv_sin = std::sin(theta_star);

    auto toWorld2D = [&](double x_aligned, double y_aligned) -> Eigen::Vector2d {
        return Eigen::Vector2d(
            x_aligned * inv_cos - y_aligned * inv_sin,
            x_aligned * inv_sin + y_aligned * inv_cos
        );
    };

    auto toWorld3D = [&](double x_aligned, double y_aligned) -> Eigen::Vector3d {
        Eigen::Vector2d w2 = toWorld2D(x_aligned, y_aligned);
        return Eigen::Vector3d(w2.x(), w2.y(), config.flight_altitude_agl_m);
    };

    // 4. Generate Boustrophedon sweep waypoints
    size_t global_idx = 0;
    const double overshoot = config.turn_overshoot_m;

    for (int strip_i = 0; strip_i < strip_count; ++strip_i) {
        const double y_strip = y_start_offset + strip_i * config.strip_spacing_m;
        auto intervals = intersectSweepLine(rotated_poly, y_strip);

        if (intervals.empty()) {
            continue;
        }

        // Use the outermost bounds for full coverage across the strip
        double x_left = intervals.front().first;
        double x_right = intervals.back().second;

        // Alternating directions (Boustrophedon / lawnmower)
        // Even strips (0, 2, 4...): Left to Right
        // Odd strips (1, 3, 5...): Right to Left
        const bool left_to_right = (strip_i % 2 == 0);
        const double x_sweep_start = left_to_right ? x_left : x_right;
        const double x_sweep_end = left_to_right ? x_right : x_left;
        const double dir_sign = left_to_right ? 1.0 : -1.0;

        // Flight heading in world frame along this strip
        const double strip_yaw = left_to_right ? theta_star : (theta_star + M_PI);

        // A. Initial mission entry (prior to first sweep line)
        if (strip_i == 0) {
            SweepWaypoint wp_entry;
            wp_entry.position = toWorld3D(x_sweep_start - dir_sign * overshoot, y_strip);
            wp_entry.yaw_rad = strip_yaw;
            wp_entry.target_speed_m_s = config.survey_speed_m_s;
            wp_entry.type = WaypointType::SurveyEntry;
            wp_entry.strip_index = strip_i;
            wp_entry.is_photo_trigger_zone = false;
            wp_entry.global_index = global_idx++;
            waypoints.push_back(wp_entry);
        }

        // B. Sweep Start (photo capture begins here)
        {
            SweepWaypoint wp_start;
            wp_start.position = toWorld3D(x_sweep_start, y_strip);
            wp_start.yaw_rad = strip_yaw;
            wp_start.target_speed_m_s = config.survey_speed_m_s;
            wp_start.type = WaypointType::SweepStart;
            wp_start.strip_index = strip_i;
            wp_start.is_photo_trigger_zone = true;
            wp_start.global_index = global_idx++;
            waypoints.push_back(wp_start);
        }

        // C. Sweep End (photo capture active through this point)
        {
            SweepWaypoint wp_end;
            wp_end.position = toWorld3D(x_sweep_end, y_strip);
            wp_end.yaw_rad = strip_yaw;
            wp_end.target_speed_m_s = config.survey_speed_m_s;
            wp_end.type = WaypointType::SweepEnd;
            wp_end.strip_index = strip_i;
            wp_end.is_photo_trigger_zone = true;
            wp_end.global_index = global_idx++;
            waypoints.push_back(wp_end);
        }

        // D. Turn transition to next strip
        if (strip_i + 1 < strip_count) {
            const double y_next = y_start_offset + (strip_i + 1) * config.strip_spacing_m;

            if (config.turn_style == TurnStyle::SquareOvershoot) {
                // Step 1: Overshoot parallel to current strip direction outside survey polygon
                SweepWaypoint wp_overshoot;
                wp_overshoot.position = toWorld3D(x_sweep_end + dir_sign * overshoot, y_strip);
                wp_overshoot.yaw_rad = strip_yaw;
                wp_overshoot.target_speed_m_s = config.turn_speed_m_s;
                wp_overshoot.type = WaypointType::TurnOvershoot;
                wp_overshoot.strip_index = strip_i;
                wp_overshoot.is_photo_trigger_zone = false;
                wp_overshoot.global_index = global_idx++;
                waypoints.push_back(wp_overshoot);

                // Step 2: Lateral translation to aligned entry point of next strip
                // Next strip will fly in reverse direction (-dir_sign)
                SweepWaypoint wp_entry_next;
                wp_entry_next.position = toWorld3D(x_sweep_end + dir_sign * overshoot, y_next);
                // Turn heading toward next strip direction
                const double next_strip_yaw = (!left_to_right) ? theta_star : (theta_star + M_PI);
                wp_entry_next.yaw_rad = next_strip_yaw;
                wp_entry_next.target_speed_m_s = config.turn_speed_m_s;
                wp_entry_next.type = WaypointType::TurnEntry;
                wp_entry_next.strip_index = strip_i + 1;
                wp_entry_next.is_photo_trigger_zone = false;
                wp_entry_next.global_index = global_idx++;
                waypoints.push_back(wp_entry_next);
            }
        } else {
            // Final mission exit
            SweepWaypoint wp_exit;
            wp_exit.position = toWorld3D(x_sweep_end + dir_sign * overshoot, y_strip);
            wp_exit.yaw_rad = strip_yaw;
            wp_exit.target_speed_m_s = config.survey_speed_m_s;
            wp_exit.type = WaypointType::SurveyExit;
            wp_exit.strip_index = strip_i;
            wp_exit.is_photo_trigger_zone = false;
            wp_exit.global_index = global_idx++;
            waypoints.push_back(wp_exit);
        }
    }

    return waypoints;
}

std::vector<SweepWaypoint> BoustrophedonPlanner::planMission3D(
    const std::vector<Eigen::Vector3d>& polygon_vertices_3d,
    const PlannerConfig& config) {

    std::vector<Eigen::Vector2d> poly_2d;
    poly_2d.reserve(polygon_vertices_3d.size());

    double avg_z = 0.0;
    for (const auto& v : polygon_vertices_3d) {
        poly_2d.emplace_back(v.x(), v.y());
        avg_z += v.z();
    }
    if (!polygon_vertices_3d.empty()) {
        avg_z /= static_cast<double>(polygon_vertices_3d.size());
    }

    PlannerConfig cfg = config;
    if (cfg.flight_altitude_agl_m <= 0.0 && avg_z > 0.0) {
        cfg.flight_altitude_agl_m = avg_z;
    }

    return planMission(poly_2d, cfg);
}

MissionSummary BoustrophedonPlanner::computeMissionSummary(
    const std::vector<SweepWaypoint>& waypoints,
    const std::vector<Eigen::Vector2d>& polygon_vertices,
    const RotatingCalipersResult& result) {

    MissionSummary summary;
    summary.orientation = result;
    summary.total_waypoints = waypoints.size();
    summary.total_strips = result.strip_count;
    summary.covered_area_m2 = computePolygonArea(polygon_vertices);

    if (waypoints.size() < 2) {
        return summary;
    }

    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        const double segment_dist = (waypoints[i + 1].position - waypoints[i].position).norm();
        summary.total_flight_distance_m += segment_dist;

        if (waypoints[i].is_photo_trigger_zone && waypoints[i + 1].is_photo_trigger_zone) {
            summary.active_survey_distance_m += segment_dist;
            summary.estimated_flight_time_s += segment_dist / waypoints[i].target_speed_m_s;
        } else {
            summary.turn_distance_m += segment_dist;
            summary.estimated_flight_time_s += segment_dist / waypoints[i].target_speed_m_s;
        }
    }

    return summary;
}

} // namespace px4_airsim_autonomy::production
