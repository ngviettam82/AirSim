"""
Boustrophedon Survey Path Planning and Photogrammetry Helper.

Converts GPS polygon boundaries into optimal lawnmower sweep tracks using
the Rotating Calipers orientation search, computing optical GSD, overlap intervals,
and photo trigger coordinates.
"""

import math
from typing import List, Dict, Tuple, Any

EARTH_RADIUS_M = 6378137.0


def latlon_to_local_enu(lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """Convert (lat, lon) degrees to local ENU (East, North) meters relative to a reference origin."""
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    ref_lat_rad = math.radians(ref_lat)

    north = d_lat * EARTH_RADIUS_M
    east = d_lon * EARTH_RADIUS_M * math.cos(ref_lat_rad)
    return east, north


def local_enu_to_latlon(east: float, north: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """Convert local ENU (East, North) meters back to (lat, lon) degrees."""
    ref_lat_rad = math.radians(ref_lat)
    d_lat = north / EARTH_RADIUS_M
    d_lon = east / (EARTH_RADIUS_M * math.cos(ref_lat_rad))

    lat = ref_lat + math.degrees(d_lat)
    lon = ref_lon + math.degrees(d_lon)
    return lat, lon


def compute_convex_hull(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Monotone chain 2D convex hull algorithm O(N log N)."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def compute_projected_width(points: List[Tuple[float, float]], angle_rad: float) -> float:
    """Compute cross-track projection width perpendicular to sweep angle."""
    nx = -math.sin(angle_rad)
    ny = math.cos(angle_rad)
    projections = [p[0] * nx + p[1] * ny for p in points]
    return max(projections) - min(projections)


def find_optimal_sweep_angle(polygon_enu: List[Tuple[float, float]], strip_spacing: float) -> Tuple[float, float, int]:
    """
    Search for minimum-strip sweep angle using Freeman-Shapira edge orientations
    and fine angular resolution.
    Returns (optimal_angle_rad, optimal_width_m, strip_count).
    """
    hull = compute_convex_hull(polygon_enu)
    if len(hull) < 3:
        return 0.0, 50.0, 2

    best_angle = 0.0
    min_strips = 999999
    min_width = float("inf")

    # Sample edge orientations of convex hull
    candidate_angles = []
    n = len(hull)
    for i in range(n):
        p1 = hull[i]
        p2 = hull[(i + 1) % n]
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        edge_angle = math.atan2(dy, dx)
        candidate_angles.append(edge_angle % math.pi)

    # Add 1-degree resolution sweep for thoroughness
    for deg in range(0, 180, 2):
        candidate_angles.append(math.radians(deg))

    for angle in candidate_angles:
        width = compute_projected_width(hull, angle)
        strips = max(1, math.ceil(width / strip_spacing))
        if strips < min_strips or (strips == min_strips and width < min_width):
            min_strips = strips
            min_width = width
            best_angle = angle

    return best_angle, min_width, min_strips


def intersect_sweep_line_with_polygon(
    rotated_poly: List[Tuple[float, float]], y_line: float
) -> List[Tuple[float, float]]:
    """Find horizontal x-segments intersecting the polygon at y = y_line."""
    intersections = []
    n = len(rotated_poly)
    for i in range(n):
        p1 = rotated_poly[i]
        p2 = rotated_poly[(i + 1) % n]

        if (p1[1] <= y_line < p2[1]) or (p2[1] <= y_line < p1[1]):
            t = (y_line - p1[1]) / (p2[1] - p1[1])
            x = p1[0] + t * (p2[0] - p1[0])
            intersections.append(x)

    intersections.sort()
    segments = []
    for i in range(0, len(intersections) - 1, 2):
        segments.append((intersections[i], intersections[i + 1]))
    return segments


def plan_boustrophedon_survey(
    polygon_gps: List[Dict[str, float]],
    altitude_m: float = 30.0,
    speed_m_s: float = 4.0,
    forward_overlap: float = 0.75,
    side_overlap: float = 0.65,
    sensor_width_mm: float = 13.2,
    sensor_height_mm: float = 8.8,
    focal_length_mm: float = 8.8,
    image_width_px: int = 5472,
    image_height_px: int = 3648,
) -> Dict[str, Any]:
    """
    Full enterprise survey planner:
    Computes optical GSD, strip spacing, optimal sweep tracks, and GPS waypoints.
    """
    if len(polygon_gps) < 3:
        raise ValueError("Polygon must contain at least 3 vertices")

    # 1. Optical calculations
    # GSD = (H * S_w) / (f * I_w) in meters per pixel
    gsd_m = (altitude_m * (sensor_width_mm * 0.001)) / ((focal_length_mm * 0.001) * image_width_px)
    gsd_cm = gsd_m * 100.0

    ground_width_m = (altitude_m * sensor_width_mm) / focal_length_mm
    ground_height_m = (altitude_m * sensor_height_mm) / focal_length_mm

    strip_spacing_m = ground_width_m * (1.0 - side_overlap)
    trigger_distance_m = ground_height_m * (1.0 - forward_overlap)

    # 2. Local ENU coordinate transformation
    ref_lat = polygon_gps[0]["lat"]
    ref_lon = polygon_gps[0]["lon"]

    poly_enu = [latlon_to_local_enu(p["lat"], p["lon"], ref_lat, ref_lon) for p in polygon_gps]

    # 3. Rotating Calipers optimal angle search
    theta_star, proj_width, strip_count = find_optimal_sweep_angle(poly_enu, strip_spacing_m)

    # 4. Rotate polygon into sweep-aligned coordinates:
    # x' = x cos(theta) + y sin(theta)  (along-strip)
    # y' = -x sin(theta) + y cos(theta) (cross-strip)
    cos_t = math.cos(theta_star)
    sin_t = math.sin(theta_star)

    rotated_poly = [(p[0] * cos_t + p[1] * sin_t, -p[0] * sin_t + p[1] * cos_t) for p in poly_enu]

    y_vals = [p[1] for p in rotated_poly]
    min_y = min(y_vals)
    max_y = max(y_vals)

    # 5. Generate parallel sweeps
    lead_overshoot_m = 4.0
    waypoints_enu = []
    photo_triggers_enu = []
    direction = 1  # 1: Left to Right, -1: Right to Left
    strip_idx = 0

    cur_y = min_y + strip_spacing_m * 0.5
    while cur_y <= max_y + 0.1:
        segments = intersect_sweep_line_with_polygon(rotated_poly, cur_y)
        for seg in segments:
            x_start = seg[0] - lead_overshoot_m if direction == 1 else seg[1] + lead_overshoot_m
            x_end = seg[1] + lead_overshoot_m if direction == 1 else seg[0] - lead_overshoot_m

            # Rotate back to ENU
            def to_enu(rx, ry):
                return (rx * cos_t - ry * sin_t, rx * sin_t + ry * cos_t)

            p_start_enu = to_enu(x_start, cur_y)
            p_end_enu = to_enu(x_end, cur_y)

            waypoints_enu.append({
                "east": p_start_enu[0], "north": p_start_enu[1], "alt": altitude_m,
                "type": "sweep_start", "strip": strip_idx
            })
            waypoints_enu.append({
                "east": p_end_enu[0], "north": p_end_enu[1], "alt": altitude_m,
                "type": "sweep_end", "strip": strip_idx
            })

            # Place camera triggers along active segment
            active_start = seg[0] if direction == 1 else seg[1]
            active_end = seg[1] if direction == 1 else seg[0]
            seg_len = abs(active_end - active_start)
            triggers_in_seg = max(1, int(seg_len / trigger_distance_m))
            for t_i in range(triggers_in_seg + 1):
                t_frac = t_i / max(1, triggers_in_seg)
                tx = active_start + t_frac * (active_end - active_start)
                p_trig_enu = to_enu(tx, cur_y)
                photo_triggers_enu.append({
                    "east": p_trig_enu[0], "north": p_trig_enu[1], "alt": altitude_m
                })

            direction *= -1
            strip_idx += 1

        cur_y += strip_spacing_m

    # 6. Convert waypoints and photo triggers to GPS
    waypoints_gps = []
    total_dist = 0.0
    prev_pt = None

    for wp in waypoints_enu:
        lat, lon = local_enu_to_latlon(wp["east"], wp["north"], ref_lat, ref_lon)
        if prev_pt is not None:
            total_dist += math.hypot(wp["east"] - prev_pt[0], wp["north"] - prev_pt[1])
        prev_pt = (wp["east"], wp["north"])

        waypoints_gps.append({
            "lat": lat,
            "lon": lon,
            "alt": wp["alt"],
            "type": wp["type"],
            "strip": wp["strip"]
        })

    photo_triggers_gps = []
    for pt in photo_triggers_enu:
        lat, lon = local_enu_to_latlon(pt["east"], pt["north"], ref_lat, ref_lon)
        photo_triggers_gps.append({"lat": lat, "lon": lon, "alt": pt["alt"]})

    est_flight_time_s = total_dist / max(1.0, speed_m_s)

    # 7. Compute polygon area
    area_m2 = 0.0
    n_poly = len(poly_enu)
    for i in range(n_poly):
        j = (i + 1) % n_poly
        area_m2 += poly_enu[i][0] * poly_enu[j][1] - poly_enu[j][0] * poly_enu[i][1]
    area_m2 = abs(area_m2) * 0.5

    return {
        "success": True,
        "optical": {
            "gsd_cm": round(gsd_cm, 2),
            "ground_footprint_w_m": round(ground_width_m, 1),
            "ground_footprint_h_m": round(ground_height_m, 1),
            "strip_spacing_m": round(strip_spacing_m, 1),
            "trigger_distance_m": round(trigger_distance_m, 1),
            "blur_speed_limit_m_s": round((0.5 * gsd_m) / (1.0 / 1000.0), 1),
        },
        "mission": {
            "optimal_angle_deg": round(math.degrees(theta_star), 1),
            "strip_count": strip_idx,
            "total_distance_m": round(total_dist, 1),
            "estimated_time_s": round(est_flight_time_s, 1),
            "photo_count": len(photo_triggers_gps),
            "survey_area_m2": round(area_m2, 1),
            "survey_area_acres": round(area_m2 / 4046.86, 2),
        },
        "waypoints": waypoints_gps,
        "photo_triggers": photo_triggers_gps,
    }

