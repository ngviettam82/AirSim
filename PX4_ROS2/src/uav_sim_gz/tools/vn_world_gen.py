#!/usr/bin/env python3
"""Generate a Gazebo scene for a real site, from OSM.

Map data (c) OpenStreetMap contributors, ODbL 1.0.
"""

import argparse
import json
import math
import os
import random
from typing import Dict, List, Sequence, Tuple

Point = Tuple[float, float]

# PX4 CONSTANTS_RADIUS_OF_EARTH, src/lib/geo/geo.h.
EARTH_RADIUS_M = 6371000.0

# HCMUT campus 1; datum cross-checked against fetched names, README 3b.
HCMUT_LATITUDE_DEG = 10.7729
HCMUT_LONGITUDE_DEG = 106.6577
HCMUT_ELEVATION_M = 10.0

# ENU tesla; only Gazebo's own magnetometer reads it, not PX4.
HCMC_MAGNETIC_FIELD_ENU = (0.3e-06, 41.0e-06, -13.3e-06)

DEFAULT_BUILDING_HEIGHT_M = 9.0
METRES_PER_LEVEL = 3.0

# Carriageway widths by road class, metres.
ROAD_WIDTHS = {
    "primary": 12.0, "primary_link": 8.0, "secondary": 10.0, "secondary_link": 7.0,
    "tertiary": 8.0, "residential": 7.0, "unclassified": 6.0, "service": 4.0,
    "living_street": 5.0, "pedestrian": 4.0, "footway": 2.5, "path": 2.0,
    "steps": 2.0, "cycleway": 2.0, "construction": 5.0, "track": 3.0,
}

# Painted flat, visual only; ground plane already carries the collision.
SURFACE_STYLES = {
    "grass": (0.32, 0.48, 0.24), "park": (0.30, 0.50, 0.26),
    "pitch": (0.26, 0.46, 0.22), "garden": (0.34, 0.50, 0.28),
    "forest": (0.22, 0.38, 0.20), "wood": (0.22, 0.38, 0.20),
    "water": (0.20, 0.36, 0.52), "parking": (0.34, 0.34, 0.36),
    "construction": (0.48, 0.44, 0.36), "residential": (0.42, 0.40, 0.38),
    "commercial": (0.44, 0.42, 0.42), "industrial": (0.40, 0.40, 0.42),
    "sand": (0.66, 0.60, 0.44), "scrub": (0.34, 0.42, 0.26),
}

ROAD_COLOUR = (0.24, 0.24, 0.26)
FOOTWAY_COLOUR = (0.52, 0.50, 0.47)
BUILDING_COLOUR = (0.62, 0.60, 0.55)
FENCE_COLOUR = (0.40, 0.38, 0.34)
TRUNK_COLOUR = (0.28, 0.20, 0.13)
CANOPY_COLOUR = (0.20, 0.40, 0.18)

FENCE_HEIGHT_M = 2.0
FENCE_THICKNESS_M = 0.15
TREE_TRUNK_RADIUS_M = 0.16
TREE_TRUNK_HEIGHT_M = 3.0
TREE_CANOPY_RADIUS_M = 2.2


def project_to_local_enu(datum: Point, point: Point) -> Point:
    """Mirrors PX4 MapProjection so generated geometry lines up with GPS."""
    datum_latitude = math.radians(datum[0])
    datum_longitude = math.radians(datum[1])
    latitude = math.radians(point[0])
    longitude = math.radians(point[1])

    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_datum = math.sin(datum_latitude)
    cos_datum = math.cos(datum_latitude)
    delta_longitude = longitude - datum_longitude
    cos_delta = math.cos(delta_longitude)

    cos_arc = max(-1.0, min(1.0, sin_datum * sin_latitude + cos_datum * cos_latitude * cos_delta))
    arc = math.acos(cos_arc)
    scale = 1.0 if abs(arc) < 1e-12 else arc / math.sin(arc)

    north = scale * EARTH_RADIUS_M * (
        cos_datum * sin_latitude - sin_datum * cos_latitude * cos_delta)
    east = scale * EARTH_RADIUS_M * cos_latitude * math.sin(delta_longitude)
    return east, north


def signed_area(polygon: Sequence[Point]) -> float:
    total = 0.0
    for index in range(len(polygon)):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % len(polygon)]
        total += x1 * y2 - x2 * y1
    return total / 2.0


def as_simple_ring(points: Sequence[Point]) -> List[Point]:
    """Drops the repeated closing node and orients counter-clockwise."""
    ring = list(points)
    if len(ring) > 1 and math.dist(ring[0], ring[-1]) < 1e-6:
        ring = ring[:-1]
    deduped: List[Point] = []
    for point in ring:
        if not deduped or math.dist(deduped[-1], point) > 1e-6:
            deduped.append(point)
    if len(deduped) >= 3 and signed_area(deduped) < 0.0:
        deduped.reverse()
    return deduped


def centroid(polygon: Sequence[Point]) -> Point:
    area = signed_area(polygon)
    if abs(area) < 1e-9:
        return (sum(p[0] for p in polygon) / len(polygon),
                sum(p[1] for p in polygon) / len(polygon))
    cx = cy = 0.0
    for index in range(len(polygon)):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % len(polygon)]
        cross = x1 * y2 - x2 * y1
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross
    return (cx / (6.0 * area), cy / (6.0 * area))


def point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    inside = False
    x, y = point
    for index in range(len(polygon)):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % len(polygon)]
        if (y1 > y) != (y2 > y):
            crossing = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if crossing > x:
                inside = not inside
    return inside


def distance_to_segment(point: Point, start: Point, end: Point) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared < 1e-12:
        return math.dist(point, start)
    t = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
                     / length_squared))
    return math.dist(point, (start[0] + t * dx, start[1] + t * dy))


def distance_to_polygon(point: Point, polygon: Sequence[Point]) -> float:
    """Zero if inside; else the gap to the nearest edge."""
    if point_in_polygon(point, polygon):
        return 0.0
    return min(distance_to_segment(point, polygon[index],
                                   polygon[(index + 1) % len(polygon)])
               for index in range(len(polygon)))


def material(colour: Tuple[float, float, float], ambient_scale: float = 0.55) -> str:
    r, g, b = colour
    return (f"""<material>
          <ambient>{r * ambient_scale:.3f} {g * ambient_scale:.3f} """
            f"""{b * ambient_scale:.3f} 1</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
          <specular>0.08 0.08 0.08 1</specular>
        </material>""")


def polyline_geometry(ring: Sequence[Point], height: float) -> str:
    points = "\n              ".join(f"<point>{x:.3f} {y:.3f}</point>" for x, y in ring)
    return f"""<polyline>
              <height>{height:.3f}</height>
              {points}
            </polyline>"""


def extruded_link(name: str, ring: Sequence[Point], height: float,
                  colour: Tuple[float, float, float], base_z: float,
                  with_collision: bool) -> str:
    geometry = polyline_geometry(ring, height)
    collision = f"""
        <collision name="collision">
          <geometry>
            {geometry}
          </geometry>
        </collision>""" if with_collision else ""
    return f"""    <link name="{name}">
      <pose>0 0 {base_z:.3f} 0 0 0</pose>{collision}
      <visual name="visual">
        <geometry>
          {geometry}
        </geometry>
        {material(colour)}
      </visual>
    </link>"""


def box_link(name: str, centre: Point, size: Tuple[float, float, float], yaw: float,
             colour: Tuple[float, float, float], base_z: float,
             with_collision: bool) -> str:
    geometry = f"<box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box>"
    collision = f"""
        <collision name="collision">
          <geometry>{geometry}</geometry>
        </collision>""" if with_collision else ""
    return f"""    <link name="{name}">
      <pose>{centre[0]:.3f} {centre[1]:.3f} {base_z + size[2] / 2.0:.3f} 0 0 {yaw:.5f}</pose>{collision}
      <visual name="visual">
        <geometry>{geometry}</geometry>
        {material(colour)}
      </visual>
    </link>"""


def tree_link(name: str, position: Point) -> str:
    """Same shapes in visual and collision; lidar and physics agree."""
    canopy_z = TREE_TRUNK_HEIGHT_M + TREE_CANOPY_RADIUS_M * 0.55
    return f"""    <link name="{name}">
      <pose>{position[0]:.3f} {position[1]:.3f} 0 0 0 0</pose>
      <collision name="trunk_collision">
        <pose>0 0 {TREE_TRUNK_HEIGHT_M / 2.0:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>{TREE_TRUNK_RADIUS_M}</radius>
          <length>{TREE_TRUNK_HEIGHT_M}</length></cylinder></geometry>
      </collision>
      <collision name="canopy_collision">
        <pose>0 0 {canopy_z:.3f} 0 0 0</pose>
        <geometry><sphere><radius>{TREE_CANOPY_RADIUS_M}</radius></sphere></geometry>
      </collision>
      <visual name="trunk_visual">
        <pose>0 0 {TREE_TRUNK_HEIGHT_M / 2.0:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>{TREE_TRUNK_RADIUS_M}</radius>
          <length>{TREE_TRUNK_HEIGHT_M}</length></cylinder></geometry>
        {material(TRUNK_COLOUR)}
      </visual>
      <visual name="canopy_visual">
        <pose>0 0 {canopy_z:.3f} 0 0 0</pose>
        <geometry><sphere><radius>{TREE_CANOPY_RADIUS_M}</radius></sphere></geometry>
        {material(CANOPY_COLOUR)}
      </visual>
    </link>"""


def building_height(tags: Dict[str, str]) -> float:
    """OSM states height or storeys; neither always present."""
    raw_height = tags.get("height")
    if raw_height:
        try:
            return float(str(raw_height).replace("m", "").strip())
        except ValueError:
            pass
    levels = tags.get("building:levels")
    if levels:
        try:
            return float(levels) * METRES_PER_LEVEL
        except ValueError:
            pass
    return DEFAULT_BUILDING_HEIGHT_M


def surface_style(tags: Dict[str, str]):
    for key in ("leisure", "landuse", "natural", "amenity"):
        value = tags.get(key)
        if value in SURFACE_STYLES:
            return value, SURFACE_STYLES[value]
    return None, None


def road_width(tags: Dict[str, str]) -> float:
    explicit = tags.get("width")
    if explicit:
        try:
            return max(1.5, float(str(explicit).replace("m", "").strip()))
        except ValueError:
            pass
    return ROAD_WIDTHS.get(tags.get("highway", ""), 4.0)


def scatter_trees(roads: List[dict], radius_m: float, seed: int,
                  spacing_m: float, limit: int) -> List[dict]:
    """Invented roadside trees, not surveyed; manifest says so, README 3b."""
    generator = random.Random(seed)
    trees: List[dict] = []
    for road in roads:
        if road["kind"] in ("footway", "path", "steps"):
            continue
        half_width = road["width"] / 2.0
        for start, end in zip(road["points"], road["points"][1:]):
            length = math.dist(start, end)
            if length < spacing_m:
                continue
            heading = math.atan2(end[1] - start[1], end[0] - start[0])
            normal = (-math.sin(heading), math.cos(heading))
            steps = int(length // spacing_m)
            for step in range(1, steps + 1):
                t = (step * spacing_m) / length
                side = 1.0 if generator.random() < 0.5 else -1.0
                offset = half_width + generator.uniform(1.5, 3.0)
                position = (start[0] + t * (end[0] - start[0]) + normal[0] * offset * side,
                            start[1] + t * (end[1] - start[1]) + normal[1] * offset * side)
                if math.hypot(*position) > radius_m:
                    continue
                trees.append({"position_enu_m": [round(position[0], 3),
                                                 round(position[1], 3)]})
    generator.shuffle(trees)
    return trees[:limit]


def extract(payload: dict, datum: Point, radius_m: float) -> Dict[str, list]:
    buildings, surfaces, roads, fences = [], [], [], []

    for element in payload.get("elements", []):
        tags = element.get("tags", {})
        if element.get("type") == "way":
            geometry = element.get("geometry")
            if not geometry:
                continue
            local = [project_to_local_enu(datum, (node["lat"], node["lon"]))
                     for node in geometry]
            identifier = element.get("id")

            if "building" in tags:
                ring = as_simple_ring(local)
                if len(ring) < 3:
                    continue
                middle = centroid(ring)
                if math.hypot(*middle) > radius_m:
                    continue
                buildings.append({
                    "id": f"building_{identifier}", "name": tags.get("name", ""),
                    "height_m": round(building_height(tags), 2),
                    "centre_enu_m": [round(middle[0], 3), round(middle[1], 3)],
                    "footprint_enu_m": [[round(x, 3), round(y, 3)] for x, y in ring],
                    "source": f"OpenStreetMap way {identifier}",
                })
                continue

            if "highway" in tags:
                inside = [point for point in local if math.hypot(*point) <= radius_m * 1.2]
                if len(inside) < 2:
                    continue
                roads.append({
                    "id": f"road_{identifier}", "kind": tags["highway"],
                    "width": road_width(tags), "points": inside,
                })
                continue

            if "barrier" in tags and tags["barrier"] in ("fence", "wall", "hedge"):
                inside = [point for point in local if math.hypot(*point) <= radius_m]
                if len(inside) < 2:
                    continue
                fences.append({"id": f"fence_{identifier}", "kind": tags["barrier"],
                               "points": inside})
                continue

            kind, colour = surface_style(tags)
            if kind:
                ring = as_simple_ring(local)
                if len(ring) < 3 or math.hypot(*centroid(ring)) > radius_m * 1.3:
                    continue
                surfaces.append({"id": f"surface_{identifier}", "kind": kind,
                                 "colour": colour,
                                 "ring": [[round(x, 3), round(y, 3)] for x, y in ring]})

    return {"buildings": buildings, "surfaces": surfaces, "roads": roads, "fences": fences}


def scene_model_sdf(model_name: str, features: Dict[str, list], trees: List[dict]) -> str:
    links: List[str] = []

    for surface in features["surfaces"]:
        ring = [tuple(point) for point in surface["ring"]]
        links.append(extruded_link(surface["id"], ring, 0.02,
                                   tuple(surface["colour"]), 0.01, with_collision=False))

    for road in features["roads"]:
        colour = FOOTWAY_COLOUR if road["kind"] in ("footway", "path", "steps") \
            else ROAD_COLOUR
        for index, (start, end) in enumerate(zip(road["points"], road["points"][1:])):
            length = math.dist(start, end)
            if length < 0.5:
                continue
            centre = (0.5 * (start[0] + end[0]), 0.5 * (start[1] + end[1]))
            yaw = math.atan2(end[1] - start[1], end[0] - start[0])
            links.append(box_link(f"{road['id']}_{index}", centre,
                                  (length, road["width"], 0.04), yaw, colour,
                                  0.03, with_collision=False))

    for fence in features["fences"]:
        for index, (start, end) in enumerate(zip(fence["points"], fence["points"][1:])):
            length = math.dist(start, end)
            if length < 0.5:
                continue
            centre = (0.5 * (start[0] + end[0]), 0.5 * (start[1] + end[1]))
            yaw = math.atan2(end[1] - start[1], end[0] - start[0])
            links.append(box_link(f"{fence['id']}_{index}", centre,
                                  (length, FENCE_THICKNESS_M, FENCE_HEIGHT_M), yaw,
                                  FENCE_COLOUR, 0.0, with_collision=True))

    for building in features["buildings"]:
        ring = [tuple(point) for point in building["footprint_enu_m"]]
        links.append(extruded_link(building["id"], ring, building["height_m"],
                                   BUILDING_COLOUR, 0.0, with_collision=True))

    for index, tree in enumerate(trees):
        links.append(tree_link(f"tree_{index}", tuple(tree["position_enu_m"])))

    body = "\n".join(links) if links else "    <!-- nothing within the radius -->"
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by tools/vn_world_gen.py. Do not edit by hand.
     Collision layout: README section 3b. Map data (c) OpenStreetMap contributors, ODbL 1.0. -->
<sdf version="1.9">
  <model name="{model_name}">
    <static>true</static>
{body}
  </model>
</sdf>
"""


def model_config(model_name: str, description: str) -> str:
    return f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    {description}
  </description>
</model>
"""


WORLD_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<!--
  Generated by tools/vn_world_gen.py. Do not edit by hand; edit the generator.
  Real coordinates matter for magnetic field; see README section 3b.

  Site:      {site_name}
  Datum:     {latitude} N, {longitude} E, {elevation} m
  Radius:    {radius} m
  Contents:  {building_count} buildings, {surface_count} ground areas,
             {road_count} ways, {fence_count} fences, {tree_count} trees
  Ground truth: {manifest_name}
  Map data (c) OpenStreetMap contributors, ODbL 1.0
-->
<sdf version="1.9">
  <world name="{world_name}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Contact" filename="gz-sim-contact-system"/>
    <plugin name="gz::sim::systems::Imu" filename="gz-sim-imu-system"/>
    <plugin name="gz::sim::systems::AirPressure" filename="gz-sim-air-pressure-system"/>
    <plugin name="gz::sim::systems::ApplyLinkWrench" filename="gz-sim-apply-link-wrench-system"/>
    <plugin name="gz::sim::systems::NavSat" filename="gz-sim-navsat-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gui fullscreen="false">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.5 0.5 0.5</ambient_light>
        <background_color>0.72 0.80 0.88</background_color>
        <camera_pose>-70 -70 45 0 0.42 0.78</camera_pose>
        <camera_clip><near>0.25</near><far>25000</far></camera_clip>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui><property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <gz-gui><property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="CameraTracking" name="Camera Tracking">
        <gz-gui><property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin name="World control" filename="WorldControl">
        <gz-gui>
          <property type="bool" key="showTitleBar">0</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <play_pause>1</play_pause><step>1</step><start_paused>1</start_paused>
      </plugin>
      <plugin name="World stats" filename="WorldStats">
        <gz-gui>
          <property type="bool" key="showTitleBar">0</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <sim_time>1</sim_time><real_time>1</real_time>
        <real_time_factor>1</real_time_factor><iterations>1</iterations>
      </plugin>
      <plugin name="Entity tree" filename="EntityTree"/>
    </gui>

    <gravity>0 0 -9.8</gravity>
    <magnetic_field>{magnetic_east} {magnetic_north} {magnetic_up}</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.72 0.80 0.88 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Sun angle for near-equator, near-noon at this latitude. -->
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1.1</intensity>
      <direction>0.15 0.35 -0.92</direction>
      <diffuse>0.96 0.95 0.90 1</diffuse>
      <specular>0.30 0.30 0.28 1</specular>
      <attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation>
      <spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot>
    </light>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{latitude}</latitude_deg>
      <longitude_deg>{longitude}</longitude_deg>
      <elevation>{elevation}</elevation>
    </spherical_coordinates>

    <!-- Plane collision, textured mesh visual; see README section 3b. -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
          <surface><friction><ode/></friction><bounce/><contact/></surface>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh><uri>{ground_mesh}</uri></mesh>
          </geometry>
        </visual>
      </link>
    </model>

    <include>
      <uri>model://{scene_model}</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
"""


def write(path: str, text: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text)
    print(f"wrote {path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="uav_arena_vn")
    parser.add_argument("--site-name", default="HCMUT campus 1, Ho Chi Minh City")
    parser.add_argument("--latitude", type=float, default=HCMUT_LATITUDE_DEG)
    parser.add_argument("--longitude", type=float, default=HCMUT_LONGITUDE_DEG)
    parser.add_argument("--elevation", type=float, default=HCMUT_ELEVATION_M)
    # Ground mesh is 400 m across; wider needs --extent regenerated.
    parser.add_argument("--radius", type=float, default=200.0,
                        help="metres; keep it to the area actually flown")
    parser.add_argument("--features", default=None,
                        help="Overpass JSON file; omit to generate bare ground")
    parser.add_argument("--clear-radius", type=float, default=18.0,
                        help="metres of open ground kept around the spawn point")
    parser.add_argument("--tree-spacing", type=float, default=14.0)
    parser.add_argument("--max-trees", type=int, default=220)
    parser.add_argument("--tree-seed", type=int, default=7)
    parser.add_argument("--ground-mesh",
                        default="model://ground_textured/materials/meshes/ground_rich.obj")
    parser.add_argument("--package-dir",
                        default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    args = parser.parse_args()

    datum = (args.latitude, args.longitude)
    features: Dict[str, list] = {"buildings": [], "surfaces": [], "roads": [], "fences": []}
    trees: List[dict] = []
    cleared: List[dict] = []

    if args.features:
        with open(args.features, encoding="utf-8") as handle:
            features = extract(json.load(handle), datum, args.radius)

        kept = []
        for building in features["buildings"]:
            ring = [tuple(point) for point in building["footprint_enu_m"]]
            if distance_to_polygon((0.0, 0.0), ring) < args.clear_radius:
                cleared.append(building)
            else:
                kept.append(building)
        features["buildings"] = kept

        trees = scatter_trees(features["roads"], args.radius, args.tree_seed,
                              args.tree_spacing, args.max_trees)

        print(f"buildings {len(features['buildings'])}  surfaces {len(features['surfaces'])}  "
              f"ways {len(features['roads'])}  fences {len(features['fences'])}  "
              f"trees {len(trees)} (synthetic)")
        # Never drop geometry silently.
        if cleared:
            print(f"removed {len(cleared)} building(s) overlapping the "
                  f"{args.clear_radius} m takeoff clearing:")
            for building in cleared:
                print(f"  {building['id']} {building['name']} at {building['centre_enu_m']}")
    else:
        print("no feature data supplied; generating bare ground")

    scene_model = f"{args.name}_scene"
    manifest_name = f"{args.name}_ground_truth.json"

    write(os.path.join(args.package_dir, "models", scene_model, "model.sdf"),
          scene_model_sdf(scene_model, features, trees))
    write(os.path.join(args.package_dir, "models", scene_model, "model.config"),
          model_config(scene_model, f"Scene around {args.site_name}, from OpenStreetMap."))

    manifest = {
        "site": args.site_name,
        "datum": {"latitude_deg": args.latitude, "longitude_deg": args.longitude,
                  "elevation_m": args.elevation},
        "radius_m": args.radius,
        "projection": "azimuthal equidistant, earth radius 6371000 m (matches PX4)",
        "frame": "local ENU, origin at the datum",
        "attribution": "Map data (c) OpenStreetMap contributors, ODbL 1.0",
        "buildings": features["buildings"],
        "fences": features["fences"],
        "surfaces": [{"id": s["id"], "kind": s["kind"]} for s in features["surfaces"]],
        "trees": {"synthetic": True,
                  "note": "invented roadside vegetation, not surveyed data",
                  "seed": args.tree_seed, "count": len(trees), "items": trees},
        "removed_for_takeoff_clearing": cleared,
    }
    write(os.path.join(args.package_dir, "docs", manifest_name),
          json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")

    write(os.path.join(args.package_dir, "worlds", f"{args.name}.sdf"),
          WORLD_TEMPLATE.format(
              world_name=args.name, site_name=args.site_name,
              latitude=args.latitude, longitude=args.longitude,
              elevation=args.elevation, radius=args.radius,
              building_count=len(features["buildings"]),
              surface_count=len(features["surfaces"]),
              road_count=len(features["roads"]),
              fence_count=len(features["fences"]),
              tree_count=len(trees),
              manifest_name=manifest_name,
              magnetic_east=HCMC_MAGNETIC_FIELD_ENU[0],
              magnetic_north=HCMC_MAGNETIC_FIELD_ENU[1],
              magnetic_up=HCMC_MAGNETIC_FIELD_ENU[2],
              ground_mesh=args.ground_mesh, scene_model=scene_model))


if __name__ == "__main__":
    main()
