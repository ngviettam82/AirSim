"""S3 - generate BK world from measured buildings; plan §3."""
import argparse
import csv
import json
import math
import os

CONFIDENCE_ORDER = {"high": 3, "medium": 2, "low": 1}

# Label is a class id, range [0,255]; lesson doc §3.
LABEL_BUILDING = 1
LABEL_TREE = 2
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def local_enu(datum_lon, datum_lat, lon, lat):
    """Flat-earth projection about the datum; site is 600 m across."""
    lat_rad = math.radians(datum_lat)
    sin_lat = math.sin(lat_rad)
    denominator = 1.0 - WGS84_E2 * sin_lat * sin_lat
    meridian = WGS84_A * (1.0 - WGS84_E2) / (denominator ** 1.5)
    transverse = WGS84_A / math.sqrt(denominator)
    east = math.radians(lon - datum_lon) * transverse * math.cos(lat_rad)
    north = math.radians(lat - datum_lat) * meridian
    return east, north


def signed_area(ring):
    total = 0.0
    for (x0, y0), (x1, y1) in zip(ring, ring[1:] + ring[:1]):
        total += x0 * y1 - x1 * y0
    return 0.5 * total


def as_counter_clockwise(ring):
    """SDF polyline expects a consistent winding or the extrusion inverts."""
    if len(ring) > 2 and ring[0] == ring[-1]:
        ring = ring[:-1]
    return ring if signed_area(ring) > 0 else ring[::-1]


def building_model(name, ring, height, label, confidence):
    points = "\n".join("          <point>%.3f %.3f</point>" % (x, y) for x, y in ring)
    geometry = """        <geometry>
          <polyline>
%s
            <height>%.3f</height>
          </polyline>
        </geometry>""" % (points, height)
    # Same geometry twice: the invariant is structural, not checked afterwards.
    return """    <model name="%s">
      <static>true</static>
      <link name="link">
        <collision name="collision">
%s
        </collision>
        <visual name="visual">
%s
          <material>
            <ambient>0.62 0.60 0.56 1</ambient>
            <diffuse>0.74 0.72 0.68 1</diffuse>
          </material>
          <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
            <label>%d</label>
          </plugin>
        </visual>
      </link>
    </model>
""" % (name, geometry, geometry, label)


def tree_model(name, east, north, top, radius, label):
    """Canopy sphere + trunk cylinder; collision equals visual (R0)."""
    radius = max(1.2, min(radius, 9.0))
    trunk_height = max(1.0, top - 2.0 * radius)
    centre_z = trunk_height + radius
    trunk = """        <geometry><cylinder><radius>0.30</radius>
          <length>%.3f</length></cylinder></geometry>""" % trunk_height
    canopy = """        <geometry><sphere><radius>%.3f</radius></sphere></geometry>""" % radius
    return """    <model name="%s">
      <static>true</static>
      <pose>%.3f %.3f 0 0 0 0</pose>
      <link name="link">
        <collision name="trunk_collision">
          <pose>0 0 %.3f 0 0 0</pose>
%s
        </collision>
        <visual name="trunk_visual">
          <pose>0 0 %.3f 0 0 0</pose>
%s
          <material><ambient>0.25 0.18 0.11 1</ambient>
            <diffuse>0.35 0.25 0.15 1</diffuse></material>
        </visual>
        <collision name="canopy_collision">
          <pose>0 0 %.3f 0 0 0</pose>
%s
        </collision>
        <visual name="canopy_visual">
          <pose>0 0 %.3f 0 0 0</pose>
%s
          <material><ambient>0.10 0.28 0.10 1</ambient>
            <diffuse>0.16 0.42 0.16 1</diffuse></material>
          <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
            <label>%d</label>
          </plugin>
        </visual>
      </link>
    </model>
""" % (name, east, north, trunk_height / 2.0, trunk, trunk_height / 2.0, trunk,
       centre_z, canopy, centre_z, canopy, label)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--buildings", required=True)
    parser.add_argument("--footprints", required=True)
    parser.add_argument("--out-world", required=True)
    parser.add_argument("--out-manifest", required=True)
    parser.add_argument("--min-confidence", default="low",
                        choices=("high", "medium", "low"))
    parser.add_argument("--world-name", default="uav_arena_bk")
    parser.add_argument("--trees", default=None,
                        help="tree table from tools/extract_trees.py")
    parser.add_argument("--tree-label-base", type=int, default=1000)
    parser.add_argument("--shadows", action="store_true",
                        help="cast shadows from all 499 objects; costs viewer frame "
                             "rate, needed only for perception realism")
    args = parser.parse_args()

    table = {row["id"]: row for row in csv.DictReader(
        open(args.buildings, encoding="utf-8"))}
    with open(args.footprints, encoding="utf-8") as handle:
        collection = json.load(handle)
    print("table rows: %d   footprints: %d"
          % (len(table), len(collection["features"])))

    rings = {}
    for feature in collection["features"]:
        identifier = feature["properties"]["id"]
        rings[identifier] = feature["geometry"]["coordinates"][0]

    # Datum: centroid of footprint vertices, must be the real place.
    everything = [point for ring in rings.values() for point in ring]
    datum_lon = sum(p[0] for p in everything) / len(everything)
    datum_lat = sum(p[1] for p in everything) / len(everything)
    print("datum: %.7f E, %.7f N" % (datum_lon, datum_lat))

    threshold = CONFIDENCE_ORDER[args.min_confidence]
    models, manifest, skipped = [], [], []
    for label, identifier in enumerate(sorted(rings), start=1):
        row = table.get(identifier)
        if row is None:
            continue
        if CONFIDENCE_ORDER.get(row["confidence"], 0) < threshold:
            skipped.append(identifier)
            continue
        height = float(row["world_height_m"])
        if not 2.0 <= height <= 80.0:
            skipped.append(identifier)
            continue
        ring = as_counter_clockwise(
            [local_enu(datum_lon, datum_lat, lon, lat) for lon, lat in rings[identifier]])
        if len(ring) < 3:
            skipped.append(identifier)
            continue
        models.append(building_model(identifier, ring, height, LABEL_BUILDING,
                                     row["confidence"]))
        manifest.append({
            "name": identifier,
            "instance": label,
            "label": LABEL_BUILDING,
            "class": "building",
            "height_m": height,
            "area_m2": float(row["area_m2"]),
            "confidence": row["confidence"],
            "caveat": row.get("caveat", ""),
            "estimator_gap_m": float(row["estimator_gap_m"]),
            "provenance": "measured from own photogrammetry; ground from DSM, "
                          "not from DTM (see plan section 4.1)",
        })

    trees = []
    if args.trees:
        # UTM -> lon/lat -> local_enu; both layers share one frame.
        from rasterio.warp import transform as warp_transform
        with open(args.trees, encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        eastings = [float(r["utm_easting"]) for r in rows]
        northings = [float(r["utm_northing"]) for r in rows]
        lons, lats = warp_transform("EPSG:32648", "EPSG:4326", eastings, northings)
        for index, (row, lon, lat) in enumerate(zip(rows, lons, lats), start=1):
            east, north = local_enu(datum_lon, datum_lat, lon, lat)
            trees.append(tree_model(row["id"], east, north,
                                    float(row["canopy_top_m"]),
                                    float(row["canopy_radius_m"]), LABEL_TREE))
            manifest.append({
                "name": row["id"], "instance": index, "label": LABEL_TREE,
                "class": "tree",
                "height_m": float(row["canopy_top_m"]),
                "canopy_radius_m": float(row["canopy_radius_m"]),
                "confidence": "medium",
                "caveat": "crown as a sphere; collision equals visual",
                "provenance": row.get("provenance", ""),
            })
        print("trees emitted: %d" % len(trees))
    models.extend(trees)

    print("models emitted: %d   skipped: %d" % (len(models), len(skipped)))
    for label, subset in (("building", [m for m in manifest if "class" not in m]),
                          ("tree", [m for m in manifest if m.get("class") == "tree"])):
        heights = sorted(m["height_m"] for m in subset)
        if heights:
            print("  %-9s heights: min %.2f  median %.2f  max %.2f m  (n=%d)"
                  % (label, heights[0], heights[len(heights) // 2], heights[-1],
                     len(heights)))

    world = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="%s">
    <!-- Real coordinates: PX4 derives magnetic field from lat/lon, README 3b. -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>%.7f</latitude_deg>
      <longitude_deg>%.7f</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <physics name="default" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.8</gravity>

    <!-- Full plugin list needed, or PX4 refuses arm (lesson 2.5a). -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system"
            name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-apply-link-wrench-system"
            name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-magnetometer-system"
            name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <!-- Off by default, costed per object (499); lesson doc §2.5b. -->
      <cast_shadows>%s</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <!-- Flat to ~1 m measured (plan 4.1); plane, not heightmap. -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1200 1200</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>1200 1200</size></plane></geometry>
          <material>
            <ambient>0.35 0.35 0.33 1</ambient>
            <diffuse>0.45 0.45 0.42 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

%s  </world>
</sdf>
""" % (args.world_name, datum_lat, datum_lon,
       "true" if args.shadows else "false", "".join(models))

    os.makedirs(os.path.dirname(os.path.abspath(args.out_world)), exist_ok=True)
    with open(args.out_world, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(world)
    with open(args.out_manifest, "w", encoding="utf-8", newline="\n") as handle:
        json.dump({
            "world": args.world_name,
            "datum": {"latitude_deg": datum_lat, "longitude_deg": datum_lon},
            "min_confidence_emitted": args.min_confidence,
            "buildings": manifest,
            "skipped": skipped,
            "invariant": "collision and visual are generated from one polyline, so "
                         "collision is never smaller than visual",
            "height_uncertainty": {
                "value_m": 1.5,
                "measured_by": "gate G1''': an independent reconstruction from the "
                               "even flight strips, measured on the same footprints",
                "bias_m": -1.15,
                "scatter_m": 0.92,
                "scatter_high_confidence_m": 0.69,
                "scatter_medium_confidence_m": 1.45,
                "warning": "An earlier cross-check inside a single DSM reported "
                           "0.16 m. That number only tested sensitivity to the "
                           "ground reference and was blind to error shared by both "
                           "estimators. Do not quote it.",
                "obligation": "Planner inflation must carry this figure. Heights "
                              "here are internally consistent and correctly scaled, "
                              "but have never been checked against the field.",
            },
        }, handle, indent=1)
    print("world   : %s (%.1f kB)" % (args.out_world,
                                      os.path.getsize(args.out_world) / 1024.0))
    print("manifest: %s" % args.out_manifest)


if __name__ == "__main__":
    main()
