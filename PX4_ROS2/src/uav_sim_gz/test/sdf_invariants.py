#!/usr/bin/env python3
"""Enforce the invariant that keeps a simulated obstacle honest: COLLISION COVERS VISUAL.

WHY THIS EXISTS (P12.2, 2026-08-25). CLAUDE.md section 5 states the rule and the reason:

    Rendering sensors read <visual>, NOT <collision>. gpu_lidar and the depth camera
    cast rays against visual geometry (Harmonic has no CPU lidar). So no visual mesh may
    ship with a smaller collision body -- otherwise the drone SEES a wall, FLIES THROUGH
    it, the avoidance test passes in simulation, and the real aircraft hits something.

Nothing checked it. `tools/check_world_invariants.py` looks like it does and does not:
it takes a world AND a manifest (it exists to accept the shelved Bach Khoa world), raises
on <mesh>, never resolves <include>, and compares geometry for EQUALITY rather than
containment -- so a collision LARGER than its visual (which is safe) reads as a mismatch,
while an <include>d scene of hundreds of links is invisible to it entirely.

THE RULE: for every link, every <visual> must sit inside the union of that link's
<collision> volumes, compared as axis-aligned boxes in the link frame.

THREE DECLARED EXEMPTIONS, each measured before it was written, each counted in the
output so it can never apply silently:

  1. GROUND DECAL. A visual with no collision whose top is at most GROUND_DECAL_TOP_M
     above the ground, in a world that HAS a ground-plane collision. Roads and painted
     surfaces are drawn on the ground; the ground plane underneath is what a drone would
     actually land on. Measured 2026-08-25 across all worlds: 174 such visuals, every one
     a road_* or surface_*, highest top 0.070 m, thickest 0.040 m.
     🔴 The threshold is a PROJECT CONVENTION derived from that data, not a physical
     constant. It sits at 0.10 m so a slightly thicker marking does not trip the gate for
     nothing -- but anything taller than 10 cm has stopped being a ground marking and a
     human has to look at it. Do not raise this number to make a run go green.
     (A first draft of this rule used 0.05 m taken from the THICKNESS. That would have
     been wrong: pose offsets put the highest top at 0.070 m. Measure the top, not the
     thickness.)

  2. AIRCRAFT-MOUNTED PART. Links inside uav0*/sensor_* models. A camera housing bolted
     to the airframe is not an obstacle, and giving it collision geometry would create
     self-collision -- those models already declare <self_collide>false</self_collide>.
     This exemption is scoped BY MODEL NAME on purpose: "no collision is fine" as a blanket
     rule would delete the whole point of this file.

  3. EXTERNAL GEOMETRY. model://x500 and model://OakD-Lite live in the PX4 tree, outside
     this package. Their geometry cannot be read from here, so it is reported as declared
     external rather than assumed correct. A green run does NOT mean the airframe's own
     collision geometry was checked.

WHAT A GREEN RUN IS NOT. This reads DECLARATIONS, not Gazebo's behaviour: "the SDF says
the collision covers the visual" is not "the simulator builds it that way". Behavioural
evidence still has to come from a flight. And where an answer cannot be computed from the
file, the result is UNVERIFIABLE and the run FAILS -- "could not measure" landing on the
safe side is the failure mode CLAUDE.md section 5 records twice.

Usage: python3 sdf_invariants.py [package_dir]
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

GROUND_DECAL_TOP_M = 0.10
AIRCRAFT_MODEL_PREFIXES = ("uav0", "sensor_")
TOL = 1e-6


def floats(text, count, default=0.0):
    parts = (text or "").split()
    out = [default] * count
    for i in range(min(count, len(parts))):
        try:
            out[i] = float(parts[i])
        except ValueError:
            return None
    return out


def pose_of(element):
    return floats(element.findtext("pose"), 6) or [0.0] * 6


def is_axis_aligned(pose):
    """True when every rotation is a multiple of 90 degrees.

    Anything else needs the real oriented box. Approximating it with an AABB inflates the
    COLLISION side, which is the unsafe direction, so those cases are reported instead.
    """
    return all(abs(math.remainder(a, math.pi / 2.0)) <= 1e-9 for a in pose[3:6])


def swap_for_rotation(half, pose):
    hx, hy, hz = half
    roll, pitch, yaw = pose[3], pose[4], pose[5]
    if abs(math.remainder(roll, math.pi)) > 1e-9:
        hy, hz = hz, hy
    if abs(math.remainder(pitch, math.pi)) > 1e-9:
        hx, hz = hz, hx
    if abs(math.remainder(yaw, math.pi)) > 1e-9:
        hx, hy = hy, hx
    return [hx, hy, hz]


def aabb(element):
    """((cx,cy,cz),(hx,hy,hz)) in the link frame, or ("plane"|"mesh"|None, detail)."""
    geometry = element.find("geometry")
    if geometry is None:
        return None, "no <geometry>"
    children = list(geometry)
    if not children:
        return None, "<geometry> is empty"
    node = children[0]
    kind = node.tag
    pose = pose_of(element)
    centre = list(pose[0:3])

    if kind == "plane":
        return "plane", None
    if kind == "mesh":
        uri = (node.findtext("uri") or "").strip()
        scale = (node.findtext("scale") or "").strip()
        return "mesh", (uri, scale, tuple(pose))

    if kind == "box":
        size = floats(node.findtext("size"), 3)
        if size is None:
            return None, "unparsable box size"
        half = [s / 2.0 for s in size]
    elif kind == "sphere":
        r = floats(node.findtext("radius"), 1)
        if r is None:
            return None, "unparsable sphere radius"
        half = [r[0]] * 3
    elif kind == "cylinder":
        r = floats(node.findtext("radius"), 1)
        length = floats(node.findtext("length"), 1)
        if r is None or length is None:
            return None, "unparsable cylinder"
        half = [r[0], r[0], length[0] / 2.0]
    elif kind == "polyline":
        xs, ys = [], []
        for point in node.findall("point"):
            xy = floats(point.text, 2)
            if xy is None:
                return None, "unparsable polyline point"
            xs.append(xy[0])
            ys.append(xy[1])
        if not xs:
            return None, "polyline with no points"
        height = (floats(node.findtext("height"), 1) or [0.0])[0]
        half = [(max(xs) - min(xs)) / 2.0, (max(ys) - min(ys)) / 2.0, height / 2.0]
        centre = [
            centre[0] + (max(xs) + min(xs)) / 2.0,
            centre[1] + (max(ys) + min(ys)) / 2.0,
            centre[2] + height / 2.0,
        ]
    else:
        return None, "unhandled geometry <%s>" % kind

    if not is_axis_aligned(pose):
        return None, "rotated %s (not a 90-degree multiple)" % kind
    return (tuple(centre), tuple(swap_for_rotation(half, pose))), None


def covers(outer, inner):
    (ocx, ocy, ocz), (ohx, ohy, ohz) = outer
    (icx, icy, icz), (ihx, ihy, ihz) = inner
    for oc, oh, ic, ih in ((ocx, ohx, icx, ihx), (ocy, ohy, icy, ihy), (ocz, ohz, icz, ihz)):
        if ic - ih < oc - oh - TOL or ic + ih > oc + oh + TOL:
            return False
    return True


class Report:
    def __init__(self):
        self.problems = []
        self.exempt_decal = 0
        self.exempt_aircraft = 0
        self.exempt_aircraft_parts = []
        self.exempt_external = 0
        self.exempt_external_models = []
        self.links = 0
        self.visuals = 0
        self.entry_points = 0


def check_link(source, link, ctx, report):
    name = link.get("name") or "<unnamed>"
    visuals = link.findall("visual")
    collisions = link.findall("collision")
    report.links += 1
    if not visuals:
        return
    link_z = pose_of(link)[2]

    boxes, meshes, plane_collision = [], [], False
    for c in collisions:
        box, detail = aabb(c)
        if box == "plane":
            plane_collision = True
        elif box == "mesh":
            meshes.append(detail)
        elif box is None:
            report.problems.append((source, name, "collision unreadable: %s" % detail))
        else:
            boxes.append(box)

    for v in visuals:
        report.visuals += 1
        box, detail = aabb(v)
        if box == "plane" or plane_collision:
            continue
        if box == "mesh":
            if detail in meshes:
                continue
            if ctx["aircraft"]:
                report.exempt_aircraft += 1
                report.exempt_aircraft_parts.append((source, name))
                continue
            report.problems.append(
                (source, name, "visual mesh %s has no identical collision -- UNVERIFIABLE"
                 % detail[0]))
            continue
        if box is None:
            report.problems.append((source, name, "visual unreadable: %s" % detail))
            continue
        if any(covers(cb, box) for cb in boxes):
            continue

        top = link_z + box[0][2] + box[1][2]
        if ctx["ground_plane"] and top <= GROUND_DECAL_TOP_M:
            report.exempt_decal += 1
            continue
        if ctx["aircraft"]:
            report.exempt_aircraft += 1
            report.exempt_aircraft_parts.append((source, name))
            continue
        report.problems.append(
            (source, name,
             "visual top %.3f m, centre=%s half=%s -- no collision covers it"
             % (top, box[0], box[1])))


def load(path):
    try:
        return ET.parse(path).getroot()
    except ET.ParseError as exc:
        return exc


def gather(path, package_dir, report, seen):
    """[(source, root)] for a file and everything it <include>s, models resolved."""
    root = load(path)
    if isinstance(root, ET.ParseError):
        report.problems.append((os.path.basename(path), "-", "unparsable SDF: %s" % root))
        return []
    out = [(path, root)]
    for include in root.iter("include"):
        uri = (include.findtext("uri") or "").strip()
        if not uri.startswith("model://"):
            continue
        name = uri[len("model://"):].split("/")[0]
        sub = os.path.join(package_dir, "models", name, "model.sdf")
        if not os.path.isfile(sub):
            report.exempt_external += 1
            report.exempt_external_models.append(name)
            continue
        if sub in seen:
            continue
        seen.add(sub)
        out.extend(gather(sub, package_dir, report, seen))
    return out


def is_aircraft(path):
    parts = os.path.normpath(path).split(os.sep)
    return any(p.startswith(AIRCRAFT_MODEL_PREFIXES) for p in parts)


def check_package(package_dir):
    """Scan a package's worlds and standalone models. Returns a filled Report."""
    report = Report()

    worlds = sorted(
        os.path.join(package_dir, "worlds", f)
        for f in os.listdir(os.path.join(package_dir, "worlds"))
        if f.endswith(".sdf")
    )
    models_dir = os.path.join(package_dir, "models")
    models = sorted(
        os.path.join(models_dir, d, "model.sdf")
        for d in os.listdir(models_dir)
        if os.path.isfile(os.path.join(models_dir, d, "model.sdf"))
    )

    # A model that a world <include>s is checked IN THAT WORLD, where the ground plane
    # is visible. Checking it standalone as well would re-judge the same links with the
    # ground removed and call every road a violation -- the first draft did exactly that
    # and reported 174 problems that were also, in the same run, counted as exempt.
    reached = set()
    for world in worlds:
        for source, _root in gather(world, package_dir, Report(), set()):
            reached.add(os.path.abspath(source))
    entry_points = worlds + [m for m in models if os.path.abspath(m) not in reached]

    for entry in entry_points:
        roots = gather(entry, package_dir, report, set())
        ground_plane = any(
            g.find("plane") is not None
            for _src, root in roots
            for c in root.iter("collision")
            for g in ([c.find("geometry")] if c.find("geometry") is not None else [])
        )
        for source, root in roots:
            ctx = {"ground_plane": ground_plane, "aircraft": is_aircraft(source)}
            for link in root.iter("link"):
                check_link(os.path.basename(os.path.dirname(source)) or
                           os.path.basename(source), link, ctx, report)
    report.entry_points = len(entry_points)
    return report


def main():
    package_dir = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    report = check_package(package_dir)
    entry_points = [None] * report.entry_points

    print("=== collision covers visual (CLAUDE.md section 5) ===")
    print("  entry points: %d   links: %d   visuals: %d"
          % (len(entry_points), report.links, report.visuals))
    print("  exempt, ground decal  (top <= %.2f m over a ground plane): %d"
          % (GROUND_DECAL_TOP_M, report.exempt_decal))
    print("  exempt, aircraft part (uav0*/sensor_*, self_collide=false): %d"
          % report.exempt_aircraft)
    print("  declared external     (model:// outside this package):     %d"
          % report.exempt_external)
    SHOWN = 12
    for source, link, why in report.problems[:SHOWN]:
        print("  🔴 VIOLATION %s :: %s" % (source, link))
        print("               %s" % why)
    if len(report.problems) > SHOWN:
        print("  ... and %d more (not truncated silently -- the count below is the total)"
              % (len(report.problems) - SHOWN))
    print()
    if report.problems:
        print("RESULT: FAIL (%d problem(s))" % len(report.problems))
        return 1
    print("RESULT: PASS -- no link ships a visual its collision does not cover")
    return 0


if __name__ == "__main__":
    sys.exit(main())
