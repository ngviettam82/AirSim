"""Gates G5 (collision>=visual) and G6 (one label) on the SDF."""
import json
import re
import sys
import xml.etree.ElementTree as ElementTree


def geometry_of(element):
    """Comparable geometry+pose; unknown shapes raise, do not pass silently."""
    pose = (element.findtext("pose", "0 0 0 0 0 0") or "").split()
    pose_key = tuple(round(float(v), 4) for v in pose[:3]) if len(pose) >= 3 else ()
    geometry = element.find("geometry")
    if geometry is None:
        raise ValueError("no geometry in %s" % element.get("name"))
    polyline = geometry.find("polyline")
    if polyline is not None:
        points = tuple(sorted(p.text.strip() for p in polyline.findall("point")))
        return ("polyline", points,
                round(float(polyline.findtext("height", "0")), 4), pose_key)
    sphere = geometry.find("sphere")
    if sphere is not None:
        return ("sphere", round(float(sphere.findtext("radius", "0")), 4), pose_key)
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        return ("cylinder", round(float(cylinder.findtext("radius", "0")), 4),
                round(float(cylinder.findtext("length", "0")), 4), pose_key)
    plane = geometry.find("plane")
    if plane is not None:
        return ("plane", (plane.findtext("size", "") or "").strip(), pose_key)
    box = geometry.find("box")
    if box is not None:
        return ("box", (box.findtext("size", "") or "").strip(), pose_key)
    raise ValueError("unhandled geometry under %s" % element.get("name"))


def main():
    world_path, manifest_path = sys.argv[1], sys.argv[2]
    tree = ElementTree.parse(world_path)
    world = tree.getroot().find("world")
    manifest = json.load(open(manifest_path, encoding="utf-8"))

    models = world.findall("model")
    print("models in world: %d" % len(models))

    print("\n=== G5: collision must not be smaller than visual ===")
    mismatches, checked, no_collision = [], 0, []
    for model in models:
        name = model.get("name")
        for link in model.findall("link"):
            visuals = [geometry_of(v) for v in link.findall("visual")]
            collisions = [geometry_of(c) for c in link.findall("collision")]
            if not collisions:
                no_collision.append(name)
                continue
            for visual in visuals:
                checked += 1
                if visual not in collisions:
                    mismatches.append((name, visual, collisions))
    print("  visual/collision pairs checked : %d" % checked)
    print("  links with a visual but no collision: %d %s"
          % (len(no_collision), no_collision[:5]))
    print("  geometry mismatches            : %d" % len(mismatches))
    for name, visual, collisions in mismatches[:5]:
        print("      %s: visual %s not among collisions" % (name, str(visual)[:70]))
    g5 = not mismatches and not no_collision
    print("  G5 %s" % ("PASS - every visual has an identical collision"
                       if g5 else "FAIL"))

    print("\n=== G6: one unique label per building ===")
    labels = []
    for model in models:
        for plugin in model.iter("plugin"):
            if "Label" in (plugin.get("name") or ""):
                labels.append((model.get("name"), int(plugin.findtext("label"))))
    labelled = {name for name, _ in labels}
    objects = [m.get("name") for m in models
               if m.get("name", "").startswith(("bk_", "tree_"))]
    values = [value for _, value in labels]
    print("  labelled objects  : %d (buildings %d, trees %d)"
          % (len(objects), sum(1 for n in objects if n.startswith("bk_")),
             sum(1 for n in objects if n.startswith("tree_"))))
    print("  label plugins     : %d" % len(labels))
    print("  distinct labels   : %d" % len(set(values)))
    print("  manifest entries  : %d" % len(manifest["buildings"]))
    unlabelled = sorted(set(objects) - labelled)
    if unlabelled:
        print("  objects with NO label: %d %s" % (len(unlabelled), unlabelled[:5]))
    # Gazebo rejects labels outside [0,255]; invisible to XML-only checks.
    out_of_range = sorted({v for v in values if not 0 <= v <= 255})
    print("  labels outside [0,255]: %d %s" % (len(out_of_range), out_of_range[:5]))
    by_class = {}
    for entry in manifest["buildings"]:
        by_class.setdefault(entry.get("class", "?"), set()).add(entry["label"])
    for name, ids in sorted(by_class.items()):
        print("  class %-9s -> label %s" % (name, sorted(ids)))
    instances = [entry.get("instance") for entry in manifest["buildings"]]
    print("  per-class instance ids unique: %s"
          % (len(instances) == len(objects)))
    manifest_labels = {entry["label"] for entry in manifest["buildings"]}
    g6 = (len(labels) == len(objects) and not out_of_range
          and set(values) == manifest_labels
          and all(len(ids) == 1 for ids in by_class.values()))
    print("  G6 %s" % ("PASS - counts agree and labels are unique and match the "
                       "manifest" if g6 else "FAIL"))

    print("\n=== provenance recorded with every height ===")
    by_confidence = {}
    for entry in manifest["buildings"]:
        by_confidence.setdefault(entry["confidence"], []).append(entry)
    for level in ("high", "medium", "low"):
        rows = by_confidence.get(level, [])
        if not rows:
            continue
        gaps = sorted(e["estimator_gap_m"] for e in rows if "estimator_gap_m" in e)
        detail = ("gap p50 %.2f m" % gaps[len(gaps) // 2]) if gaps else "trees"
        print("  %-6s : %3d objects, %s" % (level, len(rows), detail))
    caveated = [e for e in manifest["buildings"] if e["caveat"]]
    print("  with a recorded caveat: %d" % len(caveated))
    for entry in caveated[:6]:
        print("      %-8s %s" % (entry["name"], entry["caveat"]))

    return 0 if (g5 and g6) else 1


if __name__ == "__main__":
    sys.exit(main())
