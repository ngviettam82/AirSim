"""Hand-drawn campus boundary; automatic delineation failed on heavy tree cover."""
import argparse
import json
import os

import numpy as np
import rasterio
from rasterio.warp import transform as warp_transform

# Read off the orthophoto by eye; accurate to +/-20 m.
CAMPUS_LOCAL_M = [
    (250, 120),
    (355, 195),
    (440, 300),
    (465, 415),
    (430, 550),
    (360, 645),
    (270, 575),
    (185, 475),
    (140, 365),
    (155, 240),
]

LANDMARKS = {
    "sports field": (275, 280),
    "tennis/basketball": (305, 495),
    "red hip roof": (305, 385),
}


def polygon_area(points):
    x = np.asarray([p[0] for p in points], dtype=float)
    y = np.asarray([p[1] for p in points], dtype=float)
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("project_dir")
    parser.add_argument("--plot", default=None)
    parser.add_argument("--geojson", default=None)
    args = parser.parse_args()

    ortho = os.path.join(args.project_dir, "odm_orthophoto", "odm_orthophoto.tif")
    with rasterio.open(ortho) as ds:
        height = 1400
        width = int(height * ds.width / ds.height)
        rgb = ds.read([1, 2, 3], out_shape=(3, height, width))
        alpha = ds.read(4, out_shape=(height, width)) if ds.count >= 4 else None
        bounds = ds.bounds
        crs = ds.crs

    picture = np.dstack(rgb).astype(np.uint8)
    if alpha is not None:
        picture[alpha == 0] = 255

    origin = (bounds.left, bounds.bottom)
    span_x = bounds.right - bounds.left
    span_y = bounds.top - bounds.bottom
    area_ha = polygon_area(CAMPUS_LOCAL_M) / 1e4
    print("orthophoto extent: %.0f x %.0f m   CRS %s" % (span_x, span_y, crs))
    print("proposed boundary: %d vertices, area %.2f ha"
          % (len(CAMPUS_LOCAL_M), area_ha))

    easting = [origin[0] + p[0] for p in CAMPUS_LOCAL_M]
    northing = [origin[1] + p[1] for p in CAMPUS_LOCAL_M]
    lon, lat = warp_transform(crs, "EPSG:4326", easting, northing)
    print("\n  #   local (m)      UTM 48N easting/northing        lon / lat")
    for i, (p, e, n, lo, la) in enumerate(
            zip(CAMPUS_LOCAL_M, easting, northing, lon, lat), start=1):
        print("  %-3d (%4d,%4d)  %10.1f %11.1f   %.6f %.6f"
              % (i, p[0], p[1], e, n, lo, la))

    if args.geojson:
        ring = [[float(a), float(b)] for a, b in zip(lon, lat)]
        ring.append(ring[0])
        payload = {
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "properties": {
                    "name": "bk_campus_draft",
                    "area_ha": round(area_ha, 2),
                    "status": "draft read off our orthophoto, +/-20 m, awaiting review",
                },
                "geometry": {"type": "Polygon", "coordinates": [ring]},
            }],
        }
        with open(args.geojson, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=1)
        print("\ngeojson: %s" % args.geojson)

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(14, 17))
        ax.imshow(picture, extent=[0, span_x, 0, span_y])
        ring = CAMPUS_LOCAL_M + [CAMPUS_LOCAL_M[0]]
        ax.plot([p[0] for p in ring], [p[1] for p in ring],
                "-", color="#00e5ff", lw=3.0)
        ax.fill([p[0] for p in ring], [p[1] for p in ring],
                color="#00e5ff", alpha=0.11)
        for i, p in enumerate(CAMPUS_LOCAL_M, start=1):
            ax.plot(p[0], p[1], "o", color="#ff2d95", ms=9, zorder=5)
            ax.annotate(str(i), p, textcoords="offset points", xytext=(9, 6),
                        color="white", fontsize=13, fontweight="bold",
                        bbox=dict(boxstyle="round,pad=0.18", fc="#ff2d95", ec="none"))
        for name, p in LANDMARKS.items():
            ax.plot(p[0], p[1], "x", color="yellow", ms=12, mew=3, zorder=5)
            ax.annotate(name, p, textcoords="offset points", xytext=(10, -14),
                        color="yellow", fontsize=10, fontweight="bold")
        ax.set_xticks(np.arange(0, span_x, 50))
        ax.set_yticks(np.arange(0, span_y, 50))
        ax.grid(color="yellow", lw=0.4, alpha=0.4)
        plt.setp(ax.get_xticklabels(), rotation=90, fontsize=7)
        plt.setp(ax.get_yticklabels(), fontsize=7)
        ax.set_xlabel("metres east of UTM %.0f" % origin[0])
        ax.set_ylabel("metres north of UTM %.0f" % origin[1])
        ax.set_title("Proposed campus boundary — DRAFT, %.1f ha, vertices movable by number"
                     % area_ha)
        plt.tight_layout()
        plt.savefig(args.plot, dpi=105)
        print("plot: %s" % args.plot)


if __name__ == "__main__":
    main()
