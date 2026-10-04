"""Layer L3 - measure trees, the low-altitude obstacle; plan S2.3."""
import argparse
import csv
import json
import os

import numpy as np
import rasterio
from matplotlib.path import Path as MplPath
from rasterio.warp import Resampling, reproject, transform as warp_transform
from scipy import ndimage

GROUND_WINDOW_M = 60.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("project_dir")
    parser.add_argument("--boundary", required=True)
    parser.add_argument("--min-height", type=float, default=4.0)
    parser.add_argument("--separation-m", type=float, default=5.0,
                        help="minimum distance between two detected crowns")
    parser.add_argument("--out-csv", required=True)
    parser.add_argument("--plot", default=None)
    args = parser.parse_args()

    dem = os.path.join(args.project_dir, "odm_dem")
    with rasterio.open(os.path.join(dem, "dsm.tif")) as ds:
        band = ds.read(1, masked=True)
        pixel_m = float(ds.res[0])
        reference = {"transform": ds.transform, "crs": ds.crs,
                     "width": ds.width, "height": ds.height}
    valid = ~band.mask
    elevation = np.asarray(band.filled(np.nan))

    with rasterio.open(os.path.join(args.project_dir, "odm_orthophoto",
                                    "odm_orthophoto.tif")) as src:
        rgb = np.zeros((3, reference["height"], reference["width"]), dtype=np.float32)
        for i in range(3):
            reproject(source=rasterio.band(src, i + 1), destination=rgb[i],
                      src_transform=src.transform, src_crs=src.crs,
                      dst_transform=reference["transform"], dst_crs=reference["crs"],
                      resampling=Resampling.average)
    total = np.clip(rgb.sum(axis=0), 1.0, None)
    green = (2.0 * rgb[1] - rgb[0] - rgb[2]) / total > 0.06

    factor = max(1, int(round(1.0 / pixel_m)))
    filled = np.where(valid, elevation, np.nanmax(elevation[valid]))
    coarse = filled[::factor, ::factor]
    window = max(3, int(round(GROUND_WINDOW_M / (pixel_m * factor))))
    ground = ndimage.minimum_filter(coarse, size=window)
    ground = np.repeat(np.repeat(ground, factor, 0), factor, 1)
    ground = ground[:elevation.shape[0], :elevation.shape[1]]
    above = np.where(valid, filled - ground, np.nan)

    canopy = valid & green & np.isfinite(above) & (above > args.min_height)
    canopy = ndimage.binary_opening(canopy, np.ones((5, 5)))
    canopy = ndimage.binary_closing(canopy, np.ones((5, 5)))
    print("canopy above %.1f m: %.2f ha (%.1f%% of valid)"
          % (args.min_height, canopy.sum() * pixel_m ** 2 / 1e4,
             100.0 * canopy.sum() / valid.sum()))

    # Distance-transform local maxima find crown centres better than height peaks.
    radius_map = ndimage.distance_transform_edt(canopy) * pixel_m
    separation = max(3, int(round(args.separation_m / pixel_m)))
    peaks = (radius_map == ndimage.maximum_filter(radius_map, size=separation)) & \
            (radius_map > 1.2)
    rows_idx, cols_idx = np.nonzero(peaks)
    print("crown candidates: %d" % len(rows_idx))

    with open(args.boundary, encoding="utf-8") as handle:
        ring = json.load(handle)["features"][0]["geometry"]["coordinates"][0]
    east, north = warp_transform("EPSG:4326", reference["crs"],
                                 [p[0] for p in ring], [p[1] for p in ring])
    boundary = MplPath(np.column_stack([east, north]))

    smooth_height = ndimage.maximum_filter(np.where(canopy, above, np.nan),
                                           size=max(3, int(round(1.0 / pixel_m))))
    records = []
    for row, col in zip(rows_idx, cols_idx):
        radius = float(radius_map[row, col])
        if not 1.2 <= radius <= 9.0:
            continue
        easting, northing = rasterio.transform.xy(reference["transform"], [row], [col])
        if not boundary.contains_point((easting[0], northing[0])):
            continue
        patch = above[max(0, row - separation):row + separation,
                      max(0, col - separation):col + separation]
        patch = patch[np.isfinite(patch)]
        if patch.size < 20:
            continue
        top = float(np.percentile(patch, 95))
        if top < args.min_height:
            continue
        records.append({
            "id": "tree_%03d" % (len(records) + 1),
            "utm_easting": round(float(easting[0]), 2),
            "utm_northing": round(float(northing[0]), 2),
            "canopy_top_m": round(top, 2),
            "canopy_radius_m": round(radius, 2),
            "provenance": "green in own ortho, above local DSM minimum; radius from "
                          "canopy distance transform",
        })

    print("trees kept inside the campus boundary: %d" % len(records))
    if records:
        tops = np.array([r["canopy_top_m"] for r in records])
        radii = np.array([r["canopy_radius_m"] for r in records])
        print("  canopy top   : p5 %.1f  p50 %.1f  p95 %.1f  max %.1f m"
              % (*np.percentile(tops, [5, 50, 95]), tops.max()))
        print("  crown radius : p5 %.1f  p50 %.1f  p95 %.1f  max %.1f m"
              % (*np.percentile(radii, [5, 50, 95]), radii.max()))
        with open(args.out_csv, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(records[0].keys()))
            writer.writeheader()
            writer.writerows(records)
        print("csv: %s" % args.out_csv)

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        step = max(1, min(canopy.shape) // 1000)
        picture = np.clip(np.dstack([rgb[0], rgb[1], rgb[2]]) / 255.0, 0, 1)
        fig, ax = plt.subplots(1, 2, figsize=(17, 9))
        ax[0].imshow(picture[::step, ::step])
        ax[0].imshow(np.where(canopy, 1.0, np.nan)[::step, ::step],
                     cmap="summer", alpha=0.55)
        ax[0].set_title("canopy mask")
        ax[1].imshow(picture[::step, ::step])
        ax[1].scatter([r["utm_easting"] for r in records],
                      [r["utm_northing"] for r in records], s=1)
        ax[1].cla()
        ax[1].imshow(picture[::step, ::step])
        cols = [(r["utm_easting"] - reference["transform"].c)
                / reference["transform"].a / step for r in records]
        rws = [(r["utm_northing"] - reference["transform"].f)
               / reference["transform"].e / step for r in records]
        sizes = [(r["canopy_radius_m"] / pixel_m / step) ** 2 * 0.35 for r in records]
        ax[1].scatter(cols, rws, s=sizes, facecolors="none", edgecolors="#00e5ff", lw=0.8)
        ax[1].set_title("%d crowns, circle = measured radius" % len(records))
        for a in ax:
            a.set_xticks([]); a.set_yticks([])
        plt.tight_layout(); plt.savefig(args.plot, dpi=110)
        print("plot: %s" % args.plot)


if __name__ == "__main__":
    main()
