"""S2.1+S2.2 - roof height = p50(roof) - p10(ring); plan §4.1."""
import argparse
import csv
import json
import os
import sys

import numpy as np
import rasterio
from matplotlib.path import Path as MplPath
from rasterio import features
from rasterio.warp import Resampling, reproject, transform as warp_transform
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from propose_campus_boundary import rdp

GROUND_WINDOW_M = 60.0
COARSE_M = 1.0


def load_boundary(path, crs):
    with open(path, encoding="utf-8") as handle:
        payload = json.load(handle)
    ring = payload["features"][0]["geometry"]["coordinates"][0]
    lon = [p[0] for p in ring]
    lat = [p[1] for p in ring]
    easting, northing = warp_transform("EPSG:4326", crs, lon, lat)
    return MplPath(np.column_stack([easting, northing]))


def ortho_on_grid(ortho_path, reference):
    """Resample orthophoto onto the DEM grid so masks line up."""
    with rasterio.open(ortho_path) as src:
        bands = min(3, src.count)
        out = np.zeros((bands, reference["height"], reference["width"]), dtype=np.float32)
        for i in range(bands):
            reproject(
                source=rasterio.band(src, i + 1),
                destination=out[i],
                src_transform=src.transform, src_crs=src.crs,
                dst_transform=reference["transform"], dst_crs=reference["crs"],
                resampling=Resampling.average,
            )
    return out


def height_above_visible_ground(dsm, valid, pixel_m):
    """Height above the lowest surface nearby; no DTM involved."""
    factor = max(1, int(round(COARSE_M / pixel_m)))
    filled = np.where(valid, dsm, np.nanmax(dsm[valid]))
    coarse = filled[::factor, ::factor]
    window = max(3, int(round(GROUND_WINDOW_M / (pixel_m * factor))))
    ground = ndimage.minimum_filter(coarse, size=window)
    ground = np.repeat(np.repeat(ground, factor, axis=0), factor, axis=1)
    ground = ground[:dsm.shape[0], :dsm.shape[1]]
    return np.where(valid, filled - ground, np.nan), ground


def absorb_rooftop_structures(labels, count, pixel_m, min_area_m2, reach_m):
    """Fold small rooftop islands into their building; lesson doc 2.5d."""
    if count == 0:
        return labels, 0
    areas = ndimage.sum(np.ones_like(labels, dtype=np.float32), labels,
                        range(1, count + 1)) * pixel_m ** 2
    reach = max(1, int(round(reach_m / pixel_m)))
    absorbed = 0
    for index in np.argsort(areas) + 1:
        area = areas[index - 1]
        if area >= min_area_m2 or area <= 0:
            continue
        mask = labels == index
        if not mask.any():
            continue
        neighbourhood = ndimage.binary_dilation(mask, np.ones((reach, reach))) & ~mask
        touching = np.unique(labels[neighbourhood])
        touching = [t for t in touching if t > 0 and t != index
                    and areas[t - 1] >= 3.0 * area]
        if not touching:
            continue
        parent = max(touching, key=lambda t: areas[t - 1])
        labels[mask] = parent
        areas[parent - 1] += area
        areas[index - 1] = 0.0
        absorbed += 1
    return labels, absorbed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("project_dir")
    parser.add_argument("--boundary", required=True)
    parser.add_argument("--min-height", type=float, default=3.0)
    parser.add_argument("--min-area", type=float, default=80.0)
    parser.add_argument("--greenness", type=float, default=0.06,
                        help="excess-green above which a pixel is called vegetation")
    parser.add_argument("--roughness", type=float, default=0.45,
                        help="metres; DSM std over 1.5 m above which a surface is canopy")
    parser.add_argument("--reach-m", type=float, default=2.0,
                        help="metres; how far a rooftop structure may be from the "
                             "building that absorbs it")
    parser.add_argument("--bridge-m", type=float, default=1.5,
                        help="metres; closes tree-shadow gaps and reunites rooftop "
                             "structures with their building")
    parser.add_argument("--out-csv", default=None)
    parser.add_argument("--out-geojson", default=None)
    parser.add_argument("--plot", default=None)
    args = parser.parse_args()

    dem = os.path.join(args.project_dir, "odm_dem")
    with rasterio.open(os.path.join(dem, "dsm.tif")) as ds:
        dsm = ds.read(1, masked=True)
        reference = {"transform": ds.transform, "crs": ds.crs,
                     "width": ds.width, "height": ds.height}
        pixel_m = float(ds.res[0])
    valid = ~dsm.mask
    dsm_a = np.asarray(dsm.filled(np.nan))
    print("DSM %d x %d at %.2f m, %s" % (reference["width"], reference["height"],
                                         pixel_m, reference["crs"]))

    height, _ = height_above_visible_ground(dsm_a, valid, pixel_m)

    rgb = ortho_on_grid(os.path.join(args.project_dir, "odm_orthophoto",
                                     "odm_orthophoto.tif"), reference)
    total = np.clip(rgb.sum(axis=0), 1.0, None)
    excess_green = (2.0 * rgb[1] - rgb[0] - rgb[2]) / total
    green = excess_green > args.greenness

    smooth = max(3, int(round(1.5 / pixel_m)))
    mean = ndimage.uniform_filter(np.where(valid, dsm_a, 0.0), size=smooth)
    mean_sq = ndimage.uniform_filter(np.where(valid, dsm_a ** 2, 0.0), size=smooth)
    roughness = np.sqrt(np.clip(mean_sq - mean ** 2, 0.0, None))
    flat = roughness < args.roughness

    # Greenness alone, not "green AND rough"; lesson doc 2.5c.
    vegetation = green

    print("\n=== roof discriminators ===")
    print("  green (excess green > %.2f)       : %.1f%% of valid"
          % (args.greenness, 100.0 * green[valid].mean()))
    print("  locally flat (std < %.2f m)      : %.1f%% of valid"
          % (args.roughness, 100.0 * flat[valid].mean()))
    print("  vegetation (green AND rough)     : %.1f%% of valid"
          % (100.0 * vegetation[valid].mean()))
    print("  green but flat -> painted roof   : %.1f%% of valid (recovered by pass 2)"
          % (100.0 * (green & flat)[valid].mean()))
    tall = valid & np.isfinite(height) & (height > args.min_height)
    print("  above %.1f m                      : %.1f%% of valid"
          % (args.min_height, 100.0 * tall[valid].mean()))

    roof = tall & flat & ~vegetation
    # Closing reunites a roof split by shadow or a structure.
    bridge = max(3, int(round(args.bridge_m / pixel_m)))
    disc = np.zeros((bridge, bridge), dtype=bool)
    radius = bridge / 2.0
    yy, xx = np.mgrid[:bridge, :bridge]
    disc[(yy - radius + 0.5) ** 2 + (xx - radius + 0.5) ** 2 <= radius ** 2] = True
    before = ndimage.label(ndimage.binary_opening(roof, np.ones((5, 5))))[1]
    roof = ndimage.binary_closing(roof, disc)
    roof = ndimage.binary_opening(roof, np.ones((5, 5)))
    roof = ndimage.binary_fill_holes(roof)
    after = ndimage.label(roof)[1]
    print("  roof candidates                  : %.1f%% of valid"
          % (100.0 * roof[valid].mean()))
    print("  regions before/after %.1f m bridging: %d -> %d"
          % (args.bridge_m, before, after))

    labels, count = ndimage.label(roof)
    print("\nconnected roof regions: %d" % count)
    labels, absorbed = absorb_rooftop_structures(labels, count, pixel_m,
                                                 args.min_area, args.reach_m)
    print("rooftop structures absorbed into their building: %d" % absorbed)
    boundary = load_boundary(args.boundary, reference["crs"])

    ground_ok = valid & np.isfinite(height) & (height < 1.5)
    rows = []
    polygons = []
    objects = ndimage.find_objects(labels)
    for index, window in enumerate(objects, start=1):
        area = float((labels[window] == index).sum()) * pixel_m ** 2
        if area < args.min_area:
            continue
        pad = int(round(22.0 / pixel_m))
        rows_slice = slice(max(0, window[0].start - pad),
                           min(labels.shape[0], window[0].stop + pad))
        cols_slice = slice(max(0, window[1].start - pad),
                           min(labels.shape[1], window[1].stop + pad))
        patch = labels[rows_slice, cols_slice] == index
        distance = ndimage.distance_transform_edt(~patch) * pixel_m
        ring = (distance > 5.0) & (distance <= 20.0) & ground_ok[rows_slice, cols_slice]
        if ring.sum() < 40:
            ring = (distance > 3.0) & (distance <= 30.0) & ground_ok[rows_slice, cols_slice]
        dsm_patch = dsm_a[rows_slice, cols_slice]
        roof_pixels = dsm_patch[patch & np.isfinite(dsm_patch)]
        if roof_pixels.size < 20 or ring.sum() < 20:
            continue
        ring_pixels = dsm_patch[ring & np.isfinite(dsm_patch)]

        # p25=eave, p95=ridge; second estimator in lesson doc 2.5e.
        height_patch = height[rows_slice, cols_slice][patch]
        height_patch = height_patch[np.isfinite(height_patch)]
        minfilter_height = (float(np.percentile(height_patch, 50))
                            if height_patch.size else float("nan"))

        roof_low = float(np.percentile(roof_pixels, 25))
        roof_elev = float(np.percentile(roof_pixels, 50))
        roof_top = float(np.percentile(roof_pixels, 95))
        ground_elev = float(np.percentile(ring_pixels, 10))
        eave = roof_low - ground_elev
        ridge = roof_top - ground_elev

        # Fraction of the ring that is real ground, not roof.
        annulus = (distance > 5.0) & (distance <= 20.0)
        ground_fraction = (float(ring.sum()) / max(annulus.sum(), 1)
                           if annulus.sum() else 0.0)
        roof_relief = roof_top - roof_low
        mid_height = roof_elev - ground_elev
        disagreement = abs(mid_height - minfilter_height)
        # Estimator agreement outranks ring ground coverage for confidence.
        if disagreement <= 0.5 and roof_relief <= 3.0:
            confidence = "high"
        elif disagreement <= 1.5 and roof_relief <= 6.0:
            confidence = "medium"
        else:
            confidence = "low"
        reason = []
        if disagreement > 1.5:
            reason.append("estimators disagree by %.1f m" % disagreement)
        if ground_fraction < 0.20:
            reason.append("little visible ground nearby")
        if roof_relief > 3.0:
            reason.append("multi-level roof, should be split")

        centre_rows, centre_cols = np.nonzero(patch)
        row_mid = rows_slice.start + centre_rows.mean()
        col_mid = cols_slice.start + centre_cols.mean()
        easting, northing = rasterio.transform.xy(reference["transform"],
                                                  [row_mid], [col_mid])
        if not boundary.contains_point((easting[0], northing[0])):
            continue

        rows.append({
            "id": "bk_%03d" % (len(rows) + 1),
            "area_m2": round(area, 1),
            "eave_height_m": round(eave, 2),
            "ridge_height_m": round(ridge, 2),
            "mid_height_m": round(roof_elev - ground_elev, 2),
            "roof_relief_m": round(roof_top - roof_low, 2),
            "roof_elev_m": round(roof_elev, 2),
            "ground_elev_m": round(ground_elev, 2),
            "roof_std_m": round(float(roof_pixels.std()), 2),
            "world_height_m": round(mid_height, 2),
            "height_minfilter_m": round(minfilter_height, 2),
            "estimator_gap_m": round(disagreement, 2),
            "confidence": confidence,
            "caveat": "; ".join(reason),
            "ground_visible_fraction": round(ground_fraction, 3),
            "ground_area_m2": round(float(ring.sum()) * pixel_m ** 2, 1),
            "veg_fraction_in_roof": round(float(
                vegetation[rows_slice, cols_slice][patch].mean()), 3),
            "utm_easting": round(float(easting[0]), 2),
            "utm_northing": round(float(northing[0]), 2),
        })
        polygons.append((rows[-1]["id"], patch, rows_slice, cols_slice))

    print("buildings inside the campus boundary, above %.0f m2: %d"
          % (args.min_area, len(rows)))
    if not rows:
        raise SystemExit("Nothing measured; loosen the thresholds.")

    heights = np.array([r["eave_height_m"] for r in rows])
    areas = np.array([r["area_m2"] for r in rows])
    print("\n=== measured eave heights ===")
    for q in (5, 25, 50, 75, 95):
        print("  p%-3d %6.2f m" % (q, np.percentile(heights, q)))
    print("  min %.2f  max %.2f m" % (heights.min(), heights.max()))
    print("  total roof area %.2f ha; largest building %.0f m2"
          % (areas.sum() / 1e4, areas.max()))
    print("\n=== confidence (the honest part of the table) ===")
    for level in ("high", "medium", "low"):
        subset = [r for r in rows if r["confidence"] == level]
        if not subset:
            continue
        heights = np.array([r["world_height_m"] for r in subset])
        area = sum(r["area_m2"] for r in subset)
        print("  %-6s : %2d buildings, %6.0f m2 roof, height p50 %5.2f m"
              % (level, len(subset), area, np.median(heights)))
    for level in ("medium", "low"):
        for record in [r for r in rows if r["confidence"] == level and r["caveat"]]:
            print("      %-8s %-6s %5.0f m2  h=%5.2f  ground %.0f%%  %s"
                  % (record["id"], level, record["area_m2"], record["world_height_m"],
                     100 * record["ground_visible_fraction"], record["caveat"]))

    if args.out_csv:
        with open(args.out_csv, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print("\ncsv: %s" % args.out_csv)

    if args.out_geojson:
        collection = {"type": "FeatureCollection", "features": []}
        for record, (name, patch, rows_slice, cols_slice) in zip(rows, polygons):
            full = np.zeros(labels.shape, dtype=np.uint8)
            full[rows_slice, cols_slice] = patch
            for geom, value in features.shapes(full, mask=full.astype(bool),
                                               transform=reference["transform"]):
                if value != 1:
                    continue
                outer = np.asarray(geom["coordinates"][0], dtype=float)
                simple = rdp(outer, 0.6)
                lon, lat = warp_transform(reference["crs"], "EPSG:4326",
                                          simple[:, 0].tolist(), simple[:, 1].tolist())
                ring = [[round(a, 8), round(b, 8)] for a, b in zip(lon, lat)]
                if ring[0] != ring[-1]:
                    ring.append(ring[0])
                collection["features"].append({
                    "type": "Feature",
                    "properties": record,
                    "geometry": {"type": "Polygon", "coordinates": [ring]},
                })
                break
        with open(args.out_geojson, "w", encoding="utf-8") as handle:
            json.dump(collection, handle, indent=1)
        print("geojson: %s (%d footprints)" % (args.out_geojson,
                                               len(collection["features"])))

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        step = max(1, min(labels.shape) // 1100)
        picture = np.clip(np.dstack([rgb[0], rgb[1], rgb[2]]) / 255.0, 0, 1)[::step, ::step]
        painted = np.full(labels.shape, np.nan)
        for record, (name, patch, rows_slice, cols_slice) in zip(rows, polygons):
            block = painted[rows_slice, cols_slice]
            block[patch] = record["eave_height_m"]
            painted[rows_slice, cols_slice] = block
        fig, ax = plt.subplots(1, 3, figsize=(22, 8.6))
        ax[0].imshow(picture)
        ax[0].set_title("our orthophoto")
        ax[1].imshow(picture)
        im = ax[1].imshow(painted[::step, ::step], cmap="turbo", vmin=0, vmax=35, alpha=0.85)
        ax[1].set_title("%d buildings, coloured by measured eave height" % len(rows))
        plt.colorbar(im, ax=ax[1], fraction=0.046, label="m")
        ax[2].hist(heights, bins=30, color="#3b6ea5")
        ax[2].set_title("eave height distribution (m)\nmedian %.1f m" % np.median(heights))
        ax[2].set_xlabel("m"); ax[2].grid(alpha=0.3)
        for a in ax[:2]:
            a.set_xticks([]); a.set_yticks([])
        plt.tight_layout()
        plt.savefig(args.plot, dpi=110)
        print("plot: %s" % args.plot)


if __name__ == "__main__":
    main()
