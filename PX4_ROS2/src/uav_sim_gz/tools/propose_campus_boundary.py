"""Propose campus boundary from open-ground fraction; superseded by draw_campus_boundary.py."""
import argparse
import json
import os

import numpy as np
import rasterio
from rasterio.warp import transform as warp_transform
from scipy import ndimage


def largest_region(mask):
    labels, count = ndimage.label(mask)
    if count == 0:
        raise SystemExit("No candidate region found; loosen the thresholds.")
    sizes = ndimage.sum(np.ones_like(labels), labels, range(1, count + 1))
    return labels == (int(np.argmax(sizes)) + 1)


def rdp(points, epsilon):
    """Ramer-Douglas-Peucker: keep the shape, drop the redundant vertices."""
    if len(points) < 3:
        return points
    start, end = points[0], points[-1]
    line = end - start
    length = float(np.hypot(line[0], line[1]))
    offset = points - start
    if length < 1e-9:
        distances = np.hypot(offset[:, 0], offset[:, 1])
    else:
        # 2-D cross product written out; np.cross no longer accepts 2-vectors.
        distances = np.abs(line[0] * offset[:, 1] - line[1] * offset[:, 0]) / length
    index = int(np.argmax(distances))
    if distances[index] > epsilon:
        left = rdp(points[:index + 1], epsilon)
        right = rdp(points[index:], epsilon)
        return np.vstack([left[:-1], right])
    return np.vstack([start, end])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("project_dir")
    parser.add_argument("--open-window", type=float, default=60.0,
                        help="metres; window for measuring open-ground fraction")
    parser.add_argument("--open-fraction", type=float, default=0.08,
                        help="minimum tall-structure density to count as campus")
    parser.add_argument("--tall-height", type=float, default=12.0,
                        help="metres; a faculty block is this tall, a house is not")
    parser.add_argument("--simplify", type=float, default=8.0,
                        help="metres; polygon simplification tolerance")
    parser.add_argument("--plot", default=None)
    parser.add_argument("--geojson", default=None)
    args = parser.parse_args()

    dem = os.path.join(args.project_dir, "odm_dem")
    with rasterio.open(os.path.join(dem, "dsm.tif")) as ds:
        dsm = ds.read(1, masked=True)
        transform_affine = ds.transform
        crs = ds.crs
        pixel_m = float(ds.res[0])
    with rasterio.open(os.path.join(dem, "dtm.tif")) as ds:
        dtm = ds.read(1, masked=True)

    valid = (~dsm.mask) & (~dtm.mask)
    ndsm = np.where(valid, np.asarray(dsm.filled(0)) - np.asarray(dtm.filled(0)), np.nan)

    # Tall-structure density, not open-ground fraction: canopy defeats the latter.
    coarse_px = max(1, int(round(1.0 / pixel_m)))
    dsm_coarse = np.asarray(dsm.filled(np.nan))[::coarse_px, ::coarse_px]
    valid_coarse = valid[::coarse_px, ::coarse_px]
    coarse_m = pixel_m * coarse_px
    filled = np.where(valid_coarse, dsm_coarse, np.nanmax(dsm_coarse))
    ground_window = max(3, int(round(args.open_window / coarse_m)))
    local_ground = ndimage.minimum_filter(filled, size=ground_window)
    height = np.where(valid_coarse, filled - local_ground, np.nan)

    print("=== height above local visible ground (%.0f m window, DTM not used) ==="
          % args.open_window)
    for q in (50, 75, 90, 95, 99):
        print("  p%-3d %6.2f m" % (q, np.nanpercentile(height, q)))

    tall = valid_coarse & np.isfinite(height) & (height >= args.tall_height)
    print("  pixels above %.0f m: %.1f%% of valid"
          % (args.tall_height, 100.0 * tall.sum() / valid_coarse.sum()))

    density_window = max(3, int(round(args.open_window / coarse_m)))
    fraction = ndimage.uniform_filter(tall.astype(np.float32), size=density_window)
    print("\n=== tall-structure density (%.0f m window) ===" % args.open_window)
    for q in (50, 75, 90, 95):
        print("  p%-3d %.3f" % (q, np.percentile(fraction[valid_coarse], q)))

    candidate = valid_coarse & (fraction >= args.open_fraction)
    print("candidate campus pixels: %.1f%% of valid"
          % (100.0 * candidate.sum() / valid_coarse.sum()))
    pixel_m = coarse_m
    valid = valid_coarse
    transform_affine = transform_affine * rasterio.Affine.scale(coarse_px, coarse_px)
    ndsm = height

    # Close gaps buildings punch in the mask, keep one region.
    closing = max(3, int(round(40.0 / pixel_m)))
    solid = ndimage.binary_closing(candidate, np.ones((closing, closing)))
    solid = ndimage.binary_fill_holes(solid)
    solid = largest_region(solid)
    opening = max(3, int(round(25.0 / pixel_m)))
    solid = ndimage.binary_opening(solid, np.ones((opening, opening)))
    solid = ndimage.binary_fill_holes(largest_region(solid))
    area_ha = solid.sum() * pixel_m ** 2 / 1e4
    print("proposed campus area: %.2f ha" % area_ha)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    contours = plt.contour(solid.astype(float), levels=[0.5])
    longest = max(contours.allsegs[0], key=len)
    plt.close("all")
    # Pre-decimate so the recursive simplification cannot blow the stack.
    coarse = np.asarray(longest, dtype=float)[::max(1, len(longest) // 2000)]
    simplified = rdp(coarse, args.simplify / pixel_m)
    print("boundary vertices: %d -> %d after simplification" % (len(longest), len(simplified)))

    cols, rows = simplified[:, 0], simplified[:, 1]
    easting, northing = rasterio.transform.xy(transform_affine, rows, cols)
    easting = np.asarray(easting)
    northing = np.asarray(northing)
    lon, lat = warp_transform(crs, "EPSG:4326", easting.tolist(), northing.tolist())

    if args.geojson:
        ring = [[float(a), float(b)] for a, b in zip(lon, lat)]
        if ring[0] != ring[-1]:
            ring.append(ring[0])
        payload = {
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "properties": {"name": "bk_campus_proposed",
                               "area_ha": round(area_ha, 2),
                               "derived_from": "open-ground fraction >= %.2f in %.0f m"
                                               % (args.open_fraction, args.open_window)},
                "geometry": {"type": "Polygon", "coordinates": [ring]},
            }],
        }
        with open(args.geojson, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=1)
        print("geojson: %s" % args.geojson)
        print("  lon %.6f..%.6f   lat %.6f..%.6f"
              % (min(lon), max(lon), min(lat), max(lat)))

    if args.plot:
        ortho_path = os.path.join(args.project_dir, "odm_orthophoto", "odm_orthophoto.tif")
        step = max(1, min(dsm.shape) // 1100)
        rows_out, cols_out = ndsm[::step, ::step].shape
        with rasterio.open(ortho_path) as ds:
            rgb = ds.read([1, 2, 3], out_shape=(3, rows_out, cols_out))
            alpha = ds.read(4, out_shape=(rows_out, cols_out)) if ds.count >= 4 else None
        picture = np.dstack(rgb).astype(np.uint8)
        if alpha is not None:
            picture[alpha == 0] = 255

        fig, ax = plt.subplots(1, 3, figsize=(21, 8.4))
        ax[0].imshow(picture)
        ax[0].set_title("our orthophoto + proposed boundary (%.1f ha)" % area_ha)
        ax[1].imshow(fraction[::step, ::step], cmap="viridis", vmin=0, vmax=0.8)
        ax[1].set_title("open-ground fraction in %.0f m\n(campus is open, alleys are not)"
                        % args.open_window)
        ax[2].imshow(ndsm[::step, ::step], cmap="magma", vmin=0, vmax=30)
        ax[2].set_title("nDSM")
        for a in ax:
            a.plot(simplified[:, 0] / step, simplified[:, 1] / step,
                   "-", color="#00e5ff", lw=2.4)
            a.set_xticks([]); a.set_yticks([])
        plt.tight_layout()
        plt.savefig(args.plot, dpi=110)
        print("plot: %s" % args.plot)


if __name__ == "__main__":
    main()
