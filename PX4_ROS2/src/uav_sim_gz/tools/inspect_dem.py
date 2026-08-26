"""Gate the DTM before trusting height; lesson doc 2.5f."""
import argparse
import os

import numpy as np
import rasterio
from scipy import ndimage


def load(path):
    with rasterio.open(path) as ds:
        band = ds.read(1, masked=True)
        return band, ds.transform, ds.crs, ds.res, ds.bounds


def describe(name, path):
    with rasterio.open(path) as ds:
        print("  %-6s %5d x %-5d px  res %.3f m  CRS %s  nodata %s"
              % (name, ds.width, ds.height, ds.res[0], ds.crs, ds.nodatavals[0]))
        return ds.width, ds.height


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dem_dir")
    parser.add_argument("--plot", default=None)
    parser.add_argument("--min-height", type=float, default=3.0,
                        help="nDSM threshold that counts as a structure")
    args = parser.parse_args()

    dsm_path = os.path.join(args.dem_dir, "dsm.tif")
    dtm_path = os.path.join(args.dem_dir, "dtm.tif")
    print("=== rasters ===")
    describe("DSM", dsm_path)
    describe("DTM", dtm_path)

    dsm, transform, crs, res, bounds = load(dsm_path)
    dtm, _, _, _, _ = load(dtm_path)
    if dsm.shape != dtm.shape:
        raise SystemExit("DSM and DTM grids differ: %s vs %s" % (dsm.shape, dtm.shape))

    pixel_m = float(res[0])
    valid = (~dsm.mask) & (~dtm.mask)
    print("  valid pixels: %.1f%%  (%.2f ha at %.2f m/px)"
          % (100.0 * valid.mean(), valid.sum() * pixel_m ** 2 / 1e4, pixel_m))

    print("\n=== terrain (DTM) - decides heightmap vs flat plane ===")
    ground = np.asarray(dtm[valid])
    lo, hi = np.percentile(ground, [1, 99])
    print("  elevation p1 %.2f m   p99 %.2f m   relief(p99-p1) %.2f m"
          % (lo, hi, hi - lo))
    print("  full min %.2f  max %.2f  std %.2f m"
          % (ground.min(), ground.max(), ground.std()))
    if hi - lo < 1.0:
        print("  -> relief below 1.0 m: a FLAT PLANE is justified, skip the heightmap")
    else:
        print("  -> relief above 1.0 m: the world needs real terrain")

    ndsm = np.where(valid, np.asarray(dsm.filled(0)) - np.asarray(dtm.filled(0)), np.nan)
    finite = np.isfinite(ndsm)
    print("\n=== nDSM = DSM - DTM (the building-height measurement) ===")
    for q in (1, 25, 50, 75, 95, 99, 99.9):
        print("  p%-5s %7.2f m" % (q, np.nanpercentile(ndsm, q)))
    print("  max %.2f m   negative pixels %.2f%% (should be near zero)"
          % (np.nanmax(ndsm), 100.0 * np.mean(ndsm[finite] < -0.5)))

    print("\n=== ROOF-CENTRE COLLAPSE TEST (is the ground filter window wide enough?) ===")
    structure = finite & (ndsm > args.min_height)
    filled = ndimage.binary_fill_holes(structure)
    labels, count = ndimage.label(filled)
    print("  candidate structures above %.1f m: %d" % (args.min_height, count))
    sizes = ndimage.sum(np.ones_like(labels), labels, range(1, count + 1))
    areas = sizes * pixel_m ** 2
    big = np.argsort(areas)[::-1][:12] + 1
    print("  %-6s %9s %8s %8s %8s   %s" %
          ("id", "area m2", "edge h", "core h", "core/edge", "verdict"))
    ratios = []
    for lab in big:
        mask = labels == lab
        # Distance from outline: core is deep, edge is the rim.
        dist = ndimage.distance_transform_edt(mask) * pixel_m
        rim = mask & (dist > 0) & (dist <= 2.0)
        core = mask & (dist >= max(4.0, 0.35 * dist.max()))
        if rim.sum() < 20 or core.sum() < 20:
            continue
        edge_h = float(np.nanmedian(ndsm[rim]))
        core_h = float(np.nanmedian(ndsm[core]))
        ratio = core_h / edge_h if edge_h > 0.1 else np.nan
        ratios.append(ratio)
        verdict = "ok" if ratio > 0.85 else ("SUSPECT" if ratio > 0.5 else "COLLAPSED")
        print("  %-6d %9.0f %8.2f %8.2f %8.2f   %s"
              % (lab, areas[lab - 1], edge_h, core_h, ratio, verdict))
    if ratios:
        good = np.mean(np.array(ratios) > 0.85)
        print("\n  core/edge ratio: median %.2f over %d largest structures"
              % (np.nanmedian(ratios), len(ratios)))
        print("  fraction passing (>0.85): %.0f%%" % (100 * good))
        print("  GATE %s" % ("PASS - roofs stayed out of the terrain"
                             if good >= 0.8 else
                             "FAIL - widen --smrf-window and rebuild the DEM"))

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        step = max(1, min(dsm.shape) // 900)
        d = np.asarray(dsm.filled(np.nan))[::step, ::step]
        t = np.asarray(dtm.filled(np.nan))[::step, ::step]
        n = ndsm[::step, ::step]
        fig, ax = plt.subplots(1, 3, figsize=(19, 6.4))
        for a, img, title, kw in (
                (ax[0], d, "DSM (ground + objects)", dict(cmap="terrain")),
                (ax[1], t, "DTM (ground only)", dict(cmap="terrain")),
                (ax[2], n, "nDSM = height above ground", dict(cmap="magma", vmin=0, vmax=30))):
            im = a.imshow(img, **kw)
            a.set_title(title)
            a.set_xticks([]); a.set_yticks([])
            plt.colorbar(im, ax=a, fraction=0.046, label="m")
        plt.tight_layout()
        plt.savefig(args.plot, dpi=110)
        print("\nplot: %s" % args.plot)


if __name__ == "__main__":
    main()
