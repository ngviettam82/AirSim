"""Map where the ground filter absorbed buildings; plan §4.1."""
import argparse
import os

import numpy as np
import rasterio
from scipy import ndimage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("project_dir")
    parser.add_argument("--window", type=float, default=20.0,
                        help="metres; local relief window for judging DTM smoothness")
    parser.add_argument("--flat-tolerance", type=float, default=1.0,
                        help="metres of local relief still considered credible terrain")
    parser.add_argument("--plot", default=None)
    args = parser.parse_args()

    dem = os.path.join(args.project_dir, "odm_dem")
    with rasterio.open(os.path.join(dem, "dsm.tif")) as ds:
        dsm = ds.read(1, masked=True)
        pixel_m = float(ds.res[0])
    with rasterio.open(os.path.join(dem, "dtm.tif")) as ds:
        dtm = ds.read(1, masked=True)

    valid = (~dsm.mask) & (~dtm.mask)
    dtm_a = np.asarray(dtm.filled(np.nan))
    dsm_a = np.asarray(dsm.filled(np.nan))
    ndsm = np.where(valid, dsm_a - dtm_a, np.nan)

    size = max(3, int(round(args.window / pixel_m)))
    print("=== DTM local relief, %.0f m window (%d px) ===" % (args.window, size))
    work = np.where(valid, dtm_a, np.nan)
    filled = np.where(valid, dtm_a, np.nanmedian(dtm_a))
    hi = ndimage.maximum_filter(filled, size=size)
    lo = ndimage.minimum_filter(filled, size=size)
    relief = np.where(valid, hi - lo, np.nan)

    finite = np.isfinite(relief)
    for q in (50, 75, 90, 95, 99):
        print("  p%-3d %6.2f m" % (q, np.nanpercentile(relief, q)))
    bad = finite & (relief > args.flat_tolerance)
    print("  pixels above %.1f m local relief: %.1f%%  <- candidate contamination"
          % (args.flat_tolerance, 100.0 * bad.sum() / finite.sum()))

    print("\n=== does the DTM track the DSM? (it must not, over flat ground) ===")
    ok = np.isfinite(dtm_a) & np.isfinite(dsm_a) & valid
    corr = float(np.corrcoef(dtm_a[ok], dsm_a[ok])[0, 1])
    print("  corr(DTM, DSM) over all valid pixels: %.3f" % corr)
    clean = ok & (relief <= args.flat_tolerance)
    if clean.sum() > 1000:
        print("  corr on the smooth subset only        : %.3f"
              % float(np.corrcoef(dtm_a[clean], dsm_a[clean])[0, 1]))

    print("\n=== terrain relief, trustworthy area only ===")
    for label, mask in (("all valid", ok), ("smooth subset", clean)):
        if mask.sum() < 1000:
            continue
        vals = dtm_a[mask]
        p1, p99 = np.percentile(vals, [1, 99])
        print("  %-14s area %6.2f ha   relief(p1-p99) %5.2f m   std %.2f m"
              % (label, mask.sum() * pixel_m ** 2 / 1e4, p99 - p1, vals.std()))

    print("\n=== consequence for building heights ===")
    tall = np.isfinite(ndsm) & (ndsm > 3.0)
    print("  area measured as structure >3 m : %.2f ha (%.1f%% of valid)"
          % (tall.sum() * pixel_m ** 2 / 1e4, 100.0 * tall.sum() / ok.sum()))
    tall_clean = tall & (relief <= args.flat_tolerance)
    tall_bad = tall & (relief > args.flat_tolerance)
    for label, mask in (("in smooth-DTM area", tall_clean),
                        ("in contaminated area", tall_bad)):
        if mask.sum():
            print("  median measured height %-22s %.2f m  (%.2f ha)"
                  % (label, float(np.nanmedian(ndsm[mask])),
                     mask.sum() * pixel_m ** 2 / 1e4))

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        step = max(1, min(dtm_a.shape) // 850)
        rows, cols = dtm_a[::step, ::step].shape
        ortho_path = os.path.join(args.project_dir, "odm_orthophoto",
                                  "odm_orthophoto.tif")
        fig, ax = plt.subplots(1, 4, figsize=(24, 6.6))

        if os.path.isfile(ortho_path):
            with rasterio.open(ortho_path) as ds:
                rgb = ds.read([1, 2, 3], out_shape=(3, rows, cols))
                alpha = (ds.read(4, out_shape=(rows, cols))
                         if ds.count >= 4 else None)
            picture = np.dstack(rgb).astype(np.uint8)
            if alpha is not None:
                picture[alpha == 0] = 255
            ax[0].imshow(picture)
            ax[0].set_title("our own orthophoto (5 cm, resampled)", fontsize=10)
        else:
            ax[0].imshow(dtm_a[::step, ::step], cmap="terrain")
            ax[0].set_title("DTM", fontsize=10)
        ax[0].set_xticks([]); ax[0].set_yticks([])

        panels = (
            (ax[1], dtm_a[::step, ::step], "DTM (claims: ground only)",
             dict(cmap="terrain")),
            (ax[2], relief[::step, ::step],
             "DTM local relief in %.0f m\n>%.1f m = not credible terrain"
             % (args.window, args.flat_tolerance),
             dict(cmap="inferno", vmin=0, vmax=6)),
            (ax[3], ndsm[::step, ::step], "nDSM (measured height)",
             dict(cmap="magma", vmin=0, vmax=30)),
        )
        for a, img, title, kw in panels:
            im = a.imshow(img, **kw)
            a.set_title(title, fontsize=10)
            a.set_xticks([]); a.set_yticks([])
            plt.colorbar(im, ax=a, fraction=0.046)
        plt.tight_layout()
        plt.savefig(args.plot, dpi=105)
        print("\nplot: %s" % args.plot)


if __name__ == "__main__":
    main()
