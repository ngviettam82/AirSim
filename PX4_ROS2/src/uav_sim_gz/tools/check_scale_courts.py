"""Gate G2' - scale vs. ITF court size, plan §5.2."""
import argparse
import os

import cv2
import numpy as np
import rasterio
from rasterio.windows import from_bounds
from scipy import ndimage

# Region origin is the orthophoto SW corner.
REGION_M = (260.0, 360.0, 445.0, 545.0)

# Only tennis is ITF-standard; other courts fit the site instead.
STANDARDS = (("tennis doubles", 23.77, 10.97),)
INFORMATIVE = (("basketball", 28.00, 15.00), ("volleyball", 18.00, 9.00))


def identify(long_m, short_m, tolerance=0.06):
    """Match against the trusted standard first, then the informative-only ones."""
    for name, long_ref, short_ref in STANDARDS:
        error = max(abs(long_m / long_ref - 1.0), abs(short_m / short_ref - 1.0))
        if error <= tolerance:
            return (name, long_ref, short_ref, True), error
    for name, long_ref, short_ref in INFORMATIVE:
        error = max(abs(long_m / long_ref - 1.0), abs(short_m / short_ref - 1.0))
        if error <= tolerance:
            return (name, long_ref, short_ref, False), error
    return None, None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("project_dir")
    parser.add_argument("--plot", default=None)
    args = parser.parse_args()

    ortho = os.path.join(args.project_dir, "odm_orthophoto", "odm_orthophoto.tif")
    with rasterio.open(ortho) as ds:
        origin = (ds.bounds.left, ds.bounds.bottom)
        lo_x, hi_x, lo_y, hi_y = REGION_M
        window = from_bounds(origin[0] + lo_x, origin[1] + lo_y,
                             origin[0] + hi_x, origin[1] + hi_y, ds.transform)
        rgb = ds.read([1, 2, 3], window=window).astype(np.uint8)
        pixel_m = float(ds.res[0])
    image = np.dstack(rgb)
    px_per_m = 1.0 / pixel_m
    print("crop %d x %d px at %.3f m/px (%.2f px per metre)"
          % (image.shape[1], image.shape[0], pixel_m, px_per_m))

    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    hue, saturation, value = hsv[..., 0], hsv[..., 1], hsv[..., 2]

    # Court surfaces are strongly blue-cyan; OpenCV hue range is 0-179.
    court_surface = (hue > 85) & (hue < 130) & (saturation > 60) & (value > 60)
    court_surface = ndimage.binary_closing(court_surface, np.ones((9, 9)))
    court_surface = ndimage.binary_fill_holes(court_surface)
    labels, count = ndimage.label(court_surface)
    print("candidate court surfaces: %d" % count)

    # Top-hat keeps thin lines, not shaded courts; plan 5.2.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))
    tophat = cv2.morphologyEx(value, cv2.MORPH_TOPHAT, kernel)
    lines = tophat > 18

    results = []
    for index in range(1, count + 1):
        surface = labels == index
        area_m2 = surface.sum() * pixel_m ** 2
        if area_m2 < 250.0 or area_m2 > 1600.0:
            continue
        inner = ndimage.binary_dilation(surface, np.ones((5, 5)))
        painted = ndimage.binary_opening(lines & inner, np.ones((3, 3)))
        if painted.sum() < 400:
            continue

        def rectangle(mask):
            points = np.column_stack(np.nonzero(mask))[:, ::-1].astype(np.float32)
            (cx, cy), (w, h), angle = cv2.minAreaRect(points)
            return (cx, cy), max(w, h) * pixel_m, min(w, h) * pixel_m, angle

        centre, long_m, short_m, angle = rectangle(painted)
        # Eroding one pixel brackets how much the threshold bled.
        eroded = ndimage.binary_erosion(painted, np.ones((3, 3)))
        if eroded.sum() > 200:
            _, long_in, short_in, _ = rectangle(eroded)
        else:
            long_in, short_in = long_m, short_m

        match, error = identify(long_m, short_m)
        results.append({
            "surface_area_m2": area_m2,
            "long_m": long_m,
            "short_m": short_m,
            "long_inner_m": long_in,
            "short_inner_m": short_in,
            "ratio": long_m / short_m if short_m else np.nan,
            "angle": angle,
            "line_pixels": int(painted.sum()),
            "line_fraction_of_surface": float(painted.sum()) / max(surface.sum(), 1),
            "centre_local_m": (lo_x + centre[0] * pixel_m, hi_y - centre[1] * pixel_m),
            "match": match,
            "error": error,
            "mask": painted,
        })

    if not results:
        raise SystemExit("No court measured; adjust the colour thresholds.")

    results.sort(key=lambda r: -r["surface_area_m2"])
    print("\n=== measured court rectangles (top-hat isolated lines) ===")
    print("  %-16s %8s %8s %8s %8s %7s %7s %6s"
          % ("identified as", "long m", "short m", "long in", "short in",
             "ratio", "line %", "trust"))
    trusted, informative = [], []
    for record in results:
        if record["match"] is None:
            print("  %-16s %8.2f %8.2f %8.2f %8.2f %7.3f %7.1f%%"
                  % ("unmatched", record["long_m"], record["short_m"],
                     record["long_inner_m"], record["short_inner_m"], record["ratio"],
                     100 * record["line_fraction_of_surface"]))
            continue
        name, long_ref, short_ref, is_trusted = record["match"]
        long_error = 100.0 * (record["long_m"] / long_ref - 1.0)
        short_error = 100.0 * (record["short_m"] / short_ref - 1.0)
        print("  %-16s %8.2f %8.2f %8.2f %8.2f %7.3f %7.1f%% %6s"
              % (name, record["long_m"], record["short_m"], record["long_inner_m"],
                 record["short_inner_m"], record["ratio"],
                 100 * record["line_fraction_of_surface"],
                 "yes" if is_trusted else "no"))
        print("      vs %.2f x %.2f -> long %+.2f%%  short %+.2f%%"
              % (long_ref, short_ref, long_error, short_error))
        (trusted if is_trusted else informative).append((long_error, short_error))

    print("\n=== G2' verdict ===")
    if not trusted:
        print("  no tennis court measured - gate INCONCLUSIVE")
        return
    errors = np.array([e for pair in trusted for e in pair])
    print("  tennis courts measured    : %d  (%d dimensions)"
          % (len(trusted), len(errors)))
    print("  per-dimension error (%%)   : %s" % np.array2string(errors, precision=2))
    print("  mean                      : %+.3f %%" % errors.mean())
    print("  worst single dimension    : %+.3f %%" % errors[np.argmax(np.abs(errors))])
    if informative:
        other = np.array([e for pair in informative for e in pair])
        print("  (non-tennis courts, not used: %s %%)"
              % np.array2string(other, precision=2))
        print("   outdoor basketball/volleyball are built to fit the site, so a")
        print("   deviation there is not evidence about our scale.")

    passed = abs(errors.mean()) <= 0.5 and np.abs(errors).max() <= 1.5
    print("\n  G2' %s (mean <= 0.5%%, no single dimension beyond 1.5%%)"
          % ("PASS" if passed else "FAIL"))
    if passed:
        print("  Block scale is metric: no common factor is corrupting the heights.")
        print("  On a 20 m building this bound is +/- %.2f m." % (0.20 * abs(errors.mean())))
    else:
        print("  Every height carries this factor; correct it before building the world.")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(19, 9.6))
        ax[0].imshow(image)
        ax[0].set_title("courts, native 5 cm")
        overlay = np.zeros(image.shape[:2])
        for i, record in enumerate(results, start=1):
            overlay[record["mask"]] = i
        ax[1].imshow(image)
        ax[1].imshow(np.where(overlay > 0, overlay, np.nan), cmap="autumn", alpha=0.9)
        for record in results:
            if record["match"] is None:
                continue
            cx = (record["centre_local_m"][0] - REGION_M[0]) / pixel_m
            cy = (REGION_M[3] - record["centre_local_m"][1]) / pixel_m
            ax[1].annotate("%s%s\n%.2f x %.2f m"
                           % (record["match"][0],
                              "" if record["match"][3] else " (not used)",
                              record["long_m"], record["short_m"]),
                           (cx, cy), color="white", fontsize=10, fontweight="bold",
                           ha="center",
                           bbox=dict(boxstyle="round,pad=0.25", fc="#111", alpha=0.75))
        ax[1].set_title("painted lines used for the measurement")
        for a in ax:
            a.set_xticks([]); a.set_yticks([])
        plt.tight_layout()
        plt.savefig(args.plot, dpi=110)
        print("\nplot: %s" % args.plot)


if __name__ == "__main__":
    main()
