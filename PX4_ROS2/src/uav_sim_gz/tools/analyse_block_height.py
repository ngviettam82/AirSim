"""Decompose reconstructed-camera-height error; see lesson doc section 2.5g."""
import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_block_doming import camera_centre, extract_shots

TURN_TOLERANCE_DEG = 45.0


def load_exif(csv_path):
    rows = {}
    with open(csv_path, encoding="utf-8", errors="replace") as handle:
        for row in csv.DictReader(handle):
            name = row.get("FileName", "")
            if not name.upper().endswith(".JPG"):
                continue
            try:
                clock = row["DateTimeOriginal"].split(" ")[1].split(":")
                rows[name] = {
                    "baro": float(row["RelativeAltitude"]),
                    "gps_alt": float(row["GPSAltitude"]),
                    "yaw": float(row["FlightYawDegree"]),
                    "t": int(clock[0]) * 3600 + int(clock[1]) * 60 + int(clock[2]),
                }
            except (KeyError, TypeError, ValueError, IndexError):
                continue
    return rows


def angular_gap(a, b):
    return np.abs((a - b + 180.0) % 360.0 - 180.0)


def classify_legs(yaw):
    """Label each image by flight direction (legs ~180 deg apart)."""
    principal = float(np.median(yaw[angular_gap(yaw, np.median(yaw)) < 90.0]))
    to_principal = angular_gap(yaw, principal)
    leg = np.full(len(yaw), -1, dtype=int)
    leg[to_principal < TURN_TOLERANCE_DEG] = 0
    leg[angular_gap(yaw, principal + 180.0) < TURN_TOLERANCE_DEG] = 1
    return leg, principal


def segment_strips(times, leg, sortie):
    """A strip: maximal run of one leg within a sortie."""
    order = np.argsort(times)
    strip = np.full(len(times), -1, dtype=int)
    current = -1
    prev_leg, prev_sortie = None, None
    for i in order:
        if leg[i] < 0:
            continue  # mid-turn, belongs to no strip
        if leg[i] != prev_leg or sortie[i] != prev_sortie:
            current += 1
        strip[i] = current
        prev_leg, prev_sortie = leg[i], sortie[i]
    return strip


def segment_sorties(times):
    order = np.argsort(times)
    sortie = np.zeros(len(times), dtype=int)
    count = 0
    for i in range(1, len(order)):
        if times[order[i]] - times[order[i - 1]] > 60:
            count += 1
        sortie[order[i]] = count
    return sortie


def remove(design, values):
    coef, *_ = np.linalg.lstsq(design, values, rcond=None)
    return values - design @ coef, coef


def group_means(values, labels):
    return np.array([values[labels == k].mean() for k in np.unique(labels[labels >= 0])])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("opensfm_dir")
    parser.add_argument("exif_csv")
    parser.add_argument("--plot", default=None)
    args = parser.parse_args()

    shots = extract_shots(os.path.join(args.opensfm_dir, "reconstruction.topocentric.json"))
    exif = load_exif(args.exif_csv)
    names = sorted(n for n in shots if n in exif)
    centres = np.array([camera_centre(shots[n]) for n in names])
    xs = centres[:, 0] - centres[:, 0].mean()
    ys = centres[:, 1] - centres[:, 1].mean()
    zs = centres[:, 2]
    baro = np.array([exif[n]["baro"] for n in names])
    gps_alt = np.array([exif[n]["gps_alt"] for n in names])
    yaw = np.array([exif[n]["yaw"] for n in names])
    times = np.array([exif[n]["t"] for n in names], dtype=float)

    sortie = segment_sorties(times)
    leg, principal = classify_legs(yaw)
    strip = segment_strips(times, leg, sortie)
    on_strip = strip >= 0
    print("matched shots      : %d" % len(names))
    print("principal heading  : %.0f deg" % principal)
    print("sorties            : %d" % (sortie.max() + 1))
    print("strips             : %d  (%d images on strips, %d mid-turn)"
          % (strip.max() + 1, on_strip.sum(), (~on_strip).sum()))
    print("leg split          : %d outbound / %d return"
          % ((leg == 0).sum(), (leg == 1).sum()))

    sortie_design = np.zeros((len(names), sortie.max() + 1))
    sortie_design[np.arange(len(names)), sortie] = 1.0

    print("\n=== TEST 1 - is the alternation in the REFERENCE or the RECONSTRUCTION? ===")
    print("    (each series has its own per-sortie mean removed first)")
    for label, series in (("barometric AGL", baro),
                          ("EXIF GPS altitude", gps_alt),
                          ("reconstructed Z", zs)):
        centred, _ = remove(sortie_design, series.astype(float))
        a = centred[(leg == 0)].mean()
        b = centred[(leg == 1)].mean()
        print("  %-20s outbound %+.3f  return %+.3f   ALTERNATION %+.3f m"
              % (label, a, b, a - b))

    residual = zs - baro

    print("\n=== TEST 2 - does the residual drift with time inside a sortie? ===")
    print("    (barometric drift over a sortie is a reference fault, not a block fault)")
    drift_removed = residual.copy()
    for s in range(sortie.max() + 1):
        m = sortie == s
        tt = times[m] - times[m].mean()
        slope, intercept = np.polyfit(tt, residual[m], 1)
        print("  sortie %d: %5.1f min, drift %+.3f m/min, span %+.3f m over the sortie"
              % (s, np.ptp(times[m]) / 60.0, slope * 60.0, slope * np.ptp(tt)))
        drift_removed[m] = residual[m] - (slope * tt + intercept)

    print("\n=== TEST 3 - which side drifts? GPS altitude is the independent arbiter ===")
    print("    (GPS altitude is noisy but has no systematic drift over minutes;")
    print("     the barometer does. Whichever series moves against GPS is at fault.)")
    for label, series in (("barometric - GPS", baro - gps_alt),
                          ("reconstructed - GPS", zs - gps_alt),
                          ("reconstructed - baro", residual)):
        spans = []
        for s in range(sortie.max() + 1):
            m = sortie == s
            tt = times[m] - times[m].mean()
            slope = np.polyfit(tt, series[m], 1)[0]
            spans.append(slope * np.ptp(tt))
        print("  %-22s per-sortie span: %s   mean |span| %.3f m"
              % (label, np.array2string(np.array(spans), precision=2), np.mean(np.abs(spans))))

    print("\n=== nested decomposition of (reconstructed Z - barometric AGL) ===")
    print("  raw                                   std %.3f m  ptp %.3f m"
          % (residual.std(), np.ptp(residual)))
    after_sortie, sortie_coef = remove(sortie_design, residual)
    print("  - per-sortie datum                    std %.3f m" % after_sortie.std())
    print("    sortie datum offsets (m): %s"
          % np.array2string(sortie_coef - sortie_coef.mean(), precision=3))
    print("  - per-sortie datum + linear drift     std %.3f m" % drift_removed.std())

    tilt_design = np.c_[sortie_design, xs, ys]
    after_tilt, _ = remove(tilt_design, drift_removed)
    print("  - ... + frame tilt                    std %.3f m" % after_tilt.std())

    quad_design = np.c_[tilt_design, xs * xs, ys * ys, xs * ys]
    after_quad, quad_coef = remove(quad_design, drift_removed)
    curvature = np.c_[xs * xs, ys * ys, xs * ys] @ quad_coef[-3:]
    dome = float(np.ptp(curvature))
    print("  - ... + quadratic                     std %.3f m" % after_quad.std())
    print("    >>> DOME AMPLITUDE across the block: %.3f m" % dome)

    # Strips alone, on the dome-free residual; avoids the collinearity bug.
    strip_means = group_means(after_quad, strip)
    banding_std = float(strip_means.std())
    banding_ptp = float(np.ptp(strip_means))
    alternating = float(np.mean(strip_means[::2]) - np.mean(strip_means[1::2]))
    print("    >>> STRIP-TO-STRIP scatter: std %.3f m, ptp %.3f m"
          % (banding_std, banding_ptp))
    print("    >>> ALTERNATING component (odd vs even strips): %+.3f m" % alternating)
    within = float(np.mean([after_quad[strip == k].std()
                            for k in range(strip.max() + 1)]))
    print("    >>> within-strip scatter (noise floor):     %.3f m" % within)

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(19, 5.6))
        lim = np.percentile(np.abs(after_quad), 98)
        s0 = ax[0].scatter(xs, ys, c=after_quad, s=12, cmap="RdBu_r", vmin=-lim, vmax=lim)
        ax[0].set_title("residual after datum+drift+tilt+dome (m)")
        plt.colorbar(s0, ax=ax[0])
        s1 = ax[1].scatter(xs[on_strip], ys[on_strip], c=strip[on_strip] % 2,
                           s=12, cmap="coolwarm")
        ax[1].set_title("strip parity - %d strips detected" % (strip.max() + 1))
        ax[2].plot(strip_means, "o-", ms=4)
        ax[2].axhline(0.0, color="k", lw=0.6)
        ax[2].set_title("per-strip mean residual (m)\nstd %.3f m, alternation %+.3f m"
                        % (banding_std, alternating))
        ax[2].set_xlabel("strip index (time order)")
        ax[2].grid(alpha=0.3)
        for a in ax[:2]:
            a.set_aspect("equal")
            a.set_xlabel("metres east")
        plt.tight_layout()
        plt.savefig(args.plot, dpi=110)
        print("\nplot: %s" % args.plot)


if __name__ == "__main__":
    main()
