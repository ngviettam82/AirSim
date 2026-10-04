"""Gate G1' - doming from barometric residual; ABANDONED, plan §5.1."""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

# Only the shots block is parsed; points dominate the file.
SHOTS_KEY = '"shots":'


def extract_shots(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        text = handle.read()
    start = text.find(SHOTS_KEY)
    if start < 0:
        raise SystemExit("No shots block in %s" % path)
    brace = text.find("{", start + len(SHOTS_KEY))
    depth, i = 0, brace
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                break
        i += 1
    return json.loads(text[brace:i + 1])


def camera_centre(shot: dict) -> np.ndarray:
    """OpenSFM stores world->camera; the centre is -R^T t."""
    rotation = np.asarray(shot["rotation"], dtype=float)
    translation = np.asarray(shot["translation"], dtype=float)
    theta = float(np.linalg.norm(rotation))
    if theta < 1e-12:
        rot = np.eye(3)
    else:
        axis = rotation / theta
        skew = np.array([[0.0, -axis[2], axis[1]],
                         [axis[2], 0.0, -axis[0]],
                         [-axis[1], axis[0], 0.0]])
        rot = np.eye(3) + math.sin(theta) * skew + (1 - math.cos(theta)) * (skew @ skew)
    return -rot.T @ translation


def load_baro(csv_path: str) -> dict:
    baro = {}
    with open(csv_path, encoding="utf-8", errors="replace") as handle:
        for row in csv.DictReader(handle):
            try:
                baro[row["FileName"]] = float(row["RelativeAltitude"])
            except (KeyError, TypeError, ValueError):
                continue
    return baro


def report_stats(opensfm_dir: str) -> None:
    path = os.path.join(opensfm_dir, "stats", "stats.json")
    if not os.path.isfile(path):
        return
    with open(path, encoding="utf-8") as handle:
        stats = json.load(handle)
    recon = stats.get("reconstruction_statistics", {})
    print("=== ODM reconstruction statistics ===")
    for key in ("reconstructed_shots_count", "initial_shots_count",
                "reconstructed_points_count", "observations_count",
                "average_track_length", "mean_reprojection_error"):
        if key in recon:
            print("  %-32s %s" % (key, recon[key]))
    errors = recon.get("gps_errors") or {}
    if errors:
        print("  GPS prior vs reconstructed (metres):")
        for axis, value in errors.items():
            print("    %-10s %s" % (axis, value))
    print()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("opensfm_dir")
    parser.add_argument("exif_csv")
    parser.add_argument("--tolerance", type=float, default=0.5)
    parser.add_argument("--plot", default=None)
    args = parser.parse_args()

    report_stats(args.opensfm_dir)

    recon_path = os.path.join(args.opensfm_dir, "reconstruction.topocentric.json")
    if not os.path.isfile(recon_path):
        recon_path = os.path.join(args.opensfm_dir, "reconstruction.json")
    print("shots from: %s" % recon_path)
    shots = extract_shots(recon_path)
    baro = load_baro(args.exif_csv)
    names = sorted(n for n in shots if n in baro)
    print("reconstructed shots: %d   matched to barometric records: %d\n"
          % (len(shots), len(names)))
    if len(names) < 100:
        raise SystemExit("Too few matched shots to judge the block.")

    centres = np.array([camera_centre(shots[n]) for n in names], dtype=float)
    xs, ys, zs = centres[:, 0], centres[:, 1], centres[:, 2]
    hb = np.array([baro[n] for n in names], dtype=float)
    xs = xs - xs.mean()
    ys = ys - ys.mean()

    print("=== block geometry ===")
    print("  extent            %.0f m (E) x %.0f m (N)" % (np.ptp(xs), np.ptp(ys)))
    print("  reconstructed Z   median %.2f m, std %.3f m" % (np.median(zs), np.std(zs)))
    print("  barometric AGL    median %.2f m, std %.3f m" % (np.median(hb), np.std(hb)))

    # Datum offset and tilt are not defects, only curvature.
    residual = zs - hb
    plane = np.c_[np.ones_like(xs), xs, ys]
    plane_coef, *_ = np.linalg.lstsq(plane, residual, rcond=None)
    flat = residual - plane @ plane_coef
    print("\n=== residual (reconstructed - barometric) ===")
    print("  after removing datum + tilt: std %.3f m, peak-to-peak %.3f m"
          % (np.std(flat), np.ptp(flat)))

    quad = np.c_[plane, xs * xs, ys * ys, xs * ys]
    quad_coef, *_ = np.linalg.lstsq(quad, residual, rcond=None)
    curvature = quad @ quad_coef - plane @ quad_coef[:3]
    amplitude = float(np.ptp(curvature))
    rms_after = float(np.sqrt(np.mean((residual - quad @ quad_coef) ** 2)))
    print("  quadratic terms: xx=%.3e  yy=%.3e  xy=%.3e"
          % (quad_coef[3], quad_coef[4], quad_coef[5]))
    print("  RMS residual after quadratic fit: %.3f m" % rms_after)
    print("\n  DOME AMPLITUDE across block: %.3f m" % amplitude)

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 3, figsize=(19, 5.6))
        s0 = axes[0].scatter(xs, ys, c=flat, s=12, cmap="RdBu_r",
                             vmin=-np.percentile(np.abs(flat), 98),
                             vmax=np.percentile(np.abs(flat), 98))
        axes[0].set_title("residual after datum+tilt removal (m)")
        plt.colorbar(s0, ax=axes[0])
        s1 = axes[1].scatter(xs, ys, c=curvature, s=12, cmap="RdBu_r")
        axes[1].set_title("fitted curvature = the dome (m)\namplitude %.3f m" % amplitude)
        plt.colorbar(s1, ax=axes[1])
        axes[2].hist(flat, bins=60)
        axes[2].set_title("residual histogram (m)\nstd %.3f m" % np.std(flat))
        for ax in axes[:2]:
            ax.set_aspect("equal")
            ax.set_xlabel("metres east")
            ax.set_ylabel("metres north")
        plt.tight_layout()
        plt.savefig(args.plot, dpi=110)
        print("  plot: %s" % args.plot)

    passed = amplitude <= args.tolerance
    print("\nG1' %s (tolerance %.2f m)" % ("PASS" if passed else "FAIL", args.tolerance))
    if not passed:
        print("Block is bowed; do not build the world on this reconstruction.")
        print("Remedies: --camera-lens brown with tighter --gps-accuracy, or an")
        print("independent bundle in COLMAP to separate data fault from tool fault.")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
