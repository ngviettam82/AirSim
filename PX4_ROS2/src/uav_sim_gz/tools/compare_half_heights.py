"""Gate G1''' - odd/even strips agree on height; plan §5.4."""
import argparse
import json

import numpy as np
import rasterio
from rasterio.features import rasterize
from rasterio.warp import transform_geom
from scipy import ndimage

GROUND_WINDOW_M = 60.0


def heights_on(dsm_path, shapes):
    with rasterio.open(dsm_path) as ds:
        band = ds.read(1, masked=True)
        pixel_m = float(ds.res[0])
        transform_affine = ds.transform
        crs = ds.crs
        shape = (ds.height, ds.width)
    valid = ~band.mask
    elevation = np.asarray(band.filled(np.nan))
    factor = max(1, int(round(1.0 / pixel_m)))
    filled = np.where(valid, elevation, np.nanmax(elevation[valid]))
    coarse = filled[::factor, ::factor]
    window = max(3, int(round(GROUND_WINDOW_M / (pixel_m * factor))))
    ground = ndimage.minimum_filter(coarse, size=window)
    ground = np.repeat(np.repeat(ground, factor, 0), factor, 1)[:shape[0], :shape[1]]
    above = np.where(valid, filled - ground, np.nan)

    results = {}
    for name, geometry in shapes:
        projected = transform_geom("EPSG:4326", crs, geometry)
        mask = rasterize([(projected, 1)], out_shape=shape, transform=transform_affine,
                         fill=0, dtype="uint8").astype(bool)
        values = above[mask]
        values = values[np.isfinite(values)]
        if values.size >= 40:
            results[name] = float(np.percentile(values, 50))
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--footprints", required=True)
    parser.add_argument("--dsm", action="append", required=True,
                        help="name=path, repeatable")
    parser.add_argument("--tolerance", type=float, default=0.30)
    parser.add_argument("--buildings", default=None,
                        help="buildings CSV, to break the disagreement down by "
                             "the confidence this pipeline assigned")
    args = parser.parse_args()

    with open(args.footprints, encoding="utf-8") as handle:
        collection = json.load(handle)
    shapes = [(f["properties"]["id"], f["geometry"]) for f in collection["features"]]
    print("footprints: %d" % len(shapes))

    measured = {}
    for entry in args.dsm:
        name, path = entry.split("=", 1)
        measured[name] = heights_on(path, shapes)
        print("  %-10s measured %d buildings" % (name, len(measured[name])))

    names = list(measured)
    if len(names) < 2:
        raise SystemExit("Need at least two DSMs to compare.")

    common = set(measured[names[0]])
    for name in names[1:]:
        common &= set(measured[name])
    common = sorted(common)
    print("\nbuildings present in every reconstruction: %d" % len(common))

    print("\n=== pairwise agreement ===")
    verdicts = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            difference = np.array([measured[a][k] - measured[b][k] for k in common])
            print("  %-10s vs %-10s : bias %+.3f m  RMS %.3f m  p95|d| %.3f m  max %.3f m"
                  % (a, b, difference.mean(),
                     float(np.sqrt(np.mean(difference ** 2))),
                     float(np.percentile(np.abs(difference), 95)),
                     float(np.abs(difference).max())))
            verdicts.append(float(np.sqrt(np.mean(difference ** 2))))

    worst_pairs = sorted(
        ((abs(measured[names[0]][k] - measured[names[1]][k]), k) for k in common),
        reverse=True)[:8]
    print("\n  largest disagreements (%s vs %s):" % (names[0], names[1]))
    for gap, name in worst_pairs:
        print("      %-8s %.2f m   (%.2f vs %.2f)"
              % (name, gap, measured[names[0]][name], measured[names[1]][name]))

    if args.buildings:
        import csv
        table = {r["id"]: r for r in csv.DictReader(
            open(args.buildings, encoding="utf-8"))}
        print("\n=== does the confidence column predict the disagreement? ===")
        print("    (this is the test of the confidence column itself)")
        for a, b in ((names[0], names[-1]),):
            for level in ("high", "medium", "low"):
                subset = [k for k in common
                          if table.get(k, {}).get("confidence") == level]
                if len(subset) < 3:
                    continue
                difference = np.array([measured[a][k] - measured[b][k] for k in subset])
                centred = difference - difference.mean()
                print("  %-6s n=%2d : bias %+.2f m  scatter %.2f m  p90|d| %.2f m"
                      % (level, len(subset), difference.mean(),
                         float(np.sqrt(np.mean(centred ** 2))),
                         float(np.percentile(np.abs(centred), 90))))
            everything = np.array([measured[a][k] - measured[b][k] for k in common])
            centred = everything - everything.mean()
            robust = 1.4826 * float(np.median(np.abs(centred - np.median(centred))))
            print("  all    n=%2d : bias %+.2f m  scatter %.2f m  ROBUST scatter %.2f m"
                  % (len(common), everything.mean(),
                     float(np.sqrt(np.mean(centred ** 2))), robust))
            print("\n  The bias is NOT a datum artefact: the height is already the")
            print("  difference between a roof and its own ground within the same")
            print("  DSM, so a constant offset has cancelled before this point. Two")
            print("  reconstructions disagreeing by a metre in the mean means they")
            print("  genuinely place roofs differently relative to nearby ground.")
            print("  Total height uncertainty is therefore bias AND scatter together.")

    passed = max(verdicts) <= args.tolerance
    print("\n  G1''' %s (RMS between reconstructions <= %.2f m)"
          % ("PASS" if passed else "FAIL", args.tolerance))
    if passed:
        print("  Heights repeat across independent image geometry, so they are a")
        print("  property of the site and not of one particular bundle adjustment.")


if __name__ == "__main__":
    main()
