"""Split survey into odd/even flight strips; gate G1''', plan §5.4."""
import argparse
import csv
import os

import numpy as np

TURN_TOLERANCE_DEG = 45.0


def angular_gap(a, b):
    return np.abs((a - b + 180.0) % 360.0 - 180.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("exif_csv")
    parser.add_argument("out_dir")
    args = parser.parse_args()

    names, yaw, times = [], [], []
    with open(args.exif_csv, encoding="utf-8", errors="replace") as handle:
        for row in csv.DictReader(handle):
            name = row.get("FileName", "")
            if not name.upper().endswith(".JPG"):
                continue
            try:
                clock = row["DateTimeOriginal"].split(" ")[1].split(":")
                times.append(int(clock[0]) * 3600 + int(clock[1]) * 60 + int(clock[2]))
                yaw.append(float(row["FlightYawDegree"]))
                names.append(name)
            except (KeyError, ValueError, IndexError):
                continue
    yaw = np.array(yaw)
    times = np.array(times, dtype=float)
    print("images with usable metadata: %d" % len(names))

    order = np.argsort(times)
    sortie = np.zeros(len(names), dtype=int)
    count = 0
    for i in range(1, len(order)):
        if times[order[i]] - times[order[i - 1]] > 60:
            count += 1
        sortie[order[i]] = count

    principal = float(np.median(yaw[angular_gap(yaw, np.median(yaw)) < 90.0]))
    leg = np.full(len(names), -1, dtype=int)
    leg[angular_gap(yaw, principal) < TURN_TOLERANCE_DEG] = 0
    leg[angular_gap(yaw, principal + 180.0) < TURN_TOLERANCE_DEG] = 1

    strip = np.full(len(names), -1, dtype=int)
    current, prev_leg, prev_sortie = -1, None, None
    for i in order:
        if leg[i] < 0:
            continue
        if leg[i] != prev_leg or sortie[i] != prev_sortie:
            current += 1
        strip[i] = current
        prev_leg, prev_sortie = leg[i], sortie[i]
    print("strips: %d   images on strips: %d   mid-turn: %d"
          % (strip.max() + 1, (strip >= 0).sum(), (strip < 0).sum()))

    os.makedirs(args.out_dir, exist_ok=True)
    for parity, label in ((0, "odd"), (1, "even")):
        selected = [names[i] for i in range(len(names))
                    if strip[i] >= 0 and strip[i] % 2 == parity]
        path = os.path.join(args.out_dir, "strips_%s.txt" % label)
        # LF only: a stray CR breaks filenames read by bash.
        with open(path, "w", encoding="utf-8", newline="") as handle:
            handle.write("\n".join(selected) + "\n")
        print("%-5s : %4d images -> %s" % (label, len(selected), path))


if __name__ == "__main__":
    main()
