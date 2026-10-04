#!/usr/bin/env python3
"""Read the two arms of diagnose_rtf_stalls.sh and say what the stall tail is made of.

The question is not "what is the mean" -- three D3 runs already answered that. It is which
of these two shapes the tail has, because they need opposite fixes:

  a stall is ONE long step        -> the simulator blocked; look at render and at whatever
                                     holds the update loop
  a stall is MANY short steps     -> the simulator kept stepping and simply lost the race;
                                     look at total CPU, not at any single sensor

and whether the tail exists at all without the ROS stack next to it.
"""
import io
import os
import re
import sys

SIM = re.compile(r"sim_time\s*{([^}]*)}", re.S)
REAL = re.compile(r"real_time\s*{([^}]*)}", re.S)
SEC = re.compile(r"sec:\s*(-?\d+)")
NSEC = re.compile(r"nsec:\s*(-?\d+)")
RTF = re.compile(r"real_time_factor:\s*([0-9.eE+-]+)")


def seconds(block):
    if block is None:
        return None
    sec = SEC.search(block)
    nsec = NSEC.search(block)
    if sec is None:
        return None
    return int(sec.group(1)) + (int(nsec.group(1)) if nsec else 0) * 1e-9


def parse(path):
    """Each gz text message is separated by a blank line; take sim, real and the RTF field."""
    with io.open(path, "r", encoding="utf-8", errors="replace") as handle:
        text = handle.read()
    samples = []
    for chunk in text.split("\n\n"):
        if "real_time_factor" not in chunk:
            continue
        sim = seconds(SIM.search(chunk).group(1) if SIM.search(chunk) else None)
        real = seconds(REAL.search(chunk).group(1) if REAL.search(chunk) else None)
        reported = RTF.search(chunk)
        if sim is None or real is None or reported is None:
            continue
        samples.append((sim, real, float(reported.group(1))))
    return samples


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * (len(ordered) - 1)))]


def describe(name, samples):
    if len(samples) < 10:
        print("  arm %s: only %d samples -- FAILED TO MEASURE" % (name, len(samples)))
        return None

    reported = [rtf for _, _, rtf in samples]
    steps = []
    for older, newer in zip(samples, samples[1:]):
        d_sim = newer[0] - older[0]
        d_real = newer[1] - older[1]
        if d_real > 1e-9:
            steps.append((older[1], d_real, d_sim / d_real))

    derived = [rtf for _, _, rtf in steps]
    intervals = [d_real for _, d_real, _ in steps]
    slow = [(at, d_real, rtf) for at, d_real, rtf in steps if rtf < 0.95]
    lost = sum(d_real * (1.0 - rtf) for _, d_real, rtf in slow)
    span = samples[-1][1] - samples[0][1]

    print("  arm %s: %d samples over %.1f s of real time" % (name, len(samples), span))
    print("     reported RTF   min %.3f | p50 %.3f | mean %.3f"
          % (min(reported), percentile(reported, 0.5), sum(reported) / len(reported)))
    print("     derived  RTF   min %.3f | p50 %.3f | mean %.3f"
          % (min(derived), percentile(derived, 0.5), sum(derived) / len(derived)))
    print("     stats interval p50 %.3f s | p95 %.3f s | max %.3f s"
          % (percentile(intervals, 0.5), percentile(intervals, 0.95), max(intervals)))
    print("     intervals under 0.95x: %d of %d (%.1f%%), together %.2f s of sim time lost"
          % (len(slow), len(steps), 100.0 * len(slow) / len(steps), lost))
    if slow:
        worst = min(slow, key=lambda row: row[2])
        print("     worst interval: rtf %.3f lasting %.3f s, at %.1f s into the arm"
              % (worst[2], worst[1], worst[0] - samples[0][1]))
        gaps = [at - samples[0][1] for at, _, _ in slow]
        spacing = [b - a for a, b in zip(gaps, gaps[1:])]
        if spacing:
            print("     spacing between slow intervals: p50 %.2f s | min %.2f s | max %.2f s"
                  % (percentile(spacing, 0.5), min(spacing), max(spacing)))
    return {"mean": sum(derived) / len(derived), "slow": len(slow), "steps": len(steps),
            "lost": lost, "span": span}


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else "."
    results = {}
    for arm in ("A", "B"):
        path = os.path.join(directory, "stats_%s.txt" % arm)
        if not os.path.exists(path):
            print("  arm %s: no stats file -- FAILED TO MEASURE" % arm)
            continue
        results[arm] = describe(arm, parse(path))
        print("")

    a, b = results.get("A"), results.get("B")
    print("  ---- reading ----")
    if not a or not b:
        print("  one arm is missing, so nothing here separates the simulator from the stack.")
        return 1
    print("  simulator alone lost %.2f s of %.1f s; with the stack, %.2f s of %.1f s."
          % (a["lost"], a["span"], b["lost"], b["span"]))
    if a["lost"] > 0.25 * b["lost"] and a["lost"] > 0.05:
        print("  The tail is ALREADY THERE without the ROS stack: it belongs to the")
        print("  simulator's own loop. Phase or sensor-count changes are the lever.")
    elif b["lost"] > 4.0 * max(a["lost"], 1e-6):
        print("  The tail appears only WITH the stack beside it: the simulator is losing")
        print("  a CPU race, not stalling on render. Sensor merging would not touch it.")
    else:
        print("  Neither arm dominates. Say so; do not pick a fix from a tie.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
