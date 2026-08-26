#!/usr/bin/env python3
"""Positive control for the EV-reset discriminator, using evidence already on disk.

scripts/ev_reset_cadence_report.py claims it can tell a real recurring EKF2 event train
from EKF2 merely republishing one latched message every second. A discriminator nobody
has seen separate two different answers is not a discriminator -- it is an assertion.

So: run it over EVERY ulog in the PX4 log tree and print the distribution of real event
rates. If the measure works, the corpus must contain BOTH kinds of run -- some near zero
(latched republication only) and some clearly higher (genuine repeated events). If every
log in 398 came out identical, the number would be measuring the tool, not the flights.

Reads only the two datasets it needs, so a full sweep stays cheap.

Usage: python3 scripts/sweep_ev_event_rates.py <log_root> [max_logs]
"""
import os
import sys

from pyulog import ULog

WANT = ["estimator_event_flags"]
EV_BITS = (
    "reset_pos_to_vision",
    "reset_vel_to_vision",
    "reset_hgt_to_ev",
    "starting_vision_pos_fusion",
    "starting_vision_vel_fusion",
    "starting_vision_yaw_fusion",
)


def scan(path):
    """(rate, events, span, latched_msgs, total_msgs) or None if unusable."""
    try:
        ulog = ULog(path, message_name_filter_list=WANT)
    except Exception:
        return None
    ev = next((d for d in ulog.data_list if d.name == "estimator_event_flags"), None)
    if ev is None or "information_event_changes" not in ev.data:
        return None
    t = ev.data["timestamp"]
    if len(t) < 3:
        return None
    span = (t[-1] - t[0]) / 1e6
    if span <= 5.0:
        return None
    counter = ev.data["information_event_changes"]
    events = int(max(counter)) - int(min(counter))
    latched = 0
    for bit in EV_BITS:
        if bit in ev.data:
            latched = max(latched, int(sum(1 for v in ev.data[bit] if v)))
    return events / span, events, span, latched, len(t)


def main():
    root = sys.argv[1]
    cap = int(sys.argv[2]) if len(sys.argv) > 2 else 10000
    paths = []
    for dirpath, _, names in os.walk(root):
        for name in names:
            if name.endswith(".ulg"):
                paths.append(os.path.join(dirpath, name))
    paths.sort(key=os.path.getmtime, reverse=True)
    paths = paths[:cap]

    rows = []
    for p in paths:
        r = scan(p)
        if r is not None:
            rows.append((r, p))
    if not rows:
        print("FAILED TO MEASURE: no usable log found under %s" % root)
        return 2

    rates = sorted(r[0][0] for r in rows)
    print("usable logs: %d of %d scanned" % (len(rows), len(paths)))
    print("real-event rate (events/s), across the corpus:")
    print("  min %.3f | p50 %.3f | p90 %.3f | max %.3f"
          % (rates[0], rates[len(rates) // 2], rates[int(0.9 * (len(rates) - 1))], rates[-1]))

    print()
    print("  the 8 HIGHEST -- if the measure works, these are visibly different:")
    for (rate, events, span, latched, total), p in sorted(rows, key=lambda x: -x[0][0])[:8]:
        print("    %6.3f /s  events=%-4d span=%6.1fs latched_msgs=%-4d msgs=%-4d  %s"
              % (rate, events, span, latched, total, os.path.basename(p)))

    print()
    print("  the 5 LOWEST:")
    for (rate, events, span, latched, total), p in sorted(rows, key=lambda x: x[0][0])[:5]:
        print("    %6.3f /s  events=%-4d span=%6.1fs latched_msgs=%-4d msgs=%-4d  %s"
              % (rate, events, span, latched, total, os.path.basename(p)))

    print()
    band_low = sum(1 for r in rates if r <= 0.1)
    band_high = sum(1 for r in rates if r >= 0.5)
    band_mid = len(rates) - band_low - band_high
    print("  decision bands used by ev_reset_cadence_report.py:")
    print("    <=0.10 /s (artefact)   %d logs" % band_low)
    print("    0.10-0.50 (inconclusive) %d logs" % band_mid)
    print("    >=0.50 /s (real)       %d logs" % band_high)
    print()
    if band_high == 0:
        print("NOTE: no log in the corpus reaches the 'real' band. That does NOT prove the")
        print("  measure is broken -- it is consistent with 'this fault never happened'.")
        print("  But it means the corpus alone cannot demonstrate the measure separates")
        print("  two answers; the spread above is the evidence to read instead.")
    else:
        print("The corpus contains BOTH bands, so the measure demonstrably separates two")
        print("different answers rather than always returning the same one.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
