#!/usr/bin/env python3
"""Decide whether the EKF2 "EV aiding restarts every second" debt is real.

See scripts/diagnose_ev_reset_cadence.sh for the full argument. In one line: EKF2
re-publishes the previous estimator_event_flags message once per second when nothing
new happens (EKF2.cpp:1129), so counting messages that carry a latched bit counts
republications, not events. `information_event_changes` is a counter that only moves on
a real event, so it separates the two.

Prints a verdict and exits 0 if the evidence is conclusive either way, 2 if the log
cannot answer the question (R30: not-measurable is never quietly reported as fine).
"""
import sys
from collections import Counter

from pyulog import ULog

EV_INFO_BITS = (
    "reset_pos_to_vision",
    "reset_vel_to_vision",
    "reset_hgt_to_ev",
    "starting_vision_pos_fusion",
    "starting_vision_vel_fusion",
    "starting_vision_yaw_fusion",
)
EV_STATUS_FLAGS = ("cs_ev_pos", "cs_ev_vel", "cs_ev_yaw", "cs_ev_hgt")


def get(ulog, name):
    for dataset in ulog.data_list:
        if dataset.name == name:
            return dataset
    return None


def series(dataset, field):
    return dataset.data[field] if field in dataset.data else None


def main():
    ulog = ULog(sys.argv[1])

    ev = get(ulog, "estimator_event_flags")
    if ev is None:
        print("FAILED TO MEASURE: this log has no estimator_event_flags at all.")
        return 2

    t = series(ev, "timestamp")
    n = len(t)
    span = (t[-1] - t[0]) / 1e6 if n > 1 else 0.0
    print("estimator_event_flags: %d messages over %.1f s" % (n, span))

    # 1. How many messages carry each latched EV bit.
    print()
    print("  latched bits (how many MESSAGES say true -- NOT how many events):")
    any_ev_bit = False
    for bit in EV_INFO_BITS:
        s = series(ev, bit)
        if s is None:
            continue
        c = int(sum(1 for v in s if v))
        if c:
            any_ev_bit = True
        print("    %-28s %5d / %d messages" % (bit, c, n))
    if not any_ev_bit:
        print("  FAILED TO MEASURE: no EV information bit is set anywhere in this log,")
        print("  so it cannot speak to a debt about EV resets. Fly with the vision")
        print("  stack up and re-run.")
        return 2

    # 2. The discriminator.
    counter = series(ev, "information_event_changes")
    if counter is None:
        print("  FAILED TO MEASURE: information_event_changes absent from this log.")
        return 2
    distinct = sorted(set(int(v) for v in counter))
    print()
    print("  information_event_changes: %d distinct value(s) across %d messages"
          % (len(distinct), n))
    print("    values: %s%s" % (distinct[:12], " ..." if len(distinct) > 12 else ""))

    # 3. Cadence: is the message spacing the +1_s republish, or event-driven?
    if n > 2:
        gaps = [(t[i + 1] - t[i]) / 1e6 for i in range(n - 1)]
        gaps_sorted = sorted(gaps)
        med = gaps_sorted[len(gaps_sorted) // 2]
        near_one = sum(1 for g in gaps if 0.95 <= g <= 1.10)
        print()
        print("  message spacing: median %.3f s | min %.3f | max %.3f"
              % (med, gaps_sorted[0], gaps_sorted[-1]))
        print("    %d of %d gaps are within [0.95, 1.10] s -- the `+ 1_s` republish cadence"
              % (near_one, len(gaps)))

    # 4. Corroboration from control status, which is STATE, not a latched event.
    st = get(ulog, "estimator_status_flags")
    print()
    if st is None:
        print("  (no estimator_status_flags in this log -- corroboration unavailable)")
    else:
        print("  control status (state, not latched -- if EV fusion really stopped and")
        print("  restarted each second these could not sit at 100%):")
        for flag in EV_STATUS_FLAGS:
            s = series(st, flag)
            if s is None:
                continue
            true_share = 100.0 * sum(1 for v in s if v) / len(s)
            print("    %-10s true in %6.1f%% of %d samples" % (flag, true_share, len(s)))

    # 4b. INDEPENDENT CORROBORATION from a different topic and a different code path.
    #     The debt makes a second claim: "bộ đếm reset của vehicle_odometry tăng liên
    #     tục". vehicle_local_position carries the authoritative per-axis reset counters,
    #     and EKF2 increments them in resetHorizontalPositionTo()/resetVelocityTo() --
    #     i.e. in the very function the debt says runs 71 times. If the estimator really
    #     reset position and velocity once per second, these MUST show it. They are not
    #     latched bits and they are not republished, so they cannot lie the same way.
    lp = get(ulog, "vehicle_local_position")
    print()
    if lp is None:
        print("  (no vehicle_local_position in this log -- corroboration unavailable)")
    else:
        lp_t = series(lp, "timestamp")
        lp_span = (lp_t[-1] - lp_t[0]) / 1e6 if lp_t is not None and len(lp_t) > 1 else 0.0
        print("  estimator reset counters (independent of estimator_event_flags):")
        for field in ("xy_reset_counter", "z_reset_counter", "vxy_reset_counter",
                      "vz_reset_counter", "heading_reset_counter"):
            s = series(lp, field)
            if s is None:
                continue
            moved = int(max(s)) - int(min(s))
            rate = moved / lp_span if lp_span > 0 else 0.0
            print("    %-22s moved %4d time(s) over %.1f s = %.3f /s"
                  % (field, moved, lp_span, rate))

    # 5. WHEN did the counter actually move? A handful of events all inside the first
    #    seconds is a start-up transient; the same handful spread evenly is not.
    changes = []
    for i in range(1, n):
        if int(counter[i]) != int(counter[i - 1]):
            changes.append((t[i] - t[0]) / 1e6)
    print()
    print("  the counter moved %d time(s), at t+ %s s (log-relative)"
          % (len(changes), ", ".join("%.1f" % c for c in changes[:15])))
    if changes:
        print("    last real event at t+%.1f s of %.1f s" % (changes[-1], span))

    # 6. Verdict, taken on the RATE -- the debt's claim is "once per second", so the
    #    quantity that answers it is events per second, not the raw event count. The
    #    first version of this script compared the count against a flat threshold of 2
    #    and called 5 events in 73 s "REAL"; that was the wrong quantity, in the same
    #    family as judging a link by wall-clock rate while the publisher runs on sim
    #    time. Rate, then verdict.
    print()
    if span <= 0:
        print("FAILED TO MEASURE: log span is zero.")
        return 2
    events = int(max(counter)) - int(min(counter))
    rate = events / span
    latched = max(int(sum(1 for v in series(ev, b) if v))
                  for b in EV_INFO_BITS if series(ev, b) is not None)
    print("  real events %d over %.1f s = %.3f /s   |   messages carrying a latched EV bit: %d"
          % (events, span, rate, latched))
    print("  the debt claims ~1.0 /s")

    if rate >= 0.5:
        print()
        print("VERDICT: the debt is REAL -- EV events really do recur at %.2f /s." % rate)
        print("  Proceed to the replay step to find which branch fires them.")
        return 0
    if rate <= 0.1:
        print()
        print("VERDICT: the debt as recorded is a MEASUREMENT ARTEFACT.")
        print("  %d real events in %.1f s (%.3f /s) against %d messages carrying the latched"
              % (events, span, rate, latched))
        print("  bit. EKF2 republishes the previous estimator_event_flags once per second")
        print("  when nothing new happens (EKF2.cpp:1129), so counting messages counted")
        print("  republications. The estimator did NOT restart EV aiding every second.")
        if changes and changes[-1] < 0.5 * span:
            print("  Corroborated: the last real event is at t+%.1f s of %.1f s -- the events"
                  % (changes[-1], span))
            print("  are a start-up transient, not a steady-state cycle.")
        return 0
    print()
    print("VERDICT: INCONCLUSIVE at %.3f /s -- between the two decision bands." % rate)
    print("  Do not read this as either outcome. Lengthen the run and measure again.")
    return 2


if __name__ == "__main__":
    sys.exit(main())
