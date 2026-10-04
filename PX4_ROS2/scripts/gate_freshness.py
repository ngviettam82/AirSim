#!/usr/bin/env python3
"""Shared staleness/sample-count logic for scripts/preflight_light.py (N8)
and src/uav_observability/test/o4_report.py (N7) -- P10-gate-debt review
round 2, 2026-08-24. One place so the two readers of
/uav/<id>/state/system_health cannot silently diverge on what "fresh"
means (task's own instruction: "dung chung helper voi preflight_light.py
de hai cho khong lech nhau").

Pure functions only, no ROS import -- usable from a live rclpy subscriber
AND a post-hoc JSONL reader alike.
"""


def is_fresh(age_sec, max_age_sec):
    """True iff 0 <= age_sec <= max_age_sec.

    N8: age_sec NEGATIVE (companion clock ahead of the publisher's clock,
    e.g. NTP not synced or stepping backward) must FAIL, same direction as
    R30/R32's "0 is not a safe default" -- a naive `age_sec <= max_age_sec`
    lets an arbitrarily large negative age read as fresh.
    NaN also fails this comparison in both directions (Python: any
    comparison with NaN is False), which is the same fail-closed direction
    already used for "never received"/"unparsable" (R30).
    """
    return 0.0 <= age_sec <= max_age_sec


def min_expected_samples(duration_sec, publish_period_sec, margin=0.5):
    """Lower bound on sample count over duration_sec at publish_period_sec
    cadence.

    N7: a stalled /clock freezes diagnostics_node's eval timer AND publish
    timer TOGETHER -- a live node under a frozen clock keeps its process
    alive and its TransientLocal last sample latched, but produces far
    FEWER samples than a healthy node over the same wall-clock duration,
    not just a stale LAST sample. `margin` discounts the naive
    duration/period estimate for startup/discovery settle time (the first
    few seconds of any sampling window are not yet steady-state).
    """
    if publish_period_sec is None or publish_period_sec <= 0:
        return 1
    return max(1, int((duration_sec / publish_period_sec) * margin))
