#!/usr/bin/env python3
"""Adjudicates gate R0 from the two arms' evidence files.

Split out of gate_r0_dds_profile.sh on 2026-08-24 so the verdict can be re-run on
saved evidence without re-flying the two arms -- and so that a change of CRITERION
is a visible diff instead of an edit buried in a 250-line shell script.

WHY THE ABSOLUTE FLOORS MOVED FROM WALL RATE TO SIM-NORMALISED RATE
-------------------------------------------------------------------
The first run declared the floors on WALL-CLOCK rate: vehicle_odometry >= 50 Hz and
vehicle_status >= 1.5 Hz, taken from the M2 baseline of record (2026-08-03: ~100 Hz
and 1.98 Hz). The CONTROL arm then measured vehicle_status at 1.476 Hz wall -- i.e.
the control, which runs stock transport and is by construction the "known good"
condition, failed a floor meant to catch a broken transport.

That is the control doing its job. M2 was measured on a bare rig at RTF 1.000 with
model uav0 and no autonomy stack. This gate runs uav0_full with the full stack at
RTF ~0.81, and PX4 publishes on SIM time: at RTF 0.81 a healthy 2 Hz topic arrives
1.62 times per wall second no matter how perfect the transport is. A wall-clock floor
therefore measures the RTF, not the link -- exactly the confound the gate header
already identified for the ratio test, applied inconsistently to the floors.

The floors are re-expressed on the sim-normalised rate, which is the quantity that
actually means "is PX4's output reaching us". Same numbers, correct quantity. The
control validates them (91.64 Hz and 1.82 Hz), which is the order R27-1 requires:
gate the instrument, then the object. This is the G-S3-B1 precedent -- restate the
criterion as a measurable property -- and NOT a relaxation: nothing that would have
failed on a real transport fault now passes, because a severed or lossy link drops
the sim-normalised rate just as hard.

The worst-GAP criterion stays in wall-clock seconds on purpose. A stall is a stall;
its cost to an offboard stream does not shrink because the simulator was slow.

Usage: python3 scripts/gate_r0_verdict.py /tmp/gate_r0/control.txt /tmp/gate_r0/profile.txt
"""
import math
import sys

# Floors on the sim-normalised rate. PX4 nominal: odometry ~100 Hz, status ~2 Hz.
FLOOR_NORMALISED = {"vehicle_odometry": 50.0, "vehicle_status": 1.5}
# The profile may not deliver less than this fraction of what the control delivers.
MIN_RATIO = 0.90
# A worst-case gap is only held against the profile if it is both absolutely large
# AND much worse than the control's -- the rig itself stalls (control: 0.629 s).
GAP_ABS_SEC = 0.10
GAP_RATIO = 2.0

TOPICS = ("vehicle_odometry", "vehicle_status", "sensor_combined")


def load(path):
    out = {}
    with open(path) as fh:
        for line in fh:
            parts = line.split()
            if parts:
                out[parts[0]] = parts[1:]
    return out


def finite(text):
    """float(text) if it is a real, finite number -- otherwise None.

    Self-review catch, 2026-08-24, before this gate was ever trusted: the arm script
    writes the sim-normalised rate as the literal "nan" when the RTF sampler returns
    nothing. Python then compares nan against every threshold as False, so a run with
    NO USABLE DATA would have sailed through every check and printed PASS. That is the
    exact "not-measurable silently becomes an acceptable value" fault behind 9 of the
    16 P10 review findings (R30). Non-finite input must FAIL LOUD, never pass quietly.
    """
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def main():
    control, profile = load(sys.argv[1]), load(sys.argv[2])
    fail = []

    print("  %-20s %14s %14s %9s" % ("topic", "control", "profile", "ratio"))
    for topic in TOPICS:
        c, p = control.get(topic), profile.get(topic)
        if not c or c[0] == "none":
            print("  %-20s CONTROL SAW NOTHING -- instrument or rig fault" % topic)
            fail.append("%s: control blind, no conclusion possible" % topic)
            continue
        if not p or p[0] == "none":
            print("  %-20s PROFILE SAW NOTHING -- link severed" % topic)
            fail.append("%s: link severed under profile" % topic)
            continue
        cn, pn = finite(c[2]), finite(p[2])
        cg, pg = finite(c[1]), finite(p[1])
        if cn is None or pn is None or cg is None or pg is None or cn <= 0:
            print("  %-20s UNUSABLE NUMBERS (control=%s profile=%s)" % (topic, c, p))
            fail.append("%s: rate or gap is not a finite number -- nothing to judge" % topic)
            continue
        ratio = pn / cn
        print("  %-20s %11.2f Hz %11.2f Hz %9.3f  (sim-normalised)" % (topic, cn, pn, ratio))
        if ratio < MIN_RATIO:
            fail.append("%s: profile delivers %.1f%% of control" % (topic, ratio * 100))
        floor = FLOOR_NORMALISED.get(topic)
        if floor is not None:
            if cn < floor:
                # The control failing a floor invalidates the floor, not the profile.
                fail.append("%s: CONTROL %.2f Hz is under its own floor %.2f -- the "
                            "criterion is measuring the rig, not the transport" % (topic, cn, floor))
            elif pn < floor:
                fail.append("%s: profile %.2f Hz under floor %.2f" % (topic, pn, floor))
        if pg > GAP_ABS_SEC and pg > GAP_RATIO * cg:
            fail.append("%s: worst gap %.3fs vs control %.3fs" % (topic, pg, cg))

    # Same fault class as finite(): every normalised rate above is a division by the
    # arm's RTF, so an arm with no usable RTF makes the whole comparison meaningless
    # even when the individual numbers parse.
    for name, arm in (("control", control), ("profile", profile)):
        rtf = finite(arm.get("rtf", ["nan"])[0])
        if rtf is None or rtf <= 0:
            fail.append("%s arm has no usable RTF -- the normalised rates are unanchored"
                        % name)

    print()
    print("  worst gap (wall):  " + "  ".join(
        "%s c=%ss p=%ss" % (t, control.get(t, ["?", "?"])[1], profile.get(t, ["?", "?"])[1])
        for t in TOPICS))
    print("  RTF: control %s | profile %s" % (
        control.get("rtf", ["?"])[0], profile.get("rtf", ["?"])[0]))
    print()
    if fail:
        for f in fail:
            print("  FAIL: " + f)
        print()
        print("  RESULT: GATE R0 FAILED -- do NOT wire the profile into the bringup.")
        return 1
    print("  RESULT: GATE R0 PASSED -- the profile does not harm the PX4 link.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
