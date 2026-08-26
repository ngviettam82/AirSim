#!/usr/bin/env python3
"""Decide whether a full-workspace test run passed. Exit 0 = pass, 1 = fail.

WHY THIS FILE EXISTS (P12.0, 2026-08-25). scripts/verify_workspace.sh produced the
project's headline number for months and COULD NOT FAIL. Three separate reasons, all
in one script: `colcon test ... || true` swallowed the runner's code; every
`colcon test-result` call sat inside a pipeline (`| tail -n 2`) or a command
substitution, both of which discard exit status; and the cross-check mismatch printed
the word WARNING and kept going. The last command in the file was a grep with a
`|| echo` fallback, so the script exited 0 no matter what.

That matters beyond tidiness: docs/preflight-checklist.md C1 reads "Dat khi: 12 pkg,
0 errors / 0 failures / 0 skipped" and names this script as how you check it. So the
first blocking gate before a real flight was being held by a human reading output --
exactly what a gate exists to replace.

The verdict lives here rather than inline so it can be fed known inputs and checked.
Same shape as gate_r0_verdict.py + selftest_gate_r0_verdict.sh.

FOUR RULES, and the last two are the ones that were missing:

  1. errors > 0        -> fail
  2. failures > 0      -> fail
  3. skipped > 0       -> fail. A skip is "not measured", and O3 says not-measured is
     not OK. The N-g shield deliberately emits GTEST_SKIP as FAILED TO MEASURE; that
     shield is allowed to exist only because someone looks when it fires.
  4. no readable Summary at all -> fail, never pass. If the run produced nothing to
     read, the honest verdict is that nothing was proven. Mapping "could not measure"
     onto the safe side is the failure mode CLAUDE.md section 5 records twice
     (peakAcceleration reading 0.897 for a real 55.7; counting republished messages
     for events, off by 43x).

Cross-check: colcon's own Summary total counts cases AND targets in one number, so it
must equal the per-package sum the caller computed. A mismatch means a package's
results were skipped by the loop -- it happened on 2026-08-24 (1105 vs 1116) and the
cross-check was the only thing that caught it.

Usage: python3 workspace_verdict.py <test_result_output_file> <expected_total>
       expected_total = per-package (cases + targets) summed by the caller.
"""
import re
import sys

SUMMARY = re.compile(
    r"^Summary:\s*(\d+)\s+tests?,\s*(\d+)\s+errors?,\s*(\d+)\s+failures?,\s*(\d+)\s+skipped",
    re.MULTILINE,
)

# Per-file lines, e.g.
#   build/uav_safety/test_results/uav_safety/x.gtest.xml: 16 tests, 0 errors, 1 failure, 0 skipped
# A path under test_results/ counts CASES; one under Testing/ counts TARGETS. colcon adds
# both into Summary, so one failing case is reported there as two.
PER_FILE = re.compile(
    r"^(\S+): \s*(\d+)\s+tests?,\s*(\d+)\s+errors?,\s*(\d+)\s+failures?,\s*(\d+)\s+skipped",
    re.MULTILINE,
)


def by_unit(text):
    """Totals from test_results/ only: {tests, errors, failures, skipped, targets}.

    build/<pkg>/Testing/ keeps a dated directory per run -- 30 of them for uav_safety on
    2026-08-25 -- and those Test.xml files carry cumulative counts, so anything summed
    from them is history, not this run. test_results/<pkg>/<target>.gtest.xml is one file
    per target, overwritten every run. It is the only non-historical source here, and
    counting it reproduces colcon's own total exactly: 1075 cases + 49 target files = 1124.
    """
    out = {"tests": 0, "errors": 0, "failures": 0, "skipped": 0, "targets": 0}
    for path, tests, errors, failures, skipped in PER_FILE.findall(text):
        if "/test_results/" not in path:
            continue
        out["targets"] += 1
        out["tests"] += int(tests)
        out["errors"] += int(errors)
        out["failures"] += int(failures)
        out["skipped"] += int(skipped)
    return out


def verdict(text, expected_total):
    """Return (ok, lines). Never returns ok=True on unreadable evidence."""
    reasons = []
    notes = []

    match = SUMMARY.search(text)
    if match is None:
        return False, [
            "  FAIL  no readable 'Summary:' line in colcon test-result output",
            "        nothing was measured, so nothing is proven (O3)",
        ]

    total, errors, failures, skipped = (int(g) for g in match.groups())
    unit = by_unit(text)
    notes.append(
        "  colcon Summary: %d tests, %d errors, %d failures, %d skipped "
        "(adds cases and targets together)" % (total, errors, failures, skipped)
    )

    if unit["targets"]:
        notes.append(
            "  by unit: %d case / %d target -- failing: %d case"
            % (unit["tests"], unit["targets"], unit["failures"])
        )
        if unit["failures"] != failures:
            notes.append(
                "  (Summary said %d; each failing case appears there twice, once per unit)"
                % failures)
        if unit["failures"]:
            reasons.append("%d failing test case(s)" % unit["failures"])
        if unit["errors"]:
            reasons.append("%d error(s)" % unit["errors"])
        if unit["skipped"]:
            reasons.append("%d skipped -- not measured is not OK (O3)" % unit["skipped"])
    else:
        # No per-file lines to read: fall back on the Summary, which over-reports.
        if errors:
            reasons.append("%d error(s)" % errors)
        if failures:
            reasons.append("%d failure(s) (Summary count, may double-report)" % failures)
        if skipped:
            reasons.append("%d skipped -- not measured is not OK (O3)" % skipped)

    if expected_total is not None:
        if total != expected_total:
            reasons.append(
                "cross-check mismatch: colcon says %d, per-package sums to %d "
                "(a package's results were missed, or an xml double-counted)"
                % (total, expected_total)
            )
        else:
            notes.append("  cross-check: %d == %d ok" % (total, expected_total))

    if reasons:
        return False, notes + ["  FAIL  " + r for r in reasons]
    return True, notes + ["  PASS  workspace clean"]


def main():
    if len(sys.argv) < 2:
        print("usage: workspace_verdict.py <test_result_file> [expected_total]")
        return 2
    try:
        with open(sys.argv[1], encoding="utf-8", errors="replace") as handle:
            text = handle.read()
    except OSError as exc:
        print("  FAIL  cannot read evidence file: %s" % exc)
        return 1

    expected = None
    if len(sys.argv) > 2 and sys.argv[2] != "":
        try:
            expected = int(sys.argv[2])
        except ValueError:
            print("  FAIL  expected_total is not a number: %r" % sys.argv[2])
            return 1

    ok, lines = verdict(text, expected)
    for line in lines:
        print(line)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
