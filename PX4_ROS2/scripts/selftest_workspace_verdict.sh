#!/bin/bash
# Positive control for workspace_verdict.py (P12.0, 2026-08-25).
#
# A gate nobody has seen refuse is a gate nobody knows works. Before this file,
# verify_workspace.sh had never once exited non-zero -- not because the workspace was
# always clean, but because the script had no path to a non-zero exit at all. So the
# first thing the replacement owes is proof that it CAN say no.
#
# The other half matters just as much, and the house rule is explicit about it: a
# shield must not become an amnesty. Case 1 proves a genuinely clean workspace still
# passes, so the new verdict cannot be "fail everything and look rigorous".
#
# Cases 4 and 5 are the two rules that did not exist before at all:
#   - skipped > 0 is a failure (O3: not measured is not OK). The N-g witness emits
#     GTEST_SKIP as FAILED TO MEASURE precisely so a human looks; silently passing it
#     turns that shield into the amnesty it was written not to be.
#   - a missing Summary line is a failure, never a pass. This is the CLAUDE.md section 5
#     failure mode: "could not measure" quietly landing on the safe side.
#
# Case 6 replays a real incident: 2026-08-24, colcon said 1105 while the per-package
# loop summed to 1116, because check_test_domain_isolation.sh ran its own ctest inside
# the window and three runs trampled build/*/Testing/. The product was fine; the
# measurement was not. The cross-check was the only thing that noticed, and back then
# it only printed the word WARNING.
#
# Usage: bash scripts/selftest_workspace_verdict.sh
set -o pipefail

CANON=/mnt/c/code/PX4_ROS2
VERDICT="$CANON/scripts/workspace_verdict.py"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
fails=0

check() {
  local name=$1 want=$2 expected=$3 body=$4
  printf '%s\n' "$body" > "$TMP/tr.txt"
  python3 "$VERDICT" "$TMP/tr.txt" "$expected" > "$TMP/out.txt" 2>&1
  local got=$?
  if [ "$got" -eq "$want" ]; then
    echo "  ok    $name (exit $got as expected)"
  else
    echo "  FAIL  $name: expected exit $want, got $got"
    sed 's/^/          /' "$TMP/out.txt"
    fails=$((fails + 1))
  fi
}

CLEAN='build/uav_safety/test_results/uav_safety/test_cut_chain.gtest.xml: 40 tests, 0 errors, 0 failures, 0 skipped

Summary: 1116 tests, 0 errors, 0 failures, 0 skipped'

TWO_FAILURES='Summary: 1124 tests, 0 errors, 2 failures, 0 skipped'
HAS_ERRORS='Summary: 1124 tests, 3 errors, 0 failures, 0 skipped'
HAS_SKIPPED='Summary: 1124 tests, 0 errors, 0 failures, 5 skipped'
NO_SUMMARY='build/uav_safety/test_results/uav_safety/test_cut_chain.gtest.xml: 40 tests, 0 errors, 0 failures, 0 skipped'
GARBAGE='Summary: some tests, no errors, a few failures'

echo "=== selftest: workspace_verdict.py ==="
check "1 clean workspace still passes (shield is not amnesty)" 0 1116 "$CLEAN"
check "2 failures are caught (the state on 2026-08-25)"        1 1124 "$TWO_FAILURES"
check "3 errors are caught"                                    1 1124 "$HAS_ERRORS"
check "4 skipped is caught (O3: not measured is not OK)"       1 1124 "$HAS_SKIPPED"
check "5 missing Summary fails, never passes"                  1 1116 "$NO_SUMMARY"
check "6 cross-check mismatch is fatal, not a warning"         1 1116 "$TWO_FAILURES"
check "7 empty evidence fails"                                 1 1116 ""
check "8 unparseable Summary fails"                            1 1116 "$GARBAGE"

# Clean numbers but a mismatched total: the 1105-vs-1116 incident, isolated.
CLEAN_WRONG_TOTAL='Summary: 1105 tests, 0 errors, 0 failures, 0 skipped'
check "9 clean run with missed package is caught by cross-check" 1 1116 "$CLEAN_WRONG_TOTAL"

# No expected_total given: cross-check is skipped, the other rules still apply.
check "10 without cross-check, clean still passes" 0 "" "$CLEAN"
check "11 without cross-check, failures still caught" 1 "" "$TWO_FAILURES"

# Cases 12-14 exercise the per-file path, which is what a real run produces.
#
# The unit split matters. colcon lists two kinds of file and adds them into one Summary:
# test_results/<pkg>/<target>.gtest.xml counts CASES and is rewritten every run, while
# Testing/<date>/Test.xml counts TARGETS and accumulates -- uav_safety had 30 dated
# directories on 2026-08-25. So Summary over-reports failures (one failing case shows up
# once per unit), and anything summed from Testing/ is history rather than this run. Both
# mistakes were made and measured before these cases were written.

REAL_ONE_FAILURE='build/uav_safety/Testing/20260825-0401/Test.xml: 3 tests, 0 errors, 1 failure, 0 skipped
build/uav_safety/test_results/uav_safety/test_safety_cut_chain.gtest.xml: 16 tests, 0 errors, 1 failure, 0 skipped
build/uav_mission/test_results/uav_mission/test_mission_executor_node.gtest.xml: 23 tests, 0 errors, 0 failures, 0 skipped

Summary: 42 tests, 0 errors, 2 failures, 0 skipped'

REAL_CLEAN='build/uav_safety/test_results/uav_safety/test_safety_cut_chain.gtest.xml: 16 tests, 0 errors, 0 failures, 0 skipped
build/uav_mission/test_results/uav_mission/test_mission_executor_node.gtest.xml: 23 tests, 0 errors, 0 failures, 0 skipped

Summary: 41 tests, 0 errors, 0 failures, 0 skipped'

STALE_HISTORY_ONLY='build/uav_safety/Testing/20260801-0101/Test.xml: 3 tests, 0 errors, 2 failures, 0 skipped
build/uav_safety/Testing/20260812-0202/Test.xml: 3 tests, 0 errors, 1 failure, 0 skipped
build/uav_safety/test_results/uav_safety/test_safety_cut_chain.gtest.xml: 16 tests, 0 errors, 0 failures, 0 skipped
build/uav_mission/test_results/uav_mission/test_mission_executor_node.gtest.xml: 23 tests, 0 errors, 0 failures, 0 skipped

Summary: 41 tests, 0 errors, 0 failures, 0 skipped'

check "12 one failing case is reported as one, not as Summary's two" 1 42 "$REAL_ONE_FAILURE"
check "13 a clean per-file run passes"                               0 41 "$REAL_CLEAN"
check "14 old failures under Testing/ do not condemn a clean run"    0 41 "$STALE_HISTORY_ONLY"

echo
if [ "$fails" -eq 0 ]; then
  echo "SELFTEST PASS (14/14)"
  exit 0
fi
echo "SELFTEST FAIL ($fails case(s))"
exit 1
