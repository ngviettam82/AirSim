#!/bin/bash
# Full-workspace build + test with per-package "N case / M target" counts.
# Usage: bash scripts/verify_workspace.sh   (run from anywhere; uses ~/PX4_ROS2)
# Docs: .claude/rules/session-protocol.md S3 - headline numbers come from here.
set -e
cd "$HOME/PX4_ROS2"
CANON=/mnt/c/code/PX4_ROS2
source /opt/ros/humble/setup.bash
echo "=== BUILD ALL ==="
colcon build > /tmp/verify_build.log 2>&1 || { echo BUILD FAILED; tail -20 /tmp/verify_build.log; exit 1; }
tail -n 2 /tmp/verify_build.log
echo "=== TEST ALL ==="
source install/setup.bash
# SEQUENTIAL ON PURPOSE (project owner signed 2026-08-25). Most of this suite spins
# up live node graphs and measures TIME on them -- grace windows, stream rates, gaps.
# Running 12 packages at once on 16 cores does not test the product harder, it starves
# the fixtures' own stimulus threads: measured the same day, a fixture's authority
# publisher went unscheduled for 2.242 s against a 1.0 s staleness rule, and the node
# then paused the mission exactly as designed. Nine distinct fixtures across three
# packages had been hit that way, and patching four of them left the dirty rate at
# 50% either side -- the defect was the condition, not the fixtures.
# Sequential is the only setting with evidence behind it: measure_coverage.sh already
# runs `--executor sequential` and reported 1141 tests, 0 errors, 0 failures, 0 skipped.
# This narrows WHEN the claims hold, never WHAT they claim; nothing here was relaxed.
# UAV_TEST_EXECUTOR=parallel restores the old behaviour for quick local iteration --
# it is NOT valid for the S1 gate.
TEST_EXECUTOR="${UAV_TEST_EXECUTOR:-sequential}"
if [ "$TEST_EXECUTOR" = "sequential" ]; then
  colcon test --executor sequential > /tmp/verify_test.log 2>&1 || true
else
  echo "  (WARNING: parallel executor -- not valid evidence for G-SIM S1)"
  colcon test > /tmp/verify_test.log 2>&1 || true
fi
tail -n 2 /tmp/verify_test.log
echo "=== PER-PACKAGE CASE/TARGET ==="
# R34: derive the package list; a hand-kept list silently dropped uav_mission
# (2026-08-23) and uav_observability (same day) as they were added.
total_cases=0
total_targets=0
for pkg in $(find src -maxdepth 1 -mindepth 1 -type d -printf '%f\n' | sort); do
  # -m1: tests= appears on both <testsuites> root and <testsuite>
  cases=$(grep -h -m1 -o 'tests="[0-9]*"' build/$pkg/test_results/$pkg/*.xml 2>/dev/null \
    | grep -o '[0-9]*' | paste -sd+ | bc)
  targets=$(ls build/$pkg/test_results/$pkg/*.xml 2>/dev/null | wc -l)
  if [ "$targets" -eq 0 ]; then
    echo "$pkg: (no ctest target)"
    continue
  fi
  echo "$pkg: $cases case / $targets target"
  total_cases=$((total_cases + cases))
  total_targets=$((total_targets + targets))
done
echo "TOTAL: $total_cases case / $total_targets target"
echo "=== OVERALL (Summary mixes case+target - report per-package) ==="
# Keep the evidence in a file: every previous read of this ran inside a pipeline or a
# command substitution, and both throw away colcon's exit status. That, plus the
# `|| true` above, is why this script could not fail before 2026-08-25 (P12.0).
colcon test-result --all > /tmp/verify_test_result.txt 2>&1 || true
tail -n 2 /tmp/verify_test_result.txt
echo "=== FAILURES ==="
colcon test-result --all --verbose 2>/dev/null | grep -B 1 -A 6 -E "Failed|failure message" | head -n 40 || echo "(none)"
echo "=== VERDICT ==="
# The cross-check was a printed WARNING until P12.0; it is now fatal. Rules and the
# reasoning behind each live in workspace_verdict.py, exercised by
# scripts/selftest_workspace_verdict.sh (11 cases, both directions).
verdict_rc=0
python3 "$CANON/scripts/workspace_verdict.py" \
  /tmp/verify_test_result.txt "$((total_cases + total_targets))" || verdict_rc=$?

# One line per run, so G-SIM S1 can ask for three consecutive clean ones rather than
# trusting whichever run happened last. Appended, never rewritten.
HISTORY=$HOME/PX4_ROS2/gate_logs/sim_closeout/S1.history
mkdir -p "$(dirname "$HISTORY")"
if [ "$verdict_rc" -eq 0 ]; then
  echo "$(date +%s) PASS $total_cases case / $total_targets target" >> "$HISTORY"
else
  echo "$(date +%s) FAIL $total_cases case / $total_targets target" >> "$HISTORY"
fi
exit $verdict_rc
