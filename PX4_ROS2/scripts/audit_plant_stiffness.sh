#!/bin/bash
# Open debt (memory.md S:7, red): the navigator rig runs kPlantGain 8.0 while PX4's
# position loop is K_p 0.95 -- 8.4x stiffer. The comment above kPlantGain CLAIMS the
# stiff-plant tests are protocol tests whose conclusions do not depend on tracking.
# That claim has never been measured. This script measures it.
#
# Method: run the target twice on the same binary source, once at 8.0 and once at
# 0.95, and diff WHICH TESTS PASS. A test that flips is a test whose verdict depends
# on flying an aircraft 8x better than the real one -- exactly what the debt asks for.
#
# R27-1: the run at 0.95 is SLOWER (tau 1.05 s vs 0.125 s), so a timeout is not by
# itself evidence about the claim. Failures are printed with their assertion text so
# each one can be classified by hand; the script draws no conclusion on its own.
#
# Usage: bash scripts/audit_plant_stiffness.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
CANON=/mnt/c/code/PX4_ROS2
TESTSRC=$WORKSPACE/src/uav_navigation/test/test_navigator_action_server_node.cpp
BIN=$WORKSPACE/build/uav_navigation/test_navigator_action_server_node
OUT=/tmp/plant_audit

pgrep -af '[c]olcon' && { echo "FATAL: a colcon build is running, install/ is not stable"; exit 2; }

mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

sync_and_build() {
  rm -rf "$WORKSPACE/src/uav_navigation"
  cp -r "$CANON/src/uav_navigation" "$WORKSPACE/src/"
  if [ -n "$1" ]; then
    sed -i "s#^constexpr double kPlantGain = 8.0;#constexpr double kPlantGain = $1;#" "$TESTSRC"
    grep -q "kPlantGain = $1;" "$TESTSRC" || { echo "FATAL: patch to $1 did not apply"; exit 2; }
  fi
  ( cd "$WORKSPACE" && colcon build --packages-select uav_navigation \
      --cmake-args -DCMAKE_BUILD_TYPE=Release >"$OUT/build.log" 2>&1 ) \
    || { echo "FATAL: build failed"; tail -20 "$OUT/build.log"; exit 2; }
  # Re-source: the binary loads uav_interfaces typesupport from install/ at run time.
  source "$WORKSPACE/install/setup.bash"
}

run_target() {
  local label=$1
  # Confirm the stimulus reached the binary, not just the source (R27-1: the
  # measure_image_budget lesson -- editing a file that the run does not read).
  local got
  got=$(grep -aoE 'kPlantGain[^0-9]*[0-9]+\.[0-9]+' "$TESTSRC" | head -1)
  echo "### $label -- source says: $got"
  ROS_DOMAIN_ID=92 "$BIN" 2>&1 | tee "$OUT/$label.log" \
    | grep -E '^\[  (PASSED|FAILED)|^\[ *OK|^\[  FAILED' >/dev/null
  grep -E '^\[       OK \]|^\[  FAILED  \]' "$OUT/$label.log" \
    | sed -E 's/^\[ *(OK|FAILED) *\] /\1 /; s/ \([0-9]+ ms\)$//' \
    | grep -vE '^FAILED $' | sort -u >"$OUT/$label.status"
  local ok fail
  ok=$(grep -c '^OK ' "$OUT/$label.status")
  fail=$(grep -c '^FAILED ' "$OUT/$label.status")
  echo "    $ok ok, $fail failed"
  if [ $((ok + fail)) -lt 80 ]; then
    echo "  FAILED TO MEASURE: only $((ok + fail)) of 87 cases reported a verdict --"
    echo "                     the run died early, so any diff below is meaningless."
    sed -n '1,12p' "$OUT/$label.log" | sed 's/^/    /'
    exit 2
  fi
}

echo "========== A: baseline, kPlantGain 8.0 (as shipped) =========="
sync_and_build ""
run_target baseline

echo
echo "========== B: kPlantGain 0.95 (PX4's real position loop) =========="
sync_and_build "0.95"
run_target realistic

echo
echo "========== restoring the canonical tree into the build copy =========="
rm -rf "$WORKSPACE/src/uav_navigation"
cp -r "$CANON/src/uav_navigation" "$WORKSPACE/src/"
grep -q "kPlantGain = 8.0;" "$TESTSRC" && echo "  restored: build copy is back to 8.0"

echo
echo "########## TESTS WHOSE VERDICT DEPENDS ON THE STIFF PLANT ##########"
join -j 2 <(sed 's/^\(OK\|FAILED\) /\1 /' "$OUT/baseline.status" | awk '{print $1, $2}' | sort -k2) \
          <(awk '{print $1, $2}' "$OUT/realistic.status" | sort -k2) 2>/dev/null \
  | awk '$2 != $3 {printf "  %-70s  8.0=%-6s 0.95=%s\n", $1, $2, $3}'

echo
echo "########## FAILURE DETAIL AT 0.95 (classify each by hand) ##########"
grep -B 12 '^\[  FAILED  \]' "$OUT/realistic.log" \
  | grep -E 'Failure|Expected|Actual|Which is|^\[ RUN|clamped|error|m/s|leash' | head -80

echo
echo "logs: $OUT/{baseline,realistic}.log"
