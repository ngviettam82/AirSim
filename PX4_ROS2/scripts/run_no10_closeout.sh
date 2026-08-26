#!/bin/bash
# Debt #10 close-out, steps 2 and 3 of the three the campaign owes before the
# large-samples DDS profile may be wired into start_sim.sh:
#
#   1. gate R0  -- does the profile harm the PX4 link?   (scripts/gate_r0_dds_profile.sh)
#   2. the two PRODUCT gates re-run under the profile     (this script)
#   3. M5 regression 3/3 under the profile, mandatory R14 (this script)
#
# Step 1 is a PRECONDITION, not a sibling: running the product gates under a
# transport that has not been cleared would produce numbers nobody may act on. This
# script therefore refuses to start unless gate R0's own evidence says PASSED. It
# re-adjudicates that evidence itself rather than trusting a remembered outcome --
# the project has been burned by numbers copied forward instead of derived (R22 S3).
#
# Coverage note, deliberate: the three runs below exercise three DIFFERENT airframes
# under the profile -- uav0_full (obstacle gate), uav0_nav (marker gate) and uav0 (M5).
# Wiring the profile into start_sim.sh affects every model, so every model gets flown.
#
# M5 also closes the one direction gate R0 cannot see. R0 measures /fmu/out/* (PX4 to
# ROS). The offboard setpoint stream runs the other way, /fmu/in/*, and a flight that
# completes arm-takeoff-goto-land three times is proof that direction survived: PX4
# drops out of offboard the moment the stream falls under 2 Hz.
#
# Every path is inside this file. stop_sim.sh derives its kill patterns from the built
# packages (R34) and runs pkill -f "<pkg>/", so a command line carrying such a path
# makes the caller kill itself (measured: SIGHUP, exit 15).
#
# Usage: bash scripts/run_no10_closeout.sh
# Env:   SKIP_R0_CHECK=1 only if gate R0 was cleared by other means -- say so out loud.
set -o pipefail

CANON=/mnt/c/code/PX4_ROS2
WORKSPACE=$HOME/PX4_ROS2
PROFILE=$CANON/src/uav_bringup/config/fastdds_large_samples.xml
R0_OUT=$WORKSPACE/gate_logs/r0
OUT=$WORKSPACE/gate_logs/no10
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

if pgrep -x colcon >/dev/null || pgrep -x ctest >/dev/null; then
  echo "FATAL: a build/test run holds install/"; exit 2
fi
[ -f "$PROFILE" ] || { echo "FATAL: profile missing"; exit 2; }

if [ "${SKIP_R0_CHECK:-0}" != "1" ]; then
  echo "=== precondition: re-adjudicating gate R0 from its own evidence ==="
  if [ ! -s "$R0_OUT/control.txt" ] || [ ! -s "$R0_OUT/profile.txt" ]; then
    echo "FATAL: gate R0 evidence missing -- run scripts/gate_r0_dds_profile.sh first."
    exit 2
  fi
  if ! python3 "$CANON/scripts/gate_r0_verdict.py" "$R0_OUT/control.txt" "$R0_OUT/profile.txt"; then
    echo "FATAL: gate R0 does not pass on its own evidence. Nothing below may run."
    exit 2
  fi
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE"
echo
echo "profile in force for every run below"

fails=""

run_step() {
  local name=$1; shift
  echo
  echo "############################################################"
  echo "### $name"
  echo "############################################################"
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
  sleep 3
  find /dev/shm -maxdepth 1 -name 'fastrtps_*' -delete 2>/dev/null
  "$@" > "$OUT/$name.log" 2>&1
  local rc=$?
  tail -25 "$OUT/$name.log"
  if [ $rc -ne 0 ]; then
    echo "  --> $name EXIT $rc"
    fails="$fails $name"
  else
    echo "  --> $name exit 0"
  fi
}

run_step obstacle_extractor bash "$WORKSPACE/scripts/verify_obstacle_extractor.sh"
run_step marker_detector    bash "$WORKSPACE/scripts/verify_marker_detector.sh"
run_step m5_regression      bash "$WORKSPACE/scripts/run_m5_regression.sh"

bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1

echo
echo "############ DEBT #10 CLOSE-OUT SUMMARY ############"
if [ -n "$fails" ]; then
  echo "  FAILED:$fails"
  echo "  RESULT: do NOT wire the profile into start_sim.sh."
  echo "  (logs: $OUT)"
  exit 1
fi
echo "  all three steps exit 0 -- read the numbers above before believing it:"
echo "    obstacle gate must still report its box geometry within tolerance,"
echo "    marker gate must still detect and pose the marker,"
echo "    M5 must report RESULT: PASS with 3 flights and zero violations."
echo "  (logs: $OUT)"
