#!/bin/bash
# The symmetric question to verify_depth_halfres_cost.sh, and it must be asked
# for the same reason: on 2026-08-24 halving the DEPTH camera looked free by
# arithmetic (min_cluster_points still reaches 14 m) and then failed the product
# gate outright -- the box measured 0.578 x 0.298 m instead of 1.000 x 0.600.
# The arithmetic had answered the wrong question. So configuration B (down camera
# 640x480 -> 320x240) gets measured against its own product gate too, never
# reasoned about.
#
# Both resolutions in one session so the comparison carries its own control: the
# marker gate's numbers of record (2.5 m +0.036 m, 3.5 m +0.043 m) were taken
# 2026-08-14 on a much smaller stack.
#
# Tolerances that decide it (src/uav_perception/test/marker_accuracy.py):
#   DISTANCE_TOLERANCE_M 0.15 | TRACKING_TOLERANCE_M 0.10 | MIN_CONFIDENCE 0.5
#
# Usage: bash scripts/verify_down_halfres_cost.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
DOWN_CANON=/mnt/c/code/PX4_ROS2/src/uav_sim_gz/models/sensor_camera_down/model.sdf
DOWN_SDF=$WORKSPACE/install/uav_sim_gz/share/uav_sim_gz/models/sensor_camera_down/model.sdf

ps -eo comm | grep -qE '^ctest$|^colcon$' && { echo "FATAL: a build/test run holds install/"; exit 2; }
[ -f "$DOWN_SDF" ] || { echo "FATAL: $DOWN_SDF missing"; exit 2; }

restore() { cp "$DOWN_CANON" "$DOWN_SDF"; echo "installed down-camera SDF restored"; }
trap restore EXIT

pass=0
fail=0
unmeasured=0

run_case() {
  local res=$1
  cp "$DOWN_CANON" "$DOWN_SDF"
  if [ "$res" = "320" ]; then
    sed -i 's#<width>640</width>#<width>320</width>#; s#<height>480</height>#<height>240</height>#' "$DOWN_SDF"
    grep -q '<width>320</width>' "$DOWN_SDF" || { echo "FATAL: down patch did not apply"; exit 2; }
  fi
  echo
  echo "############################################################"
  echo "### down camera ${res}x"
  echo -n "###   installed down sdf: "
  grep -E '<width>|<height>' "$DOWN_SDF" | tr -d ' ' | tr '\n' ' '
  echo
  echo "############################################################"

  local out
  out=$(bash "$WORKSPACE/scripts/verify_marker_detector.sh" 2>&1)
  printf '%s\n' "$out" > "/tmp/downres_${res}.log"
  echo "$out" | grep -E 'n=|error|expected|tracking|confidence|RESULT|PASS|FAIL|exit' | tail -16

  if ! echo "$out" | grep -qE 'RESULT|exit'; then
    echo "  FAILED TO MEASURE: the gate did not run to completion"
    unmeasured=$((unmeasured + 1))
    return
  fi
  if echo "$out" | grep -q 'RESULT: PASS'; then
    echo "  ==> PASS"
    pass=$((pass + 1))
  else
    echo "  ==> FAIL (see /tmp/downres_${res}.log)"
    fail=$((fail + 1))
  fi
}

run_case 640
run_case 320

echo
echo "############ VERDICT ############"
echo "  pass $pass | fail $fail | failed-to-measure $unmeasured  (of 2)"
echo
echo "  Read the 640 row FIRST: if the control fails, the rig moved since"
echo "  2026-08-14 and the 320 row says nothing about configuration B."
echo "  B is affordable only if 320 passes AND its distance error stays inside"
echo "  0.15 m -- remember the residual is SYSTEMATIC (0.700% slope + 18.5 mm),"
echo "  so averaging more frames does not shrink it."
[ "$unmeasured" -eq 0 ] || exit 2
[ "$fail" -eq 0 ] || exit 1
