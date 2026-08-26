#!/bin/bash
# Condition attached to configuration D (depth 640x480 -> 320x240, 2026-08-24):
# D fixes the obstacle stream completely (30% -> 97% delivered, worst-case
# staleness 1256 ms -> 68 ms) at no cost to the down camera. The price is half the
# ANGULAR resolution, and that price must be measured, not assumed.
#
# Two boxes, each at both resolutions, so every comparison has its control in the
# same session (the shipped 2026-08-16 numbers were taken on a smaller stack):
#   * the gate's own box  0.4 x 1.0 x 0.6 m @ 3.0 m -- proves no regression where
#     P5.5 already passed
#   * a smaller, farther box 0.4 x 0.25 x 0.5 m @ 6.0 m -- 207 points at 320x240
#     by arithmetic, so it is still detectable and the question becomes ACCURACY,
#     which is what actually degrades with fewer pixels
#
# Detection range is NOT the risk here and the arithmetic says so: min_cluster_points
# = 30 is a PIXEL count, so at 320x240 a 10 cm post is still found to 14.1 m and the
# far clip is 19.1 m. Only wire-thin things (2 cm) lose materially (12.6 -> 6.3 m).
#
# Usage: bash scripts/verify_depth_halfres_cost.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
DEPTH_CANON=/mnt/c/code/PX4_ROS2/src/uav_sim_gz/models/sensor_depth_front/model.sdf
DEPTH_SDF=$WORKSPACE/install/uav_sim_gz/share/uav_sim_gz/models/sensor_depth_front/model.sdf

ps -eo comm | grep -qE '^ctest$|^colcon$' && { echo "FATAL: a build/test run holds install/"; exit 2; }
[ -f "$DEPTH_SDF" ] || { echo "FATAL: $DEPTH_SDF missing"; exit 2; }

restore() { cp "$DEPTH_CANON" "$DEPTH_SDF"; echo "installed depth SDF restored"; }
trap restore EXIT

pass=0
fail=0
unmeasured=0

run_case() {
  local res=$1 label=$2
  shift 2
  cp "$DEPTH_CANON" "$DEPTH_SDF"
  if [ "$res" = "320" ]; then
    sed -i 's#<width>640</width>#<width>320</width>#; s#<height>480</height>#<height>240</height>#' "$DEPTH_SDF"
    grep -q '<width>320</width>' "$DEPTH_SDF" || { echo "FATAL: depth patch did not apply"; exit 2; }
  fi
  echo
  echo "############################################################"
  echo "### ${res}x  --  $label"
  echo -n "###   installed depth sdf: "
  grep -E '<width>|<height>' "$DEPTH_SDF" | tr -d ' ' | tr '\n' ' '
  echo
  echo "############################################################"

  local out
  out=$(env "$@" bash "$WORKSPACE/scripts/verify_obstacle_extractor.sh" 2>&1)
  printf '%s\n' "$out" > "/tmp/halfres_${res}_${label// /_}.log"
  echo "$out" | grep -E 'error|distance|size|yaw|samples|n=|PASS|FAIL|exit status' | tail -16

  # R27-1: the stimulus must have reached the simulator. The gate itself never
  # looks at the SDF, so without this a patched file that Gazebo ignored would
  # read as "half resolution costs nothing" -- the empty-experiment trap again.
  if ! echo "$out" | grep -q 'exit status'; then
    echo "  FAILED TO MEASURE: the gate did not run to completion"
    unmeasured=$((unmeasured + 1))
    return
  fi
  if echo "$out" | grep -q 'P5.4 exit status: 0'; then
    echo "  ==> PASS"
    pass=$((pass + 1))
  else
    echo "  ==> FAIL (see /tmp/halfres_${res}_${label// /_}.log)"
    fail=$((fail + 1))
  fi
}

run_case 640 "gate box 3m"  STANDOFF=3.0 BOX_THICKNESS=0.4 BOX_WIDTH=1.0  BOX_HEIGHT=0.6
run_case 320 "gate box 3m"  STANDOFF=3.0 BOX_THICKNESS=0.4 BOX_WIDTH=1.0  BOX_HEIGHT=0.6
run_case 640 "small box 6m" STANDOFF=6.0 BOX_THICKNESS=0.4 BOX_WIDTH=0.25 BOX_HEIGHT=0.5
run_case 320 "small box 6m" STANDOFF=6.0 BOX_THICKNESS=0.4 BOX_WIDTH=0.25 BOX_HEIGHT=0.5

echo
echo "############ VERDICT ############"
echo "  pass $pass | fail $fail | failed-to-measure $unmeasured  (of 4)"
echo
echo "  Read the 640 rows FIRST. A 640 row that fails means the rig, not the"
echo "  resolution -- the 320 row beside it then says nothing about D."
echo "  D is affordable only if each 320 row passes AND its errors stay in the"
echo "  same band as the 640 row above it (tolerances: distance 0.15 m,"
echo "  size 0.15 m, yaw 5 deg)."
[ "$unmeasured" -eq 0 ] || exit 2
[ "$fail" -eq 0 ] || exit 1
