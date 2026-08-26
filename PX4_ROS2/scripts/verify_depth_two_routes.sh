#!/bin/bash
# Two routes for the same problem, after configuration D was killed by the product
# gate on 2026-08-24: depth feeds obstacle avoidance at 30% delivery (4.8/15.6 Hz,
# 594 frames lost per minute, worst-case 1.256 s blind), and halving its resolution
# fragmented one box into twelve clusters (measured 0.578 x 0.298 m instead of
# 1.000 x 0.600 -- wrong in the DANGEROUS direction).
#
# Route G -- the one the code itself points at. The clusterer works on a grid of
#   (width + stride - 1) / stride, and pixel_stride ships at 2. So today's grid is
#   640x480 / 2 = 320x240, and min_cluster_points = 30 plus neighbour connectivity
#   are tuned to THAT grid, not to the camera. Configuration D kept stride 2 on a
#   320x240 image -> a 160x120 grid, four times coarser than anything was tuned for.
#   Setting stride 1 on a 320x240 image restores EXACTLY the 320x240 grid, for the
#   same sample count and the same CPU, while the bridge carries a quarter of the
#   bytes. If the gate passes, this beats configuration B on every axis: the
#   obstacle stream is fixed AND the down camera is never touched.
#
# Route F -- keep 640x480, halve the source rate 15 -> 8 Hz. Nothing about the grid
#   changes, so fragmentation cannot occur; the open question is only whether the
#   bridge then delivers the stream cleanly.
#
# Every hypothesis runs beside a same-session 640x480/stride-2 control, because the
# numbers of record were taken 2026-08-16 on a much smaller stack.
#
# Usage: bash scripts/verify_depth_two_routes.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
DEPTH_CANON=/mnt/c/code/PX4_ROS2/src/uav_sim_gz/models/sensor_depth_front/model.sdf
DEPTH_SDF=$WORKSPACE/install/uav_sim_gz/share/uav_sim_gz/models/sensor_depth_front/model.sdf

ps -eo comm | grep -qE '^ctest$|^colcon$' && { echo "FATAL: a build/test run holds install/"; exit 2; }
[ -f "$DEPTH_SDF" ] || { echo "FATAL: $DEPTH_SDF missing"; exit 2; }

restore() { cp "$DEPTH_CANON" "$DEPTH_SDF"; echo "installed depth SDF restored"; }
trap restore EXIT

pass=0; fail=0; unmeasured=0
summary=""

run_case() {
  local tag=$1 res=$2 rate=$3 stride=$4
  cp "$DEPTH_CANON" "$DEPTH_SDF"
  if [ "$res" = "320" ]; then
    sed -i 's#<width>640</width>#<width>320</width>#; s#<height>480</height>#<height>240</height>#' "$DEPTH_SDF"
    grep -q '<width>320</width>' "$DEPTH_SDF" || { echo "FATAL: resolution patch failed"; exit 2; }
  fi
  if [ "$rate" != "15" ]; then
    sed -i "s#<update_rate>15</update_rate>#<update_rate>${rate}</update_rate>#" "$DEPTH_SDF"
    grep -q "<update_rate>${rate}</update_rate>" "$DEPTH_SDF" || { echo "FATAL: rate patch failed"; exit 2; }
  fi
  local grid_w=$(( (${res} + stride - 1) / stride ))
  echo
  echo "############################################################"
  echo "### $tag  --  depth ${res}px @ ${rate}Hz, pixel_stride=${stride}"
  echo "###   clusterer grid width: ${grid_w}  (shipped baseline is 320)"
  echo -n "###   installed depth sdf: "
  grep -E '<width>|<height>|<update_rate>' "$DEPTH_SDF" | tr -d ' ' | tr '\n' ' '
  echo
  echo "############################################################"

  local out
  out=$(PIXEL_STRIDE="$stride" bash "$WORKSPACE/scripts/verify_obstacle_extractor.sh" 2>&1)
  printf '%s\n' "$out" > "/tmp/tworoutes_${tag}.log"
  echo "$out" | grep -E 'obstacles/frame|n=|distance |size\.|RESULT|exit status' | tail -12

  if ! echo "$out" | grep -q 'exit status'; then
    echo "  ==> FAILED TO MEASURE (gate did not finish)"
    unmeasured=$((unmeasured + 1)); summary="$summary\n  $tag: FAILED TO MEASURE"; return
  fi
  local perframe
  perframe=$(echo "$out" | grep -oE 'obstacles/frame min=[0-9]+' | grep -oE '[0-9]+$' | head -1)
  if echo "$out" | grep -q 'P5.4 exit status: 0'; then
    echo "  ==> PASS   (obstacles/frame=${perframe:-?})"
    pass=$((pass + 1)); summary="$summary\n  $tag: PASS   obstacles/frame=${perframe:-?}"
  else
    echo "  ==> FAIL   (obstacles/frame=${perframe:-?})"
    fail=$((fail + 1)); summary="$summary\n  $tag: FAIL   obstacles/frame=${perframe:-?}"
  fi
}

run_case CONTROL 640 15 2   # the shipped configuration, this session
run_case G       320 15 1   # same clusterer grid, a quarter of the bytes
run_case F       640  8 2   # same grid and resolution, half the source rate

echo
echo "############ VERDICT ############"
printf '%b\n' "$summary"
echo "  pass $pass | fail $fail | failed-to-measure $unmeasured  (of 3)"
echo
echo "  obstacles/frame is the tell: the CONTROL finds 3. Configuration D found 12"
echo "  because its grid was 160x120. If G also finds 3, the fragmentation was the"
echo "  GRID, not the camera -- and G is then strictly better than configuration B."
[ "$unmeasured" -eq 0 ] || exit 2
[ "$fail" -eq 0 ] || exit 1
