#!/bin/bash
# G-M0: P9.5 sim assets - target_box motion, marker-at-hover, RTF, M5 regression.
# Usage: bash scripts/gate_gm0_mission_assets.sh
# Env: UAV_WORLD
# NOT RUN by the author - sim access is reserved for the verifier-runner.
# What each check means: src/uav_bringup/test/gm0_assets_gate.py
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
UAV_WORLD=${UAV_WORLD:-uav_arena}
UAV_ID=uav0
LOGDIR=$HOME/gate_logs
mkdir -p "$LOGDIR"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

PROBE=$WORKSPACE/src/uav_bringup/test/gm0_assets_gate.py
RTF_MIN=0.90
OVERALL=0   # 0 PASS, bumped to 1 FAIL or 2 FAILED-TO-MEASURE by any round

wait_px4() {
  local deadline=$((SECONDS + 420))
  until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
      | grep -q '/fmu/out/vehicle_odometry'; do
    if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
    sleep 10
  done
  echo "PX4 publishing after ${SECONDS}s"
}

bump() {
  # $1 = this round's verdict (0/1/2); OVERALL only ever gets worse.
  if [ "$1" -gt "$OVERALL" ]; then OVERALL=$1; fi
}

echo "############################################################"
echo "=== round A: target_box motion (model=uav0, world=$UAV_WORLD) ==="
echo "############################################################"
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
UAV_MODEL=uav0 UAV_WORLD=$UAV_WORLD bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -3
wait_px4

bash "$WORKSPACE/scripts/spawn_target_box.sh" "$UAV_WORLD" 30 30 0.2 0.0 gm0_target_box
python3 -u "$PROBE" target-box --world "$UAV_WORLD" --name gm0_target_box \
  --speed 0.2 --duration 45 2>&1 | tee "$LOGDIR/gm0_a_target_box.log" | tail -20
verdict_a=${PIPESTATUS[0]}
bump "$verdict_a"

bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

echo
echo "############################################################"
echo "=== round B: marker hover + RTF uav0_nav (world=$UAV_WORLD) ==="
echo "############################################################"
rm -f "$HOME/marker_debug"/marker_hover_*.png
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
UAV_MODEL=uav0_nav UAV_WORLD=$UAV_WORLD bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -3
wait_px4

bash "$WORKSPACE/scripts/spawn_marker.sh" "$UAV_WORLD" 7 0 0

setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$UAV_ID" \
  > "$LOGDIR/gm0_b_launch.log" 2>&1 < /dev/null &
sleep 20
# 5 backend + 6 localization + 3 navigation (advisors + navigator, both default true).
deadline=$((SECONDS + 90))
until [ "$(pgrep -c -f 'uav_px4_backend/|uav_localization/|uav_navigation/' 2>/dev/null)" = "14" ]; do
  if [ $SECONDS -gt $deadline ]; then
    echo "FATAL: expected 14 backend+localization+navigation processes, got:"
    pgrep -af 'uav_px4_backend/|uav_localization/|uav_navigation/'
    exit 1
  fi
  sleep 3
done
echo "14 processes up"

setsid nohup ros2 run uav_perception marker_detector_node \
  --ros-args -p marker_size_m:=0.5 -p camera:=down \
  > "$LOGDIR/gm0_b_marker_detector.log" 2>&1 < /dev/null &
sleep 8

# RTF, sampled concurrently with the hover flight (this is the "under full
# stack, flying" case, matching README S7's earlier uav0_nav measurement).
bash "$WORKSPACE/scripts/sample_rtf.sh" "$UAV_WORLD" 200 > "$LOGDIR/gm0_b_rtf.log" 2>&1 &
RTF_PID=$!

python3 -u "$PROBE" marker --uav-id "$UAV_ID" --marker-id 7 --height 2.5 --seconds 15 \
  2>&1 | tee "$LOGDIR/gm0_b_marker.log" | tail -30
verdict_b=${PIPESTATUS[0]}
bump "$verdict_b"

wait "$RTF_PID"
cat "$LOGDIR/gm0_b_rtf.log"
rtf_nav_mean=$(grep -oP 'mean \K[0-9.]+' "$LOGDIR/gm0_b_rtf.log")
if [ -z "$rtf_nav_mean" ]; then
  echo "RTF uav0_nav: FAILED TO MEASURE (no data)"
  bump 2
else
  echo "RTF uav0_nav mean = $rtf_nav_mean (threshold $RTF_MIN)"
  if awk -v m="$rtf_nav_mean" -v t="$RTF_MIN" 'BEGIN{exit !(m>=t)}'; then
    echo "  -> PASS"
  else
    echo "  -> FAIL (labelled per R27, not silently passed)"
    bump 1
  fi
fi

pkill -f 'marker_detector_node' 2>/dev/null
pkill -f 'ros2 launch uav_bringup' 2>/dev/null
pkill -f 'install/uav_px4_backend/lib' 2>/dev/null
pkill -f 'install/uav_localization/lib' 2>/dev/null
pkill -f 'install/uav_navigation/lib' 2>/dev/null
pkill -f 'install/uav_control_authority/lib' 2>/dev/null
pkill -f 'install/uav_safety/lib' 2>/dev/null
sleep 2
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

echo
echo "############################################################"
echo "=== round C: RTF uav0_track, idle full stack (world=$UAV_WORLD) ==="
echo "############################################################"
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
UAV_MODEL=uav0_track UAV_WORLD=$UAV_WORLD bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -3
wait_px4

setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$UAV_ID" \
  > "$LOGDIR/gm0_c_launch.log" 2>&1 < /dev/null &
sleep 20
# 5 backend + 6 localization + 3 navigation (advisors + navigator, both default true).
deadline=$((SECONDS + 90))
until [ "$(pgrep -c -f 'uav_px4_backend/|uav_localization/|uav_navigation/' 2>/dev/null)" = "14" ]; do
  if [ $SECONDS -gt $deadline ]; then
    echo "FATAL: expected 14 backend+localization+navigation processes, got:"
    pgrep -af 'uav_px4_backend/|uav_localization/|uav_navigation/'
    exit 1
  fi
  sleep 3
done
echo "14 processes up, idle settle"
sleep 10

bash "$WORKSPACE/scripts/sample_rtf.sh" "$UAV_WORLD" 200 | tee "$LOGDIR/gm0_c_rtf.log"
rtf_track_mean=$(grep -oP 'mean \K[0-9.]+' "$LOGDIR/gm0_c_rtf.log")
if [ -z "$rtf_track_mean" ]; then
  echo "RTF uav0_track: FAILED TO MEASURE (no data)"
  bump 2
else
  echo "RTF uav0_track mean = $rtf_track_mean (threshold $RTF_MIN)"
  if awk -v m="$rtf_track_mean" -v t="$RTF_MIN" 'BEGIN{exit !(m>=t)}'; then
    echo "  -> PASS"
  else
    echo "  -> FAIL (labelled per R27, not silently passed)"
    bump 1
  fi
fi

pkill -f 'ros2 launch uav_bringup' 2>/dev/null
pkill -f 'install/uav_px4_backend/lib' 2>/dev/null
pkill -f 'install/uav_localization/lib' 2>/dev/null
pkill -f 'install/uav_navigation/lib' 2>/dev/null
pkill -f 'install/uav_control_authority/lib' 2>/dev/null
pkill -f 'install/uav_safety/lib' 2>/dev/null
sleep 2
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

echo
echo "############################################################"
echo "=== round D: M5 regression after the launch-file edit (model=uav0) ==="
echo "############################################################"
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
UAV_MODEL=uav0 UAV_WORLD=$UAV_WORLD bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -3
wait_px4

setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$UAV_ID" \
  > "$LOGDIR/gm0_d_launch.log" 2>&1 < /dev/null &
sleep 20

ros2 run uav_bringup smoke_flight.py --flights 3 2>&1 | tee "$LOGDIR/gm0_d_smoke.log" | tail -40
verdict_d=${PIPESTATUS[0]}
if [ "$verdict_d" != "0" ]; then bump 1; fi

pkill -f 'ros2 launch uav_bringup' 2>/dev/null
sleep 2
bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1

echo
echo "############################################################"
echo "=== G-M0 summary ==="
echo "############################################################"
echo "  (a) target_box motion:      $([ "$verdict_a" = 0 ] && echo PASS || echo "FAIL/FTM ($verdict_a)")"
echo "  (b) marker at hover 2.5 m:  $([ "$verdict_b" = 0 ] && echo PASS || echo "FAIL/FTM ($verdict_b)")"
echo "  (c1) RTF uav0_nav mean:     ${rtf_nav_mean:-N/A} (threshold $RTF_MIN)"
echo "  (c2) RTF uav0_track mean:   ${rtf_track_mean:-N/A} (threshold $RTF_MIN)"
echo "  (d) M5 regression 3/3:      $([ "$verdict_d" = 0 ] && echo PASS || echo "FAIL ($verdict_d)")"
echo
echo "G-M0 exit status: $OVERALL   (0=PASS 1=FAIL 2=FAILED TO MEASURE)"
exit "$OVERALL"
