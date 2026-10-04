#!/bin/bash
# P5 dynamic gate D3: "M5 regression still PASSES 3/3 with every P5 node running."
# Acceptance, from .claude/plan/P5-perception.md Sec 3: PASS 3/3 AND RTF >= 0.95.
#
# WHY IT IS RUNNABLE NOW AND WAS NOT BEFORE. D3 has been blocked since 2026-08-16 by
# one thing: the RTF of uav0_full. That model carries three camera streams, and until
# debt #10 was closed on 2026-08-24 those streams were being fragmented over UDP and
# losing whole frames, which cost both delivery AND real-time factor (measured
# 0.889 -> 0.963 on the transport fix alone, no sensor touched). This gate re-asks the
# question under the fixed transport.
#
# 🔴 IT MAY STILL FAIL, AND THAT IS A RESULT, NOT AN ERROR. Under the full autonomy
# stack the R0 gate measured uav0_full at RTF 0.791 with three `ros2 topic hz` probes
# and an RTF sampler also running; the image-budget campaign measured 0.963 with the
# stack but no probes. The 0.95 bar sits inside that spread, so this gate is genuinely
# undecided until run. Do NOT lower the bar to make it green -- if it fails, the honest
# outcome is "D3 still blocked, RTF is the reason, here is the number".
#
# WHAT IS DIFFERENT FROM plain run_m5_regression.sh:
#   - model uav0_full (cameras) instead of uav0
#   - perception:=true, so every P5 node is in the loop, which is the whole point
#   - RTF sampled DURING the flight, not after
# Safety is left enabled on purpose: if a live obstacle map makes safety HOLD the
# aircraft mid-regression, that is exactly the interaction D3 exists to expose.
#
# Every path is inside this file (stop_sim.sh kill patterns -- ops-playbook S22).
#
# Usage: bash scripts/run_d3_p5_dynamic.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
OUT=$WORKSPACE/gate_logs/d3
mkdir -p "$OUT"
# uav0_full_rgbd, not uav0_full (S10, 2026-08-26). Same aircraft, same four sensor
# STREAMS, but the front RGB and depth are one rgbd_camera instead of two, so three
# render passes hit each tick instead of four. That is the whole difference, and it is
# the one that mattered: measured on uav_arena, 90 s per branch, 878 samples, the
# fraction of the window below 0.95x RTF went 17.5% / 27.2% (sim alone / with the stack)
# to 0.0% / 0.0%, RTF mean 1.000. The cost is the front RGB clip dropping 100 m -> 19.1 m,
# which no consumer in this simulation can use: marker detection runs on the DOWN camera
# (far 100, untouched), obstacle extraction runs on front DEPTH (already 19.1 m), and the
# only reader of front RGB pixels is a HOG people-detector in a world with no people.
# Contract-checked before switching, both on this model: camera health OK on all three
# streams, 0 lost frames, and ERROR on all three when the bridges are killed; obstacle
# geometry at 3 m within 0.003 m on distance, width and height.
export UAV_MODEL=${UAV_MODEL:-uav0_full_rgbd}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
FLIGHTS=${FLIGHTS:-3}
RTF_FLOOR=0.95

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

if pgrep -x colcon >/dev/null || pgrep -x ctest >/dev/null; then
  echo "FATAL: a build/test run holds install/"; exit 2
fi

STACK_PID=""
cleanup() {
  [ -n "$STACK_PID" ] && { kill "$STACK_PID" 2>/dev/null; wait "$STACK_PID" 2>/dev/null; }
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
}
trap cleanup EXIT

echo "=== 1/5 simulator (model=$UAV_MODEL) ==="
bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
sleep 3
find /dev/shm -maxdepth 1 -name 'fastrtps_*' -delete 2>/dev/null
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -3

deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; do
  [ $SECONDS -gt $deadline ] && { echo "FATAL: PX4 never published"; exit 2; }
  sleep 10
done
echo "PX4 up after ${SECONDS}s"

echo
echo "=== 2/5 autonomy stack WITH perception (this is what makes it D3) ==="
ros2 launch uav_bringup sim.launch.py perception:=true > "$OUT/stack.log" 2>&1 &
STACK_PID=$!
sleep 40
nodes=$(timeout 60 ros2 node list --no-daemon --spin-time 5 2>/dev/null \
        | grep -cE 'camera_health_node|marker_detector_node|object_detector_node|target_tracker_node|obstacle_extractor_node')
echo "  perception nodes up: ${nodes:-0}/5"
if [ "${nodes:-0}" -lt 5 ]; then
  echo "  FAILED TO MEASURE: D3 asks for EVERY P5 node; only ${nodes:-0} are up."
  exit 2
fi

echo
echo "=== 3/5 transport actually in force (debt #10 is the premise of this gate) ==="
seg=$(find /dev/shm -maxdepth 1 -name 'fastrtps_*' ! -name '*_el' -printf '%s\n' 2>/dev/null | sort -n | tail -1)
echo "  largest shm segment: ${seg:-none} B"
if [ "${seg:-0}" -lt 16777216 ]; then
  echo "  FAILED TO MEASURE: the large-samples profile is NOT in force, so this run"
  echo "                     does not answer the question D3 is being re-asked under."
  exit 2
fi

echo
echo "=== 4/5 regression flight with RTF sampled DURING it ==="
bash "$WORKSPACE/scripts/sample_rtf.sh" "$UAV_WORLD" 200 > "$OUT/rtf.log" 2>&1 &
rtf_pid=$!
# A gate that accuses has to keep what it accused on. Two D3 runs on 2026-08-26 failed
# with "safety raised CAMERA_STREAM_UNHEALTHY and nobody has written down what it means",
# and the evidence for WHY was gone the moment the run ended.
timeout 240 ros2 topic echo --no-daemon /uav/uav0/state/camera_health \
  > "$OUT/camera_health.log" 2>&1 &
cam_pid=$!
# The summary carries a level and a sentence; the per-stream numbers that say WHICH
# stream degraded and by HOW MUCH live on the detailed array.
timeout 240 ros2 topic echo --no-daemon /uav/uav0/diagnostics/perception \
  > "$OUT/perception_diag.log" 2>&1 &
diag_pid=$!
python3 "$WORKSPACE/install/uav_bringup/lib/uav_bringup/smoke_flight.py" \
  --flights "$FLIGHTS" 2>&1 | tee "$OUT/flight.log" | tail -20
flight_rc=${PIPESTATUS[0]}
wait "$rtf_pid" 2>/dev/null
kill "$cam_pid" "$diag_pid" 2>/dev/null
wait "$cam_pid" "$diag_pid" 2>/dev/null
echo
# The autopilot's own reason for refusing, kept where the verdict is. Three runs on
# 2026-08-26 failed with an identical "arm refused for 124 s over 12 attempt(s)" and the
# reason -- "Preflight Fail: height estimate not stable" -- was only in /tmp/px4.log,
# which the next run overwrites. A gate that accuses has to keep what it accused on.
cp /tmp/px4.log "$OUT/px4.log" 2>/dev/null

echo "  RTF during the flight: $(cat "$OUT/rtf.log")"

echo
echo "=== 5/5 verdict ==="
# WHICH RTF THIS GATE SCORES, and why it changed on 2026-08-26 (owner signed).
# The bar is unchanged at 0.95. What changed is the quantity read. Until today this
# line took the arithmetic mean of Gazebo's `real_time_factor` field -- a ONE-STEP
# ratio that P12.5 had already measured disagreeing with the message carrying it (one
# sample said 0.151 while sim_time/real_time of that same message said 0.748). Two D3
# runs on the same day settled it: over one 20.4 s window the field ranked uav0_full
# (0.931) ABOVE uav0_full_rgbd (0.907) while the clocks ranked them the other way
# (0.960 vs 0.973), and the field reported max 4.948, which no window can be. A number
# that cannot even order two configurations the same way as the thing it claims to
# measure is not fit to gate on -- CLAUDE.md section 5, read what actually went out.
# Both numbers stay in the log so nothing is hidden by this change.
rtf=$(grep -oE 'window RTF [0-9.]+' "$OUT/rtf.log" | grep -oE '[0-9.]+$')
rtf_field=$(grep -oE 'mean [0-9.]+' "$OUT/rtf.log" | grep -oE '[0-9.]+$')
fail=0
if [ "$flight_rc" -ne 0 ]; then
  echo "  FAIL: smoke flight exit $flight_rc (D3 needs PASS 3/3)"; fail=1
else
  echo "  PASS: smoke flight 3/3"
fi
if [ -z "$rtf" ]; then
  echo "  FAILED TO MEASURE: no window RTF (sample_rtf.sh printed no clock line)"; exit 2
fi
if awk -v r="$rtf" -v f="$RTF_FLOOR" 'BEGIN{exit !(r >= f)}'; then
  echo "  PASS: window RTF $rtf >= $RTF_FLOOR  (field mean was ${rtf_field:-n/a})"
else
  echo "  FAIL: window RTF $rtf < $RTF_FLOOR -- D3 stays blocked, and RTF is still the reason."
  echo "        (field mean was ${rtf_field:-n/a}; both are in $OUT/rtf.log)"
  fail=1
fi
echo
if [ "$fail" -ne 0 ]; then
  echo "  RESULT: D3 NOT MET. Report the number; do not move the bar."
  echo "  (evidence: $OUT)"
  exit 1
fi
echo "  RESULT: D3 MET -- M5 3/3 with all five P5 nodes live, RTF $rtf."
echo "  (evidence: $OUT)"
