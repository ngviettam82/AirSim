#!/bin/bash
# Debt #10, final step: prove the profile is wired in for real, and that fixing the
# transport actually clears the fault the go/no-go light was deliberately left red on.
#
# TWO CLAIMS, MEASURED SEPARATELY.
#
# (1) WIRING. Nothing is exported by hand here. If start_sim.sh and sim.launch.py did
#     not pick the profile up on their own, the largest /dev/shm/fastrtps_* segment
#     stays at the stock 549 408 B and this fails. That distinction matters because the
#     export in start_sim.sh lives in a CHILD shell and reaches only the agent, Gazebo
#     and the bridge -- the stack is covered by sim.launch.py's own action, and the two
#     halves have to be checked together, not assumed.
#
# (2) THE CONSEQUENCE THE WAIVER TABLE PREDICTED. preflight_waivers.yaml deliberately
#     does NOT load the three camera rows, and says why in its own header: with
#     perception:=true the light "correctly reads NO_GO until debt #10 is fixed;
#     waiving them would be exactly the 'never widen a gate' this design forbids."
#     So if the transport fix is real, the light must go GO **with the waiver table
#     untouched**. That is the strongest end-to-end evidence available for this debt,
#     and it is evidence nobody can fake by editing a threshold: the only way to turn
#     this light green is to actually deliver the frames.
#
#     A red light here does NOT license editing the waiver table. It means debt #10 is
#     not closed.
#
# Every path is inside this file (stop_sim.sh kill patterns -- ops-playbook S22).
#
# Usage: bash scripts/verify_no10_wiring.sh
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
OUT=$WORKSPACE/gate_logs/no10
mkdir -p "$OUT"
export UAV_MODEL=uav0_full
export UAV_WORLD=uav_arena
ASKED_SEGMENT=16777216

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

if pgrep -x colcon >/dev/null || pgrep -x ctest >/dev/null; then
  echo "FATAL: a build/test run holds install/"; exit 2
fi

# Deliberately NOT exported: the point is whether the wiring does it by itself.
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset UAV_DDS_PROFILE

STACK_PID=""
cleanup() {
  [ -n "$STACK_PID" ] && { kill "$STACK_PID" 2>/dev/null; wait "$STACK_PID" 2>/dev/null; }
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
}
trap cleanup EXIT

bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
ros2 daemon stop >/dev/null 2>&1
sleep 3
find /dev/shm -maxdepth 1 -name 'fastrtps_*' -delete 2>/dev/null

echo "=== 1/4 simulator (no manual export) ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -3
deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; do
  [ $SECONDS -gt $deadline ] && { echo "FATAL: PX4 never published"; exit 2; }
  sleep 10
done
echo "PX4 up after ${SECONDS}s"

echo
echo "=== 2/4 autonomy stack with perception ==="
ros2 launch uav_bringup sim.launch.py perception:=true > "$OUT/wiring_stack.log" 2>&1 &
STACK_PID=$!
sleep 45

echo
echo "=== 3/4 CLAIM 1: is the profile actually in force? ==="
seg=$(find /dev/shm -maxdepth 1 -name 'fastrtps_*' ! -name '*_el' -printf '%s\n' 2>/dev/null | sort -n | tail -1)
echo "  largest shm segment: ${seg:-none} B (stock 549408, profile asks $ASKED_SEGMENT)"
if [ "${seg:-0}" -lt "$ASKED_SEGMENT" ]; then
  echo "  FAIL: the wiring did not take -- the profile is NOT in force."
  exit 1
fi
echo "  PASS: wiring confirmed without any manual export."

echo
echo "=== 4/4 CLAIM 2: go/no-go light with perception:=true, waiver table UNTOUCHED ==="
echo "  (waiting for the camera streams to settle before reading the light)"
sleep 30
bash "$WORKSPACE/scripts/preflight_light.sh" --uav-id uav0 2>&1 | tee "$OUT/preflight_light.log"
light_rc=${PIPESTATUS[0]}
echo "  preflight_light exit status: $light_rc"

echo
echo "  camera_health as the stack itself reports it (per-stream rates -- this is the"
echo "  direct product evidence that the frames arrive, not an inference from the light):"
# --no-daemon is mandatory. Without it the first run of this check reported
# "topic does not appear to be published yet / could not determine the type" while
# camera_health_node was demonstrably up and the go/no-go light read GO with no
# blocking item -- a stale daemon that cannot resolve the type looks exactly like an
# absent topic. Never let a CLI artifact stand as evidence about the product.
timeout 40 ros2 topic echo /uav/uav0/state/camera_health --once --no-daemon 2>&1 | head -40 \
  | tee "$OUT/camera_health.log"

echo
if [ "$light_rc" -eq 0 ]; then
  echo "  RESULT: GO -- debt #10 closed end to end. The light turned green because the"
  echo "          frames arrive, not because anything was waived."
else
  echo "  RESULT: not GO. Read $OUT/preflight_light.log for the worst item."
  echo "          Do NOT edit preflight_waivers.yaml to make this green."
fi
exit "$light_rc"
