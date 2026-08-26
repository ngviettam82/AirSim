#!/bin/bash
# V1 (P8.6 verifier brief): boot-time-only proof that B2 (battery NaN
# default) + B5 (armed && OFFBOARD && command_fresh gates the 4 LATCHING
# codes) hold on the wire. Full sim.launch.py default stack (safety +
# safety_enforcement both true), but the stack is NEVER armed and NEVER
# sent a goal -- see src/uav_safety/test/boot_no_latch_gate.py.
# Usage: bash scripts/verify_boot_no_latch.sh
# Env: UAV_MODEL (default uav0_nav) UAV_WORLD (default uav_arena) UAV_ID (default uav0)
# NOT RUN by the author - sim access is reserved for the verifier.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
export UAV_MODEL=${UAV_MODEL:-uav0_nav}
export UAV_WORLD=${UAV_WORLD:-uav_arena}
UAV_ID=${UAV_ID:-uav0}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
source "$WORKSPACE/scripts/gate_common.sh"

PROBE=$WORKSPACE/src/uav_safety/test/boot_no_latch_gate.py
[ -f "$PROBE" ] || { echo "FATAL: $PROBE missing"; exit 1; }

# Same teardown discipline as verify_obstacle_hold.sh (G-S3-HOLD): a
# leftover process from a previous run serves the OLD binary/state and
# silently poisons this one.
teardown() {
  pkill -f 'ros2 launch uav_bringup' 2>/dev/null
  pkill -f 'boot_no_latch_gate.py' 2>/dev/null
  sleep 3
  bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
  sleep 2
  pkill -9 -f "$WORKSPACE/install/.*/lib/.*_node" 2>/dev/null
  sleep 1

  local orphans
  orphans=$(pgrep -f 'safety_supervisor_node|control_authority_manager_node|vio_adapter_node|localization_mux_node|gz sim|px4' | wc -l)
  if [ "$orphans" -gt 0 ]; then
    echo "WARNING: $orphans orphan process(es) still alive after teardown -- force killing:"
    pgrep -af 'safety_supervisor_node|control_authority_manager_node|vio_adapter_node|localization_mux_node|gz sim|px4'
    pkill -9 -f 'safety_supervisor_node|control_authority_manager_node|vio_adapter_node|localization_mux_node' 2>/dev/null
    sleep 1
    orphans=$(pgrep -f 'safety_supervisor_node|control_authority_manager_node|vio_adapter_node|localization_mux_node|gz sim|px4' | wc -l)
    if [ "$orphans" -gt 0 ]; then
      echo "FATAL: $orphans orphan(s) survived SIGKILL -- refusing to continue with a polluted env:"
      pgrep -af 'safety_supervisor_node|control_authority_manager_node|vio_adapter_node|localization_mux_node|gz sim|px4'
      return 1
    fi
  fi
  ros2 daemon stop > /dev/null 2>&1
  return 0
}

teardown || { echo "FATAL: teardown could not reach a clean env before start"; exit 1; }

echo "=== 1/5 simulator (model=$UAV_MODEL world=$UAV_WORLD) ==="
bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -3

deadline=$((SECONDS + 420))
until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
    | grep -q '/fmu/out/vehicle_odometry'; do
  if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; exit 1; fi
  sleep 10
done
echo "PX4 publishing after ${SECONDS}s"

echo
echo "=== 2/5 autonomy stack (sim.launch.py -- ALL DEFAULTS: safety/safety_enforcement true, perception false) ==="
echo "NOT arming, NOT sending any goal -- boot-only probe."
BRINGUP_LOG=$HOME/boot_no_latch_bringup.log
setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$UAV_ID" \
  > "$BRINGUP_LOG" 2>&1 < /dev/null &
sleep 30
echo "process started count: $(grep -c 'process started' "$BRINGUP_LOG")"
grep -q "safety supervisor ready" "$BRINGUP_LOG" || {
  echo "FATAL: safety_supervisor_node never came up"; tail -20 "$BRINGUP_LOG"; teardown; exit 1;
}
grep "safety supervisor ready" "$BRINGUP_LOG"

echo
echo "=== 3/5 R27 dieu 1: confirm ZERO publisher on obstacle_map_local BEFORE the probe injects (step 4) ==="
OBSTACLE_TOPIC="/uav/$UAV_ID/world/obstacle_map_local"
assert_single_writer "$OBSTACLE_TOPIC" 0
writer_rc=$?
if [ "$writer_rc" != "0" ]; then
  teardown
  exit "$writer_rc"
fi
echo "confirmed: 0 publishers on $OBSTACLE_TOPIC -- the probe below will be the ONLY source"

echo
echo "=== 4/5 gate: 30s clean-boot window + R27-3 obstacle counter-check (no arm, no goal) ==="
python3 -u "$PROBE" --uav-id "$UAV_ID" 2>&1 | tail -300
verdict=${PIPESTATUS[0]}

echo
echo "real-time factor during the run:"
bash "$WORKSPACE/scripts/sample_rtf.sh" "$UAV_WORLD" 40 | sed 's/^/  /'

echo
echo "=== 5/5 teardown ==="
teardown

echo
echo "V1 boot-no-latch exit status: $verdict (0=PASS, 1=FAIL, 2=FAILED TO MEASURE)"
exit "$verdict"
