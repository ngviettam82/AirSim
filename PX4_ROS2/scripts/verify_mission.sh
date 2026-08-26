#!/bin/bash
# P9.6-P9.9: 4-gate mission verification (G-M1 indoor_patrol, G-M2
# inspect_point, G-M3 follow_target, G-M4 event injections) + final M5
# regression (mission:=false). Each round is independently runnable.
# Usage: bash scripts/verify_mission.sh [round ...]
#   rounds: m1 m2 m3 m4a m4b m4b-control m4c m4d m5   (default: all, in order)
# What each check means: src/uav_mission/test/mission_gate.py
# Env: UAV_WORLD (default uav_arena; m1/m4a/m4b/m4d force their own world)
# NOT RUN by the author -- sim access is reserved for the verifier-runner.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
UAV_ID=uav0
LOGDIR=$HOME/gate_logs
mkdir -p "$LOGDIR"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
source "$WORKSPACE/scripts/gate_common.sh"

PROBE=$WORKSPACE/src/uav_mission/test/mission_gate.py
[ -f "$PROBE" ] || { echo "FATAL: $PROBE missing"; exit 1; }

OVERALL=0
bump() { if [ "$1" -gt "$OVERALL" ]; then OVERALL=$1; fi; }

# ---------------------------------------------------------------- teardown
teardown() {
  pkill -f 'ros2 launch uav_bringup' 2>/dev/null
  pkill -f 'mission_gate.py' 2>/dev/null
  sleep 3
  bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
  sleep 2
  pkill -9 -f "$WORKSPACE/install/.*/lib/.*_node" 2>/dev/null
  sleep 1
  local orphans
  orphans=$(pgrep -f 'mission_executor_node|safety_supervisor_node|control_authority_manager_node|vio_adapter_node|localization_mux_node|world_model_node|target_tracker_node|obstacle_extractor_node|marker_detector_node|object_detector_node|navigator_action_server_node|gz sim|px4' | wc -l)
  if [ "$orphans" -gt 0 ]; then
    echo "WARNING: $orphans orphan process(es) after teardown -- force killing:"
    pgrep -af 'mission_executor_node|safety_supervisor_node|control_authority_manager_node|vio_adapter_node|localization_mux_node|world_model_node|target_tracker_node|obstacle_extractor_node|marker_detector_node|object_detector_node|navigator_action_server_node'
    pkill -9 -f 'mission_executor_node|safety_supervisor_node|control_authority_manager_node|vio_adapter_node|localization_mux_node|world_model_node|target_tracker_node|obstacle_extractor_node|marker_detector_node|object_detector_node|navigator_action_server_node' 2>/dev/null
    sleep 1
    orphans=$(pgrep -f 'mission_executor_node|safety_supervisor_node|control_authority_manager_node|gz sim|px4' | wc -l)
    if [ "$orphans" -gt 0 ]; then
      echo "FATAL: $orphans orphan(s) survived SIGKILL -- refusing to continue polluted:"
      pgrep -af 'mission_executor_node|safety_supervisor_node|control_authority_manager_node|gz sim|px4'
      return 1
    fi
  fi
  ros2 daemon stop > /dev/null 2>&1
  return 0
}

wait_px4() {
  local deadline=$((SECONDS + 420))
  until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
      | grep -q '/fmu/out/vehicle_odometry'; do
    if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; return 1; fi
    sleep 10
  done
  echo "PX4 publishing after ${SECONDS}s"
}

# wait_nodes -- moved to scripts/gate_common.sh (sourced above), unchanged.

# PX4 SITL PERSISTS SIM_BAT_DRAIN/SIM_BAT_MIN_PCT across stop_sim.sh/
# start_sim.sh restarts (saved to build/px4_sitl_default/rootfs/eeprom/,
# found the hard way: G-M4.3's positive-control-adjacent run silently
# aborted on ABORTED_LOW_BATTERY because G-M4.2's drain injection from the
# PREVIOUS round's PX4 process was still on disk). Reset to PX4's own
# compiled-in defaults (battery_simulator_params.c: DRAIN=60, MIN_PCT=50.0)
# right after every PX4 boot, unconditionally -- cheap, and the ONE round
# that actually wants a drain (event-battery) sets its own value anyway.
reset_battery_params() {
  python3 -c "
from pymavlink import mavutil
import sys, time
conn = mavutil.mavlink_connection('udpin:0.0.0.0:14540')
conn.wait_heartbeat(timeout=15)
for name, val in [('SIM_BAT_DRAIN', 60.0), ('SIM_BAT_MIN_PCT', 50.0)]:
    conn.mav.param_set_send(conn.target_system, conn.target_component, name.encode(), val,
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    end = time.time() + 5
    ok = False
    while time.time() < end:
        m = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if m and m.param_id.strip('\\x00') == name:
            print('reset %s -> %s' % (name, m.param_value)); ok = True; break
    if not ok:
        print('WARNING: %s reset not acknowledged' % name)
" 2>&1
}

# start_stack MODEL WORLD PERCEPTION LOGNAME -- returns via echo'd log path.
start_stack() {
  local model="$1" world="$2" perception="$3" logname="$4"
  UAV_MODEL="$model" UAV_WORLD="$world" bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -3
  wait_px4 || return 1
  reset_battery_params

  local log="$LOGDIR/$logname"
  setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$UAV_ID" \
    perception:="$perception" mission:=true \
    > "$log" 2>&1 < /dev/null &
  sleep 25
  grep -q "mission_executor_node ready for" "$log" || {
    echo "FATAL: mission_executor_node never came up"; tail -30 "$log"; return 1;
  }
  echo "MISSION_LOG=$log"
}

# ============================================================ G-M1 patrol
round_m1() {
  echo "############################################################"
  echo "=== G-M1: indoor_patrol e2e (uav0_nav_indoor / uav_arena_indoor) ==="
  echo "############################################################"
  teardown || { bump 1; return; }
  local out
  out=$(start_stack uav0_nav_indoor uav_arena_indoor false gm1_bringup.log) || { bump 1; return; }
  local log; log=$(echo "$out" | grep -oP 'MISSION_LOG=\K.*')
  wait_nodes mission_executor_node navigator_action_server_node route_planner_node \
    control_authority_manager_node safety_supervisor_node || { bump 1; teardown; return; }

  python3 -u "$PROBE" --bringup-log "$log" patrol --uav-id "$UAV_ID" \
    2>&1 | tee "$LOGDIR/gm1_probe.log" | tail -80
  local v=${PIPESTATUS[0]}
  bump "$v"
  echo "G-M1 verdict: $v"
  teardown
}

# ============================================================ G-M2 inspect
round_m2() {
  echo "############################################################"
  echo "=== G-M2: inspect_point e2e (uav0_nav / uav_arena, perception:=true) ==="
  echo "############################################################"
  teardown || { bump 1; return; }
  local out
  out=$(start_stack uav0_nav uav_arena true gm2_bringup.log) || { bump 1; return; }
  local log; log=$(echo "$out" | grep -oP 'MISSION_LOG=\K.*')
  wait_nodes mission_executor_node navigator_action_server_node marker_detector_node \
    world_model_node control_authority_manager_node safety_supervisor_node \
    || { bump 1; teardown; return; }

  local mx=5.0 my=-6.0
  bash "$WORKSPACE/scripts/spawn_marker.sh" uav_arena 7 "$mx" "$my" 0.01 gm2_marker
  sleep 2

  python3 -u "$PROBE" --bringup-log "$log" inspect --uav-id "$UAV_ID" \
    --marker-id 7 --marker-x "$mx" --marker-y "$my" --inspect-z 2.5 \
    2>&1 | tee "$LOGDIR/gm2_probe.log" | tail -80
  local v=${PIPESTATUS[0]}
  bump "$v"
  echo "G-M2 verdict: $v"
  teardown
}

# ============================================================ G-M3 follow
round_m3() {
  echo "############################################################"
  echo "=== G-M3: follow_target e2e (uav0_track / uav_arena, perception:=true) ==="
  echo "############################################################"
  teardown || { bump 1; return; }
  local out
  out=$(start_stack uav0_track uav_arena true gm3_bringup.log) || { bump 1; return; }
  local log; log=$(echo "$out" | grep -oP 'MISSION_LOG=\K.*')
  wait_nodes mission_executor_node navigator_action_server_node obstacle_extractor_node \
    target_tracker_node world_model_node control_authority_manager_node safety_supervisor_node \
    || { bump 1; teardown; return; }

  bash "$WORKSPACE/scripts/spawn_target_box.sh" uav_arena 3.0 0.0 0.2 0.0 gm3_target_box
  sleep 2

  python3 -u "$PROBE" --bringup-log "$log" follow --uav-id "$UAV_ID" \
    2>&1 | tee "$LOGDIR/gm3_probe.log" | tail -100
  local v=${PIPESTATUS[0]}
  bump "$v"
  echo "G-M3 verdict: $v"
  teardown
}

# ==================================================== G-M4.1 marker hide/reveal
round_m4a() {
  echo "############################################################"
  echo "=== G-M4.1: marker hide/reveal during inspect_point ==="
  echo "############################################################"
  teardown || { bump 1; return; }
  local out
  out=$(start_stack uav0_nav uav_arena true gm4a_bringup.log) || { bump 1; return; }
  local log; log=$(echo "$out" | grep -oP 'MISSION_LOG=\K.*')
  wait_nodes mission_executor_node navigator_action_server_node marker_detector_node \
    world_model_node || { bump 1; teardown; return; }

  local mx=5.0 my=-6.0
  local sdf="$WORKSPACE/install/uav_sim_gz/share/uav_sim_gz/models/aruco_7/model.sdf"
  bash "$WORKSPACE/scripts/spawn_marker.sh" uav_arena 7 "$mx" "$my" 0.01 gm_event_marker
  sleep 2

  python3 -u "$PROBE" --bringup-log "$log" event-marker --uav-id "$UAV_ID" \
    --world uav_arena --marker-id 7 --marker-x "$mx" --marker-y "$my" --marker-z 0.01 \
    --inspect-z 2.5 --marker-name gm_event_marker --marker-sdf "$sdf" \
    2>&1 | tee "$LOGDIR/gm4a_probe.log" | tail -100
  local v=${PIPESTATUS[0]}
  bump "$v"
  echo "G-M4.1 verdict: $v"
  teardown
}

# ==================================================== G-M4.2 battery WARN
round_m4b() {
  echo "############################################################"
  echo "=== G-M4.2: battery WARN during indoor_patrol (R31: artificial drain) ==="
  echo "############################################################"
  teardown || { bump 1; return; }
  local out
  out=$(start_stack uav0_nav_indoor uav_arena_indoor false gm4b_bringup.log) || { bump 1; return; }
  local log; log=$(echo "$out" | grep -oP 'MISSION_LOG=\K.*')
  wait_nodes mission_executor_node navigator_action_server_node || { bump 1; teardown; return; }

  python3 -u "$PROBE" --bringup-log "$log" event-battery --uav-id "$UAV_ID" --drain-sec 45 \
    2>&1 | tee "$LOGDIR/gm4b_probe.log" | tail -100
  local v=${PIPESTATUS[0]}
  bump "$v"
  echo "G-M4.2 verdict: $v"
  teardown
}

# ============================================= G-M4.3 safety seize + control
round_m4c() {
  echo "############################################################"
  echo "=== G-M4.3: safety seize -> PAUSED -> ClearFault -> Resume (mission) ==="
  echo "############################################################"
  teardown || { bump 1; return; }
  local out
  out=$(start_stack uav0_nav_indoor uav_arena_indoor false gm4c_bringup.log) || { bump 1; return; }
  local log; log=$(echo "$out" | grep -oP 'MISSION_LOG=\K.*')
  wait_nodes mission_executor_node navigator_action_server_node safety_supervisor_node \
    control_authority_manager_node || { bump 1; teardown; return; }

  echo "--- R27-1: confirm 0 publishers on obstacle_map_local before injection ---"
  # Discovery lags the process table (ops-playbook Sec12): safety_supervisor_node
  # DOES subscribe (safety_supervisor_node.cpp:238), but a fresh --no-daemon
  # `ros2 topic info` process needs a few seconds to discover that
  # subscription -- "Unknown topic" on the first try is a discovery race, not
  # evidence of 0 subscribers. assert_single_writer (gate_common.sh) retries
  # with a deadline instead of one-shot, and returns 2 (not 1) when it truly
  # never became measurable within that deadline.
  local topic="/uav/$UAV_ID/world/obstacle_map_local"
  assert_single_writer "$topic" 0
  local writer_rc=$?
  if [ "$writer_rc" != "0" ]; then
    bump "$writer_rc"; teardown; return
  fi

  python3 -u "$PROBE" --bringup-log "$log" event-authority --uav-id "$UAV_ID" \
    2>&1 | tee "$LOGDIR/gm4c_probe.log" | tail -150
  local v=${PIPESTATUS[0]}
  bump "$v"
  echo "G-M4.3 verdict: $v"
  teardown
}

# =============== G-M4.3 positive control: WITHOUT mission, direct navigator Goto
round_m4c_control() {
  echo "############################################################"
  echo "=== G-M4.3 POSITIVE CONTROL: no mission -- reruns verify_obstacle_hold.sh"
  echo "=== (proves that WITHOUT mission cancelling, the goal auto-resumes)"
  echo "############################################################"
  teardown || { bump 1; return; }
  bash "$WORKSPACE/scripts/verify_obstacle_hold.sh" 2>&1 | tee "$LOGDIR/gm4c_control.log" | tail -80
  local v=${PIPESTATUS[0]}
  bump "$v"
  echo "G-M4.3 positive control verdict (0=PASS -> confirms auto-resume happened): $v"
}

# ==================================================== G-M4.4 lost target
round_m4d() {
  echo "############################################################"
  echo "=== G-M4.4: lost target during follow_target ==="
  echo "############################################################"
  teardown || { bump 1; return; }
  local out
  out=$(start_stack uav0_track uav_arena true gm4d_bringup.log) || { bump 1; return; }
  local log; log=$(echo "$out" | grep -oP 'MISSION_LOG=\K.*')
  wait_nodes mission_executor_node navigator_action_server_node obstacle_extractor_node \
    target_tracker_node world_model_node || { bump 1; teardown; return; }

  # Coordinator ruling 2026-08-23: the previous G-M4.4 "failure" was the
  # target_id=-1 ("bam bat ky vat gi") contract running CORRECTLY -- after
  # the spawned box vanished, TargetSeen legitimately rebound to
  # obstacle_pillar_north/south (real, static obstacles in uav_arena, both
  # within front-camera FOV) and tracked them stably for 58s with zero
  # WARNs -- that is what a real trackable object looks like, not a
  # phantom (phantoms die in ~1s under M/N, confirm-window measured at
  # 3-4s max). The gate was testing the wrong scenario ("target vanishes
  # among other trackable things") instead of the intended one ("nothing
  # trackable remains"). Runtime-remove BOTH pillars so THIS round's arena
  # has genuinely nothing else to rebind to -- world file itself is left
  # untouched (sim restarts fresh every round, no restore needed).
  echo "--- runtime-removing obstacle_pillar_north/south for the no-target scenario ---"
  for pillar in obstacle_pillar_north obstacle_pillar_south; do
    reply=$(gz service -s "/world/uav_arena/remove" \
      --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 5000 \
      --req "name: \"$pillar\" type: MODEL")
    echo "  remove $pillar: $reply"
    echo "$reply" | grep -q 'data: true' || {
      echo "FATAL: could not confirm removal of $pillar"; bump 1; teardown; return; }
  done
  sleep 1
  remaining=$(gz model --list 2>/dev/null | grep -c 'obstacle_pillar')
  if [ "$remaining" != "0" ]; then
    echo "FATAL: R27-1 -- gz model --list still shows $remaining obstacle_pillar* after removal"
    gz model --list 2>/dev/null | grep 'obstacle_pillar'
    bump 1; teardown; return
  fi
  echo "arena cleared of both catchable pillars (R27-1 confirmed via gz model --list) -- clean for the no-target scenario"

  bash "$WORKSPACE/scripts/spawn_target_box.sh" uav_arena 3.0 0.0 0.2 0.0 gm_event_box
  sleep 2

  python3 -u "$PROBE" --bringup-log "$log" event-target --uav-id "$UAV_ID" \
    --world uav_arena --box-name gm_event_box --mission-timeout-sec 180 \
    2>&1 | tee "$LOGDIR/gm4d_probe.log" | tail -100
  local v=${PIPESTATUS[0]}
  bump "$v"
  echo "G-M4.4 verdict: $v"
  teardown
}

# ==================================================== M5 regression, mission:=false
round_m5() {
  echo "############################################################"
  echo "=== M5 regression after P9 (mission:=false, baseline untouched) ==="
  echo "############################################################"
  teardown || { bump 1; return; }
  UAV_MODEL=uav0 UAV_WORLD=uav_arena bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -3
  wait_px4 || { bump 1; return; }
  reset_battery_params
  setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$UAV_ID" \
    > "$LOGDIR/m5_after_p9.log" 2>&1 < /dev/null &
  sleep 20
  ros2 run uav_bringup smoke_flight.py --flights 3 \
    2>&1 | tee "$LOGDIR/m5_after_p9_smoke.log" | tail -40
  local v=${PIPESTATUS[0]}
  bump "$v"
  echo "M5 verdict: $v"
  teardown
}

# --------------------------------------------------------------- dispatch
ROUNDS=("$@")
[ ${#ROUNDS[@]} -eq 0 ] && ROUNDS=(m1 m2 m3 m4a m4b m4c m4c-control m4d m5)

for r in "${ROUNDS[@]}"; do
  case "$r" in
    m1) round_m1 ;;
    m2) round_m2 ;;
    m3) round_m3 ;;
    m4a) round_m4a ;;
    m4b) round_m4b ;;
    m4c) round_m4c ;;
    m4c-control|m4b-control) round_m4c_control ;;
    m4d) round_m4d ;;
    m5) round_m5 ;;
    *) echo "FATAL: unknown round '$r'"; bump 1 ;;
  esac
done

echo
echo "############################################################"
echo "verify_mission.sh OVERALL exit status: $OVERALL   (0=PASS 1=FAIL 2=FAILED TO MEASURE)"
exit "$OVERALL"
