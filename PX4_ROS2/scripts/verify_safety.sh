#!/bin/bash
# G-S2 (P8-safety.md S:8): fly a Goto, inject localization loss mid-flight by
# 3 variants, and confirm safety cuts within budget with the real acceptance
# criteria (0 cmd_safety messages, cut before 0.83 m of blind travel).
# Usage: bash scripts/verify_safety.sh
# Env: UAV_MODEL UAV_WORLD UAV_ID INJECT_AFTER_SEC WATCH_SEC
# NOT RUN by the author - sim access is reserved for the verifier.
# R3: PX4's real post-loss-of-offboard behavior (nav_state_raw/flight_mode)
# is READ from safety_gate.py's report, never invented -- print it verbatim.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
# Indoor preferred (S:8): no GPS fallback to mask a VIO/mux loss.
# 🔴 MUST be the _indoor airframe, NOT plain uav0_nav: only
# airframes/4112_gz_uav0_nav_indoor sets SIM_GPS_USED=0 (GPS truly unusable).
# Flying plain uav0_nav in this world still gets a full GPS fix -- GPS
# silently backfills any VIO/mux loss and the whole test measures nothing
# (found the hard way: first G-S2 run, 6/6 injections showed is_valid never
# flipping false because GPS was quietly covering for the killed source).
export UAV_MODEL=${UAV_MODEL:-uav0_nav_indoor}
export UAV_WORLD=${UAV_WORLD:-uav_arena_indoor}
UAV_ID=${UAV_ID:-uav0}
INJECT_AFTER_SEC=${INJECT_AFTER_SEC:-8}
WATCH_SEC=${WATCH_SEC:-35}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

PROBE=$WORKSPACE/src/uav_safety/test/safety_gate.py
[ -f "$PROBE" ] || { echo "FATAL: $PROBE missing"; exit 1; }

# --------------------------------------------------------------- cleanup
# Lesson from G-CA2 round 3: a leftover process from the PREVIOUS variant
# serves the OLD binary/state and silently poisons the NEXT run. Kill by
# name, not by pgrep pid alone, and VERIFY nothing is left before moving on.
teardown() {
  pkill -f 'ros2 launch uav_bringup' 2>/dev/null
  pkill -f 'safety_gate.py' 2>/dev/null
  pkill -f 'navigator_action_server_node' 2>/dev/null
  sleep 3
  bash "$WORKSPACE/scripts/stop_sim.sh" > /dev/null 2>&1
  sleep 2

  # First real run (2026-08-21) found safety_supervisor_node surviving as a
  # true orphan (no parent) after this point: ros2 launch's own SIGINT-then
  # -SIGTERM-then-SIGKILL escalation for a stubborn child can outlive our
  # 3s+2s window, and once we pkill the launch PARENT the escalation timers
  # never fire. Force-kill every bringup node executable by path instead of
  # trusting launch's own timers to finish before we move on.
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
  # Stale ros2 daemon graph cache after a polluted run caused a real
  # "Node not found" on `ros2 param set /vio_adapter_node ...` in variant 3
  # of the first attempt -- force a fresh daemon for the next variant.
  ros2 daemon stop > /dev/null 2>&1
  return 0
}

start_stack() {
  local log="$1"
  echo "--- starting sim (model=$UAV_MODEL world=$UAV_WORLD) ---"
  bash "$WORKSPACE/scripts/start_sim.sh" 2>&1 | tail -2

  # $SECONDS is cumulative for the WHOLE script (bash builtin, never reset
  # per-call) -- use a local elapsed counter so each variant's "after Xs" is
  # its own wait time, not a running total that looks like a slowdown.
  local wait_start=$SECONDS
  local deadline=$((SECONDS + 420))
  until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
      | grep -q '/fmu/out/vehicle_odometry'; do
    if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; return 1; fi
    sleep 10
  done
  echo "PX4 publishing after $((SECONDS - wait_start))s"

  setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$UAV_ID" \
    > "$log" 2>&1 < /dev/null &
  sleep 25
  local started
  started=$(grep -c 'process started' "$log")
  echo "bringup: $started process(es) started"
  grep -q "safety supervisor ready" "$log" || {
    echo "FATAL: safety_supervisor_node never came up"; tail -20 "$log"; return 1;
  }
  return 0
}

# variant: "none" | "pkill_vio" | "pkill_mux" | "force_tracking_loss"
run_variant() {
  local variant="$1"
  # /tmp observed cleared mid-session (WSL VM idle-reset, 2026-08-21 run) --
  # $HOME survives across those resets, unlike tmpfs /tmp.
  mkdir -p "$HOME/gs2_logs"
  local probe_log="$HOME/gs2_logs/gs2_${variant}.log"
  local bringup_log="$HOME/gs2_logs/gs2_${variant}_bringup.log"

  teardown || { echo "FATAL: teardown could not reach a clean env before $variant"; return 1; }
  start_stack "$bringup_log" || { echo "FATAL: stack never came up for $variant"; return 1; }

  echo "--- flying + watching ($variant) ---"
  local extra_args=()
  [ "$variant" = "none" ] && extra_args+=(--expect-clean)
  python3 -u "$PROBE" --uav-id "$UAV_ID" --watch-seconds "$WATCH_SEC" "${extra_args[@]}" \
    > "$probe_log" 2>&1 &
  local probe_pid=$!

  if [ "$variant" != "none" ]; then
    # R21: anchor injection on the "goto sent" EVENT the probe prints, not a
    # fixed sleep -- a fixed 8s sleep landed mid-TAKEOFF (2.5 m at 0.45 m/s
    # climb ~= 5.5s, no margin), aborting the takeoff action before the
    # probe ever reached the goto leg the whole test is designed around.
    # INJECT_AFTER_SEC is now a bound-from-goto-sent, not from probe start.
    local goto_deadline=$((SECONDS + 90))
    until grep -q 'goto sent' "$probe_log" 2>/dev/null; do
      if [ $SECONDS -gt $goto_deadline ]; then
        echo "FATAL: probe never reached the goto leg (no 'goto sent' in $probe_log)"
        kill "$probe_pid" 2>/dev/null
        wait "$probe_pid" 2>/dev/null
        return 1
      fi
      sleep 0.5
    done
    sleep "$INJECT_AFTER_SEC"
    local injected_at
    injected_at=$(date +%s.%N)
    case "$variant" in
      pkill_vio)
        echo "INJECT @ $injected_at: pkill vio_adapter_node"
        pkill -f 'vio_adapter_node' ;;
      pkill_mux)
        echo "INJECT @ $injected_at: pkill localization_mux_node"
        pkill -f 'localization_mux_node' ;;
      force_tracking_loss)
        echo "INJECT @ $injected_at: ros2 param set vio_adapter_node degrade.force_tracking_loss true"
        ros2 param set /vio_adapter_node degrade.force_tracking_loss true ;;
    esac
  fi

  wait "$probe_pid"
  local verdict=$?
  echo "=== $variant probe log ==="
  cat "$probe_log"
  echo "=== $variant verdict: $verdict ==="

  teardown
  return $verdict
}

overall=0

echo "############ POSITIVE CONTROL (no injection) ############"
run_variant none
control_verdict=$?
[ $control_verdict -eq 0 ] || overall=1

echo "############ VARIANT 1: pkill vio_adapter_node ############"
run_variant pkill_vio
v1=$?
[ $v1 -eq 0 ] || overall=1

echo "############ VARIANT 2: pkill localization_mux_node ############"
run_variant pkill_mux
v2=$?
[ $v2 -eq 0 ] || overall=1

echo "############ VARIANT 3: degrade.force_tracking_loss ############"
run_variant force_tracking_loss
v3=$?
[ $v3 -eq 0 ] || overall=1

echo ""
echo "=== G-S2 SUMMARY ==="
echo "positive control : $([ $control_verdict -eq 0 ] && echo PASS || echo FAIL) ($control_verdict)"
echo "pkill_vio         : $([ $v1 -eq 0 ] && echo PASS || echo FAIL) ($v1)"
echo "pkill_mux         : $([ $v2 -eq 0 ] && echo PASS || echo FAIL) ($v2)"
echo "force_tracking_loss: $([ $v3 -eq 0 ] && echo PASS || echo FAIL) ($v3)"
echo "G-S2 exit status: $overall"
exit $overall
