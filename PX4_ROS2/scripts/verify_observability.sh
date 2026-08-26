#!/bin/bash
# P10.3 -- G-O1 storage-durability gate for rosbag_manager_node (P10.2).
# Pure-ROS experiment on ROS_DOMAIN_ID 99: NO Gazebo/PX4. kill -9's the
# recorder mid-write across 3 storage configs (mcap+file/zstd, mcap+none,
# sqlite3+none) x10 rounds each, plus a 2-round positive control (clean
# SIGINT) proving the measurement method itself can count correctly.
# Usage: bash scripts/verify_observability.sh [round ...]
#   rounds: o1-control o1-mcap-zstdfile o1-mcap-none o1-sqlite3-none
#           o1-report   (default: all 4, in that order == "o1")
#           o1-prod (P10.6 debt close, plan S:6c "Nợ nhỏ": sqlite3+file/
#           zstd x2 kills, own CSV/verdict, NOT part of the "o1" bundle)
#           o2-main o2-control o2-hz o2-report (P10.7 G-O2 cost gate;
#           "o2" runs all 4 in that order). o2-m5 is standalone -- run it
#           AFTER editing+rebuilding observability_params.yaml off o2-hz's
#           numbers (plan S:6 row P10.7).
#           o3-camera-health o3-flight o3-replay o3-loop o3-loop-control
#           o3-synth o3-loop-report (P10.8b/c G-O3: camera_health_node
#           wire-check + replay-vs-original + N-c loop liveness watch +
#           deterministic N-c regression proof; "o3" runs all 7 in that
#           order). o3-flight writes ~/gate_logs/o3_bag_dir.txt +
#           o3_jsonl_path.txt; o3-replay/o3-loop*/o3-loop-report all read
#           the bag path back off that file, so o3-flight MUST run first
#           (or be re-pointed by hand-editing that file). o3-synth does NOT
#           depend on the bag (own standalone /clock + cmd_mission driver).
#           d0-baseline (P10.9b D0: parked-stack preflight baseline capture,
#           2 configs x 60s, produces a candidate waiver table for the
#           project owner to review -- .claude/plan/P10-gonogo-design-
#           panel.md S:4 thứ tự thi công D0). Standalone, does not depend
#           on o4-gate's state.
#           o4-gate (P10.9a G-O4, redesigned P10.9b: go/no-go on the wire).
#           One real Gazebo+PX4+ROS bringup per sub-part, domain 0. Every
#           sub-part stays disarmed the whole time, so gate_mode settles to
#           "preflight" by MEASUREMENT (armed=false, P10.9b -- there is no
#           more launch-arg default to point at) -- go_no_go is a PRE-flight
#           gate by design, this round never flies. (a)+(b) share one
#           bringup (perception:=false default):
#           settle then read go_no_go=GO off the wire, then SIGKILL
#           localization_mux_node as the positive control and assert
#           NO_GO within 2s with the correct worst_item. (c) perception:=
#           true (world/semantic_landmarks gets a real owner). (d)
#           blackbox:=false diagnostics:=true (D-4's real-flight combo --
#           proves the black box being off does not gate a flight, O1).
#           (e) stamp_sec/wall_stamp_sec both present and monotonic (C6),
#           read off the JSONL already collected by (a)/(b)/(c)/(d). (f) MỚI
#           (P10.9b G6): a FRESH bringup, preflight GO with waived_count>0
#           actually observed (R27-1) and the waived child set a SUBSET of
#           a signed allow-list (o4_waiver_gate.py) -- catches "someone
#           quietly added a waiver row" (F3's organisational failure mode).
# Exit convention (verify_mission.sh's): 0=PASS 1=FAIL 2=FAILED TO MEASURE
# What each number means: src/uav_observability/test/go1_report.py (o1),
# inline python in round_o2_report (o2).
# Plan: .claude/plan/P10-observability.md S:6 rows P10.3 (o1) / P10.7 (o2).
#
# R31/residual risk: kill -9 tests PROCESS DEATH only. The OS still flushes
# its own page cache to disk when a process dies -- this does NOT model a
# real power-loss event (page cache survives a kill -9, it does not survive
# a power cut). Any PASS below is scoped to "recorder process crashed",
# never "board lost power mid-flight". That gap stays open in the README.
set -o pipefail

WORKSPACE_WSL=$HOME/PX4_ROS2
# test/*.py and test/*.yaml are NOT visible under WORKSPACE_WSL/src --
# scripts/ is a symlink into /mnt/c but src/ is a real WSL-native copy that
# only colcon build syncs (ops-playbook "src/ khong phai symlink nhu
# scripts/"). Read straight off /mnt/c instead of requiring a manual cp.
WORKSPACE_WIN=/mnt/c/code/PX4_ROS2
LOGDIR=$HOME/gate_logs
mkdir -p "$LOGDIR"

source /opt/ros/humble/setup.bash
source "$WORKSPACE_WSL/install/setup.bash"
export ROS_DOMAIN_ID=99

NODE_BIN="$WORKSPACE_WSL/install/uav_observability/lib/uav_observability/rosbag_manager_node"
PARAMS_TEMPLATE="$WORKSPACE_WIN/src/uav_observability/test/go1_params.yaml"
PUBLISHER="$WORKSPACE_WIN/src/uav_observability/test/go1_publisher.py"
READER="$WORKSPACE_WIN/src/uav_observability/test/go1_bag_reader.py"
REPORTER="$WORKSPACE_WIN/src/uav_observability/test/go1_report.py"
BAG_ROOT=/tmp/go1_bags
RESULTS_CSV="$LOGDIR/go1_results.csv"

[ -x "$NODE_BIN" ] || { echo "FATAL: $NODE_BIN missing or not built"; exit 1; }
for f in "$PARAMS_TEMPLATE" "$PUBLISHER" "$READER" "$REPORTER"; do
  [ -f "$f" ] || { echo "FATAL: $f missing"; exit 1; }
done

OVERALL=0
bump() { if [ "$1" -gt "$OVERALL" ]; then OVERALL=$1; fi; }

# ---------------------------------------------------------------- teardown
teardown() {
  pkill -9 -f "[r]osbag_manager_node.*go1" 2>/dev/null
  pkill -9 -f "[g]o1_publisher.py" 2>/dev/null
  sleep 1
  rm -rf "$BAG_ROOT"
  return 0
}

# make_params CONFIG_NAME STORAGE_ID MODE FORMAT -> echoes generated path
make_params() {
  local name="$1" storage="$2" mode="$3" fmt="$4"
  local out="/tmp/go1_params_${name}.yaml"
  sed -e "s/__STORAGE_ID__/${storage}/" -e "s/__COMPRESSION_MODE__/${mode}/" \
      -e "s/__COMPRESSION_FORMAT__/${fmt}/" "$PARAMS_TEMPLATE" > "$out"
  echo "$out"
}

# wait_node_up LOG -- bounded wait for the recorder's own "ready" log line.
wait_node_up() {
  local log="$1"
  local deadline=$((SECONDS + 20))
  until grep -q "rosbag manager ready for" "$log" 2>/dev/null; do
    if [ $SECONDS -gt $deadline ]; then
      return 1
    fi
    sleep 1
  done
  return 0
}

# wait_dead PID DEADLINE_SEC
wait_dead() {
  local pid="$1" budget="$2"
  local deadline=$((SECONDS + budget))
  while kill -0 "$pid" 2>/dev/null; do
    if [ $SECONDS -gt $deadline ]; then
      return 1
    fi
    sleep 0.3
  done
  return 0
}

# measure_bag CONFIG ITER STORAGE_ID BAG_DIR -> echoes
#   "info_direct,info_reindex,msg_count,seconds_lost" (info_reindex is NA
#   unless storage_id=sqlite3 AND direct info failed).
measure_bag() {
  local config="$1" iter="$2" storage_id="$3" bag_dir="$4" t_kill="$5"

  local info_direct=1
  ros2 bag info "$bag_dir" > "/tmp/go1_info_${config}_${iter}_direct.log" 2>&1 || info_direct=0

  local info_reindex="NA"
  if [ "$info_direct" = "0" ] && [ "$storage_id" = "sqlite3" ]; then
    ros2 bag reindex "$bag_dir" > "/tmp/go1_reindex_${config}_${iter}.log" 2>&1
    if ros2 bag info "$bag_dir" > "/tmp/go1_info_${config}_${iter}_reindexed.log" 2>&1; then
      info_reindex=1
    else
      info_reindex=0
    fi
  fi

  local reader_out msg_count last_stamp
  reader_out=$(python3 "$READER" "$bag_dir" "$storage_id" 2>"/tmp/go1_reader_${config}_${iter}.err")
  msg_count=$(echo "$reader_out" | grep -oP 'MSG_COUNT=\K[0-9]+')
  last_stamp=$(echo "$reader_out" | grep -oP 'LAST_STAMP_SEC=\K(?:[0-9.]+|NAN)')
  msg_count=${msg_count:-0}
  last_stamp=${last_stamp:-NAN}

  local seconds_lost=NAN
  if [ "$last_stamp" != "NAN" ]; then
    seconds_lost=$(python3 -c "print(f'{${t_kill} - ${last_stamp}:.3f}')")
  fi
  echo "$info_direct,$info_reindex,$msg_count,$seconds_lost"
}

# ------------------------------------------------------- o1-control (x2)
round_o1_control() {
  echo "############################################################"
  echo "=== G-O1 positive control: clean SIGINT, 1x mcap 1x sqlite3 ==="
  echo "############################################################"
  local specs=("ctrl_mcap:mcap:none:none" "ctrl_sqlite3:sqlite3:none:none")
  local spec name storage mode fmt params_file
  for spec in "${specs[@]}"; do
    IFS=: read -r name storage mode fmt <<< "$spec"
    teardown
    params_file=$(make_params "$name" "$storage" "$mode" "$fmt")
    echo "--- control round: $name (storage=$storage) ---"

    rm -rf "$BAG_ROOT"; mkdir -p "$BAG_ROOT"
    local node_log="/tmp/go1_node_${name}_ctrl.log"
    : > "$node_log"
    "$NODE_BIN" --ros-args --params-file "$params_file" > "$node_log" 2>&1 < /dev/null &
    local node_pid=$!

    if ! wait_node_up "$node_log"; then
      echo "FAILED TO MEASURE: $name node never came up"
      tail -30 "$node_log"
      kill -9 "$node_pid" 2>/dev/null
      bump 2; continue
    fi

    local fast_log="/tmp/go1_pub_fast_${name}_ctrl.log" slow_log="/tmp/go1_pub_slow_${name}_ctrl.log"
    python3 "$PUBLISHER" /uav_obs_test/topic_fast 20 60 > "$fast_log" 2>&1 &
    local fast_pid=$!
    python3 "$PUBLISHER" /uav_obs_test/topic_slow 5 60 > "$slow_log" 2>&1 &
    local slow_pid=$!

    sleep 22
    # Control ordering DIFFERS from the kill rounds on purpose: stop the
    # publishers FIRST, give the executor a moment to drain what already
    # arrived, THEN SIGINT the node -- this is what makes msg_count vs
    # published a fair ±1% test of the measurement method, not a race
    # between "last message sent" and "process asked to shut down".
    kill -TERM "$fast_pid" "$slow_pid" 2>/dev/null
    wait "$fast_pid" 2>/dev/null; wait "$slow_pid" 2>/dev/null
    sleep 2
    kill -INT "$node_pid" 2>/dev/null
    if ! wait_dead "$node_pid" 15; then
      echo "FAILED TO MEASURE: $name node did not exit cleanly within 15s of SIGINT"
      kill -9 "$node_pid" 2>/dev/null
      bump 2; continue
    fi

    local published_fast published_slow
    published_fast=$(grep -oP 'PUBLISHED=\K[0-9]+' "$fast_log" | tail -1); published_fast=${published_fast:-0}
    published_slow=$(grep -oP 'PUBLISHED=\K[0-9]+' "$slow_log" | tail -1); published_slow=${published_slow:-0}
    local published=$((published_fast + published_slow))

    local bag_dir
    bag_dir=$(find "$BAG_ROOT" -mindepth 1 -maxdepth 1 -type d | head -1)
    if [ -z "$bag_dir" ]; then
      echo "FAILED TO MEASURE: $name -- no bag directory found after clean shutdown"
      bump 2; continue
    fi

    local t_kill; t_kill=$(python3 -c 'import time; print(f"{time.time():.6f}")')
    local measured; measured=$(measure_bag "$name" 0 "$storage" "$bag_dir" "$t_kill")
    IFS=, read -r info_direct info_reindex msg_count seconds_lost <<< "$measured"

    echo "${name},0,false,${storage},NA,${t_kill},${info_direct},${info_reindex},${msg_count},${published},${seconds_lost}" \
      >> "$RESULTS_CSV"
    echo "$name: readable=$([ "$info_direct" = 1 ] && echo yes || echo no) published=$published msg_count=$msg_count"
  done
  teardown
}

# ------------------------------------------------------- kill config (x10)
round_o1_config() {
  local config="$1" storage="$2" mode="$3" fmt="$4"
  echo "############################################################"
  echo "=== G-O1 kill -9: $config (storage=$storage compression=$mode/$fmt) x10 ==="
  echo "############################################################"
  local params_file; params_file=$(make_params "$config" "$storage" "$mode" "$fmt")

  local iter
  for iter in $(seq 1 10); do
    teardown
    rm -rf "$BAG_ROOT"; mkdir -p "$BAG_ROOT"
    local node_log="/tmp/go1_node_${config}_${iter}.log"
    : > "$node_log"
    "$NODE_BIN" --ros-args --params-file "$params_file" > "$node_log" 2>&1 < /dev/null &
    local node_pid=$!

    if ! wait_node_up "$node_log"; then
      echo "  [$config #$iter] FAILED TO MEASURE: node never came up"
      tail -20 "$node_log"
      kill -9 "$node_pid" 2>/dev/null
      bump 2
      # VÀNG#5: do_kill=false -- the node never came up so nothing was
      # actually killed. Marking this true made a startup FTM count as a
      # real kill-test round in go1_report.py's denominator (do_kill filter).
      echo "$config,$iter,false,$storage,NA,NA,0,NA,0,0,NAN" >> "$RESULTS_CSV"
      continue
    fi

    local fast_log="/tmp/go1_pub_fast_${config}_${iter}.log" slow_log="/tmp/go1_pub_slow_${config}_${iter}.log"
    python3 "$PUBLISHER" /uav_obs_test/topic_fast 20 60 > "$fast_log" 2>&1 &
    local fast_pid=$!
    python3 "$PUBLISHER" /uav_obs_test/topic_slow 5 60 > "$slow_log" 2>&1 &
    local slow_pid=$!
    local t_start; t_start=$(python3 -c 'import time; print(f"{time.time():.6f}")')

    sleep 22   # let a few hundred messages accumulate before the kill

    local t_kill; t_kill=$(python3 -c 'import time; print(f"{time.time():.6f}")')
    kill -9 "$node_pid" 2>/dev/null
    # Publishers stay unsignaled a beat longer than the kill on PURPOSE --
    # the recorder must be dead WHILE traffic is still flowing to actually
    # exercise the cache/flush path this gate is measuring.
    kill -TERM "$fast_pid" "$slow_pid" 2>/dev/null

    wait_dead "$node_pid" 10 || echo "  WARNING: $node_pid outlived SIGKILL by >10s (should not happen)"
    wait "$fast_pid" 2>/dev/null; wait "$slow_pid" 2>/dev/null

    local published_fast published_slow
    published_fast=$(grep -oP 'PUBLISHED=\K[0-9]+' "$fast_log" | tail -1); published_fast=${published_fast:-0}
    published_slow=$(grep -oP 'PUBLISHED=\K[0-9]+' "$slow_log" | tail -1); published_slow=${published_slow:-0}
    local published=$((published_fast + published_slow))

    local bag_dir
    bag_dir=$(find "$BAG_ROOT" -mindepth 1 -maxdepth 1 -type d | head -1)
    if [ -z "$bag_dir" ]; then
      echo "  [$config #$iter] no bag directory survived the kill at all (t_start=$t_start)"
      echo "$config,$iter,true,$storage,$t_start,$t_kill,0,NA,0,$published,NAN" >> "$RESULTS_CSV"
      bump 1
      continue
    fi

    local measured; measured=$(measure_bag "$config" "$iter" "$storage" "$bag_dir" "$t_kill")
    IFS=, read -r info_direct info_reindex msg_count seconds_lost <<< "$measured"
    echo "$config,$iter,true,$storage,$t_start,$t_kill,$info_direct,$info_reindex,$msg_count,$published,$seconds_lost" \
      >> "$RESULTS_CSV"
    echo "  [$config #$iter] readable_direct=$info_direct reindex=$info_reindex msg_count=$msg_count/$published seconds_lost=$seconds_lost"
  done
  teardown
}

# ------------------------------------------------------- o1-prod (x2, P10.6)
# Closes the plan S:6c debt: sqlite3+file/zstd was never kill-tested (only
# sqlite3+none was, in round_o1_config). Own CSV + inline verdict -- kept
# OUT of go1_results.csv/go1_report.py on purpose: that reporter's
# KILL_CONFIGS/STORAGE_OF tables are pinned to the 3-config x10 G-O1 shape,
# and this is a 1-config x2 top-up, not a re-run of the original gate.
round_o1_prod() {
  local config="sqlite3_zstdfile_prod" storage="sqlite3" mode="file" fmt="zstd"
  local prod_csv="$LOGDIR/go1_prod_results.csv"
  : > "$prod_csv"
  echo "############################################################"
  echo "=== G-O1 prod debt close: $config (storage=$storage compression=$mode/$fmt) x2 ==="
  echo "############################################################"
  local params_file; params_file=$(make_params "$config" "$storage" "$mode" "$fmt")

  local iter
  for iter in 1 2; do
    teardown
    rm -rf "$BAG_ROOT"; mkdir -p "$BAG_ROOT"
    local node_log="/tmp/go1_node_${config}_${iter}.log"
    : > "$node_log"
    "$NODE_BIN" --ros-args --params-file "$params_file" > "$node_log" 2>&1 < /dev/null &
    local node_pid=$!

    if ! wait_node_up "$node_log"; then
      echo "  [$config #$iter] FAILED TO MEASURE: node never came up"
      tail -20 "$node_log"
      kill -9 "$node_pid" 2>/dev/null
      bump 2
      # VÀNG#5 (same fix as round_o1_config): do_kill=false, node never came up.
      echo "$config,$iter,false,$storage,NA,NA,0,NA,0,0,NAN" >> "$prod_csv"
      continue
    fi

    local fast_log="/tmp/go1_pub_fast_${config}_${iter}.log" slow_log="/tmp/go1_pub_slow_${config}_${iter}.log"
    python3 "$PUBLISHER" /uav_obs_test/topic_fast 20 60 > "$fast_log" 2>&1 &
    local fast_pid=$!
    python3 "$PUBLISHER" /uav_obs_test/topic_slow 5 60 > "$slow_log" 2>&1 &
    local slow_pid=$!
    local t_start; t_start=$(python3 -c 'import time; print(f"{time.time():.6f}")')

    sleep 22

    local t_kill; t_kill=$(python3 -c 'import time; print(f"{time.time():.6f}")')
    kill -9 "$node_pid" 2>/dev/null
    kill -TERM "$fast_pid" "$slow_pid" 2>/dev/null

    wait_dead "$node_pid" 10 || echo "  WARNING: $node_pid outlived SIGKILL by >10s (should not happen)"
    wait "$fast_pid" 2>/dev/null; wait "$slow_pid" 2>/dev/null

    local published_fast published_slow
    published_fast=$(grep -oP 'PUBLISHED=\K[0-9]+' "$fast_log" | tail -1); published_fast=${published_fast:-0}
    published_slow=$(grep -oP 'PUBLISHED=\K[0-9]+' "$slow_log" | tail -1); published_slow=${published_slow:-0}
    local published=$((published_fast + published_slow))

    local bag_dir
    bag_dir=$(find "$BAG_ROOT" -mindepth 1 -maxdepth 1 -type d | head -1)
    if [ -z "$bag_dir" ]; then
      echo "  [$config #$iter] no bag directory survived the kill at all (t_start=$t_start)"
      echo "$config,$iter,true,$storage,$t_start,$t_kill,0,NA,0,$published,NAN" >> "$prod_csv"
      bump 1
      continue
    fi

    local measured; measured=$(measure_bag "$config" "$iter" "$storage" "$bag_dir" "$t_kill")
    IFS=, read -r info_direct info_reindex msg_count seconds_lost <<< "$measured"
    echo "$config,$iter,true,$storage,$t_start,$t_kill,$info_direct,$info_reindex,$msg_count,$published,$seconds_lost" \
      >> "$prod_csv"
    echo "  [$config #$iter] readable_direct=$info_direct reindex=$info_reindex msg_count=$msg_count/$published seconds_lost=$seconds_lost"
  done
  teardown

  echo
  echo "=== o1-prod verdict ($config) ==="
  # VÀNG#5: exclude do_kill=false rows (node never came up -- a startup
  # FTM, not a kill measurement) from the denominator, same fix family as
  # go1_report.py's do_kill filter.
  local n=0 startup_ftm=0 readable_direct=0 readable_after_reindex=0
  while IFS=, read -r _c _i dk _s _ts _tk info_direct info_reindex _mc _pub _sl; do
    if [ "$dk" != "true" ]; then
      startup_ftm=$((startup_ftm + 1))
      continue
    fi
    n=$((n + 1))
    [ "$info_direct" = "1" ] && readable_direct=$((readable_direct + 1))
    { [ "$info_direct" = "1" ] || [ "$info_reindex" = "1" ]; } && readable_after_reindex=$((readable_after_reindex + 1))
  done < "$prod_csv"
  echo "  readable_direct=${readable_direct}/${n} readable(+reindex)=${readable_after_reindex}/${n}  (startup_ftm=${startup_ftm} excluded from denominator)"
  echo "  results CSV: $prod_csv"
  if [ "$n" -eq 0 ]; then
    echo "  FAILED TO MEASURE: no measured (do_kill=true) rows recorded" \
      "(startup_ftm=${startup_ftm} attempt(s) never even reached a kill)"
    bump 2
  elif [ "$readable_after_reindex" -lt "$n" ]; then
    echo "  FAIL: not all measured rounds recovered via reindex on the production compression combo"
    bump 1
  else
    echo "  PASS: sqlite3+file/zstd recovers via reindex same as sqlite3+none (S:6c debt closed)"
  fi
}

round_o1_report() {
  echo "############################################################"
  echo "=== G-O1 report/verdict ==="
  echo "############################################################"
  [ -f "$RESULTS_CSV" ] || { echo "FATAL: $RESULTS_CSV missing -- run the o1-* rounds first"; return 1; }
  python3 "$REPORTER" "$RESULTS_CSV"
}

# ============================================================ P10.7 -- G-O2
# Cost gate: does the black box add load ON the flight path (O1)? RTF is
# only a label; the real safety question is p99 inter-arrival of
# command_selected (R27: measured on the wire via a probe, not read off an
# internal field). This runs a REAL Gazebo+PX4+ROS flight -- unlike the o1
# rounds above (pure-ROS, domain 99), so every o2 round starts by unsetting
# the o1 harness's ROS_DOMAIN_ID=99 export back to the sim's default (0).

O2_UAV_ID=uav0
O2_WORLD=uav_arena
O2_MODEL=uav0_nav          # ON/OFF/control comparison: no camera, matches M5 baseline
O2_HZ_MODEL=uav0_track      # flight_only hz survey only: needs a trackable target
O2_FLIGHTS=2                # >=60s stable-streaming window per run, cheap margin over 1
LATENCY_PROBE="$WORKSPACE_WIN/src/uav_observability/test/o2_latency_probe.py"
CPU_SAMPLER="$WORKSPACE_WIN/src/uav_observability/test/o2_cpu_sampler.py"
O2_MAIN_CSV="$LOGDIR/o2_main_results.csv"
O2_CONTROL_CSV="$LOGDIR/o2_control_results.csv"
O2_HZ_LOG="$LOGDIR/o2_hz_survey.log"
O2_PROBE_DURATION=220        # ROS timer ceiling; SIGINT'd right after the flight ends anyway
O2_CPU_DURATION=220
O2_RTF_SAMPLES=90

for f in "$LATENCY_PROBE" "$CPU_SAMPLER"; do
  [ -f "$f" ] || { echo "FATAL: $f missing"; exit 1; }
done

# ---------------------------------------------------------- o2 plumbing
o2_teardown() {
  unset ROS_DOMAIN_ID
  pkill -f 'ros2 launch uav_bringup' 2>/dev/null
  pkill -f 'o2_latency_probe.py' 2>/dev/null
  pkill -f 'o2_cpu_sampler.py' 2>/dev/null
  pkill -f 'smoke_flight.py' 2>/dev/null
  sleep 2
  bash "$WORKSPACE_WSL/scripts/stop_sim.sh" > /dev/null 2>&1
  sleep 2
  pkill -9 -f "$WORKSPACE_WSL/install/.*/lib/.*_node" 2>/dev/null
  sleep 1
  local pattern='mission_executor_node|safety_supervisor_node|control_authority_manager_node|vio_adapter_node|localization_mux_node|world_model_node|target_tracker_node|obstacle_extractor_node|marker_detector_node|object_detector_node|navigator_action_server_node|rosbag_manager_node|diagnostics_node|event_logger_node'
  local orphans
  orphans=$(pgrep -f "$pattern"'|gz sim|px4' | wc -l)
  if [ "$orphans" -gt 0 ]; then
    echo "WARNING: $orphans orphan process(es) after teardown -- force killing:"
    pgrep -af "$pattern"
    pkill -9 -f "$pattern" 2>/dev/null
    sleep 1
    orphans=$(pgrep -f "$pattern"'|gz sim|px4' | wc -l)
    if [ "$orphans" -gt 0 ]; then
      echo "FATAL: $orphans orphan(s) survived SIGKILL -- refusing to continue polluted:"
      pgrep -af "$pattern"'|gz sim|px4'
      return 1
    fi
  fi
  ros2 daemon stop > /dev/null 2>&1
  return 0
}

o2_wait_px4() {
  local deadline=$((SECONDS + 420))
  until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
      | grep -q '/fmu/out/vehicle_odometry'; do
    if [ $SECONDS -gt $deadline ]; then echo "FATAL: PX4 never published"; return 1; fi
    sleep 10
  done
  echo "PX4 publishing after ${SECONDS}s"
}

# o2_wait_nodes NAME [NAME ...] -- own deadline PER node (verify_mission.sh
# bug: one shared deadline starves nodes late in a long list).
o2_wait_nodes() {
  for node in "$@"; do
    local deadline=$((SECONDS + 90))
    until timeout 15 ros2 node list --no-daemon 2>/dev/null | grep -q "$node"; do
      if [ $SECONDS -gt $deadline ]; then
        echo "FATAL: $node never appeared in ros2 node list"
        timeout 15 ros2 node list --no-daemon 2>/dev/null
        return 1
      fi
      sleep 3
    done
  done
  echo "all requested nodes confirmed: $*"
}

# o2_start_stack MODEL WORLD PERCEPTION BLACKBOX DIAGNOSTICS EVENT_LOG LOGNAME
# -> echoes O2_LOG=<bringup log path> on success.
o2_start_stack() {
  local model="$1" world="$2" perception="$3" blackbox="$4" diagnostics="$5"
  local event_log="$6" logname="$7"
  UAV_MODEL="$model" UAV_WORLD="$world" bash "$WORKSPACE_WSL/scripts/start_sim.sh" 2>&1 | tail -3
  o2_wait_px4 || return 1

  local log="$LOGDIR/$logname"
  setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$O2_UAV_ID" \
    perception:="$perception" mission:=false \
    blackbox:="$blackbox" diagnostics:="$diagnostics" event_log:="$event_log" \
    > "$log" 2>&1 < /dev/null &

  local nodes=(control_authority_manager_node safety_supervisor_node
    navigator_action_server_node route_planner_node localization_mux_node)
  [ "$perception" = "true" ] && nodes+=(marker_detector_node obstacle_extractor_node
    target_tracker_node object_detector_node world_model_node)
  [ "$blackbox" = "true" ] && nodes+=(rosbag_manager_node)
  [ "$diagnostics" = "true" ] && nodes+=(diagnostics_node)
  [ "$event_log" = "true" ] && nodes+=(event_logger_node)

  o2_wait_nodes "${nodes[@]}" || { echo "--- bringup log tail ---"; tail -40 "$log"; return 1; }
  echo "O2_LOG=$log"
}

# o2_rtf_mean FILE -> "mean n" from a gz-topic-stats capture (ops-playbook
# S2: mean is the stable reading, RTF under render load is bimodal so
# median flip-flops between modes run to run).
o2_rtf_mean() {
  awk '{v[NR]=$1; s+=$1} END {if (!NR) {print "NaN 0"; exit} printf "%.4f %d\n", s/NR, NR}' \
    <(grep real_time_factor "$1" | awk '{print $2}')
}

# o2_run_flight TAG ITER BLACKBOX DIAGNOSTICS EVENT_LOG LOAD_PROCS CSV
# TAG in {ON,OFF,LOAD} -- purely a label for the CSV/logs.
o2_run_flight() {
  local tag="$1" iter="$2" blackbox="$3" diagnostics="$4" event_log="$5"
  local load_procs="$6" csv="$7"
  echo "############################################################"
  echo "=== G-O2 flight: tag=$tag iter=$iter blackbox=$blackbox diagnostics=$diagnostics event_log=$event_log load=$load_procs ==="
  echo "############################################################"
  unset ROS_DOMAIN_ID
  o2_teardown || {
    echo "$tag,$iter,FAILED_TEARDOWN,,,,,,,,,,,,,,,,," >> "$csv"; bump 1; return
  }

  # CHẶN C7 lỗ 4: wipe stale bag dirs BEFORE this round's stack starts, so
  # the "newest by mtime" lookup below can never resolve to a PRIOR round's
  # bag when this round's recorder writes nothing (report would otherwise
  # see bag_bytes>0 and think this round recorded, when it didn't).
  rm -rf "$HOME/uav_bags" 2>/dev/null

  local out rc
  out=$(o2_start_stack "$O2_MODEL" "$O2_WORLD" false "$blackbox" "$diagnostics" "$event_log" \
    "o2_${tag}_${iter}_bringup.log")
  rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "FATAL: stack never came up for $tag#$iter"
    echo "$tag,$iter,FAILED_BRINGUP,,,,,,,,,,,,,,,,," >> "$csv"
    bump 1; o2_teardown; return
  fi

  local load_pids=()
  if [ "$load_procs" -gt 0 ] 2>/dev/null; then
    echo "--- positive control: launching $load_procs busy-loop load processes (nproc=$(nproc)) ---"
    local i
    for i in $(seq 1 "$load_procs"); do
      ( while :; do :; done ) &
      load_pids+=($!)
    done
  fi

  local rb_pid di_pid ev_pid
  rb_pid=$(pgrep -f 'install/uav_observability/lib/uav_observability/rosbag_manager_node' | head -1)
  di_pid=$(pgrep -f 'install/uav_observability/lib/uav_observability/diagnostics_node' | head -1)
  ev_pid=$(pgrep -f 'install/uav_observability/lib/uav_observability/event_logger_node' | head -1)

  local probe_json="/tmp/o2_probe_${tag}_${iter}.json"
  python3 "$LATENCY_PROBE" --uav-id "$O2_UAV_ID" --duration "$O2_PROBE_DURATION" \
    --output "$probe_json" > "$LOGDIR/o2_probe_${tag}_${iter}.log" 2>&1 &
  local probe_pid=$!

  local rtf_file="/tmp/o2_rtf_${tag}_${iter}.log"
  timeout 120 gz topic -e -t "/world/$O2_WORLD/stats" -n "$O2_RTF_SAMPLES" > "$rtf_file" 2>/dev/null &
  local rtf_pid=$!

  local cpu_json="/tmp/o2_cpu_${tag}_${iter}.json"
  local cpu_pid=""
  local cpu_targets=""
  [ -n "$rb_pid" ] && cpu_targets="$cpu_targets $rb_pid"
  [ -n "$di_pid" ] && cpu_targets="$cpu_targets $di_pid"
  [ -n "$ev_pid" ] && cpu_targets="$cpu_targets $ev_pid"
  if [ -n "$cpu_targets" ]; then
    # shellcheck disable=SC2086
    python3 "$CPU_SAMPLER" $cpu_targets --duration "$O2_CPU_DURATION" --interval 1 \
      --output "$cpu_json" > "$LOGDIR/o2_cpu_${tag}_${iter}.log" 2>&1 &
    cpu_pid=$!
  fi

  local flight_log="$LOGDIR/o2_flight_${tag}_${iter}.log"
  ros2 run uav_bringup smoke_flight.py --flights "$O2_FLIGHTS" --uav-id "$O2_UAV_ID" \
    2>&1 | tee "$flight_log" | tail -30
  local flight_status=${PIPESTATUS[0]}

  kill -INT "$probe_pid" 2>/dev/null
  wait "$probe_pid" 2>/dev/null
  wait "$rtf_pid" 2>/dev/null
  if [ -n "$cpu_pid" ]; then
    kill -TERM "$cpu_pid" 2>/dev/null
    wait "$cpu_pid" 2>/dev/null
  fi
  if [ "$load_procs" -gt 0 ] 2>/dev/null; then
    kill "${load_pids[@]}" 2>/dev/null
    wait 2>/dev/null
  fi

  local rtf_mean rtf_n
  read -r rtf_mean rtf_n <<< "$(o2_rtf_mean "$rtf_file")"

  local p50 p95 p99 pmax n_used n_excl
  if [ -f "$probe_json" ]; then
    read -r p50 p95 p99 pmax n_used n_excl <<< "$(python3 -c "
import json
d = json.load(open('$probe_json'))
s = d.get('stats') or {}
print(s.get('p50', float('nan')), s.get('p95', float('nan')), s.get('p99', float('nan')),
      s.get('max', float('nan')), s.get('n_intervals_used', 0), s.get('n_gaps_excluded', 0))
")"
  else
    p50=NaN; p95=NaN; p99=NaN; pmax=NaN; n_used=0; n_excl=0
  fi

  local cpu_rb_avg=NA cpu_rb_peak=NA cpu_di_avg=NA cpu_di_peak=NA cpu_ev_avg=NA cpu_ev_peak=NA
  local died_mid_sample=false
  if [ -f "$cpu_json" ]; then
    # CHẶN C7 lỗ 1: died_mid_sample was already computed by o2_cpu_sampler.py
    # (proc death mid-round -- p99/CPU for that round are NOT trustworthy)
    # but was previously read here and thrown away. Now threaded to the CSV
    # so round_o2_report can FTM on it instead of silently PASSing a round
    # where a monitored node crashed mid-flight while pgrep still saw a PID.
    read -r cpu_rb_avg cpu_rb_peak cpu_di_avg cpu_di_peak cpu_ev_avg cpu_ev_peak died_mid_sample <<< "$(python3 -c "
import json
d = json.load(open('$cpu_json'))
def get(pid):
    e = d.get(str(pid)) if pid else None
    if not e or e.get('avg_pct') is None:
        return 'NA', 'NA', bool(e and e.get('died_mid_sample'))
    return '%.2f' % e['avg_pct'], '%.2f' % e['peak_pct'], bool(e.get('died_mid_sample'))
out = []
died = False
for pid in ('$rb_pid', '$di_pid', '$ev_pid'):
    avg, peak, d_flag = get(pid)
    out += [avg, peak]
    died = died or d_flag
print(*out, str(died).lower())
")"
  fi

  local bag_bytes=NA bag_dir=NA
  if [ "$blackbox" = "true" ]; then
    bag_dir=$(find ~/uav_bags -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null \
      | sort -rn | head -1 | cut -d' ' -f2-)
    [ -n "$bag_dir" ] && bag_bytes=$(du -sb "$bag_dir" 2>/dev/null | cut -f1)
  fi

  echo "$tag,$iter,$flight_status,$rtf_mean,$rtf_n,$p50,$p95,$p99,$pmax,$n_used,$n_excl,$cpu_rb_avg,$cpu_rb_peak,$cpu_di_avg,$cpu_di_peak,$cpu_ev_avg,$cpu_ev_peak,$died_mid_sample,$bag_bytes,$bag_dir" \
    >> "$csv"
  echo "[$tag#$iter] flight=$flight_status rtf_mean=$rtf_mean(n=$rtf_n) p50=$p50 p95=$p95 p99=$p99 max=$pmax used=$n_used excl=$n_excl died_mid_sample=$died_mid_sample bag_bytes=$bag_bytes"
  [ "$flight_status" -ne 0 ] && bump 1

  o2_teardown
}

# ------------------------------------------------------------- o2-main (x6)
round_o2_main() {
  echo "tag,iter,flight_status,rtf_mean,rtf_n,p50,p95,p99,max,n_used,n_excluded,cpu_rb_avg,cpu_rb_peak,cpu_di_avg,cpu_di_peak,cpu_ev_avg,cpu_ev_peak,died_mid_sample,bag_bytes,bag_dir" \
    > "$O2_MAIN_CSV"
  local iter
  for iter in 1 2 3; do
    # ON immediately followed by OFF each iteration -- interleaved
    # ON/OFF/ON/OFF/ON/OFF per the R31-adjacent instruction, NOT 3 ON then
    # 3 OFF (thermal/cache drift over a long run must not alias with the
    # ON/OFF effect being measured).
    o2_run_flight ON "$iter" true true true 0 "$O2_MAIN_CSV"
    o2_run_flight OFF "$iter" false false false 0 "$O2_MAIN_CSV"
  done
}

# --------------------------------------------------- o2-control (R27-3, x1)
round_o2_control() {
  echo "tag,iter,flight_status,rtf_mean,rtf_n,p50,p95,p99,max,n_used,n_excluded,cpu_rb_avg,cpu_rb_peak,cpu_di_avg,cpu_di_peak,cpu_ev_avg,cpu_ev_peak,died_mid_sample,bag_bytes,bag_dir" \
    > "$O2_CONTROL_CSV"
  local load=$(( $(nproc) - 6 ))
  [ "$load" -lt 4 ] && load=4
  echo "positive control: load_procs=$load busy loops (nproc=$(nproc)) -- proves the p99 probe can bite (R27-3)"
  o2_run_flight LOAD 1 true true true "$load" "$O2_CONTROL_CSV"
}

# ---------------------------------------- o2-extra: top up n after a noisy RTF read
# APPENDS (does not truncate) $O2_MAIN_CSV -- for when the first 3 ON/3 OFF
# pairs show a wide RTF spread and more samples are needed to tell a real
# ON/OFF effect apart from WSL/render run-to-run noise (ops-playbook S2).
round_o2_extra() {
  [ -f "$O2_MAIN_CSV" ] || { echo "FATAL: $O2_MAIN_CSV missing -- run o2-main first"; bump 1; return; }
  local last_iter
  last_iter=$(awk -F, 'NR>1 && $1=="ON" {print $2}' "$O2_MAIN_CSV" | sort -n | tail -1)
  [ -z "$last_iter" ] && last_iter=0
  local iter
  for iter in $((last_iter + 1)) $((last_iter + 2)); do
    o2_run_flight ON "$iter" true true true 0 "$O2_MAIN_CSV"
    o2_run_flight OFF "$iter" false false false 0 "$O2_MAIN_CSV"
  done
}

# ------------------------------------------ o2-hz: flight_only hz debt (P10.4)
round_o2_hz() {
  echo "############################################################"
  echo "=== G-O2 debt: flight_only hz survey (perception:=true, $O2_HZ_MODEL + target box) ==="
  echo "############################################################"
  unset ROS_DOMAIN_ID
  o2_teardown || { bump 1; return; }

  local out rc
  out=$(o2_start_stack "$O2_HZ_MODEL" "$O2_WORLD" true true true true o2_hz_bringup.log)
  rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "FATAL: stack never came up for hz survey"; bump 1; o2_teardown; return
  fi

  bash "$WORKSPACE_WSL/scripts/spawn_target_box.sh" "$O2_WORLD" 3.0 0.0 0.2 0.0 o2_hz_target_box
  sleep 2

  # perception/detections has NO source in this arena: object_detector_node
  # is an HOG PEOPLE detector (object_detector_node.cpp) and no person/actor
  # model exists anywhere in this repo to spawn -- kept in the list on
  # purpose so its absence is measured and reported, not silently dropped.
  local topics=(
    "control/command_selected"
    "world/obstacle_map_local"
    "world/target_state"
    "perception/obstacles_local"
    "perception/target_track"
    "perception/detections"
    "state/camera_health"
    "diagnostics/perception"
  )
  local pids=() logs=() t safe hzlog
  for t in "${topics[@]}"; do
    safe="${t//\//_}"
    hzlog="/tmp/o2_hz_${safe}.log"
    timeout 150 ros2 topic hz "/uav/$O2_UAV_ID/$t" > "$hzlog" 2>&1 &
    pids+=($!)
    logs+=("$hzlog")
  done

  ros2 run uav_bringup smoke_flight.py --flights "$O2_FLIGHTS" --uav-id "$O2_UAV_ID" \
    2>&1 | tee "$LOGDIR/o2_hz_flight.log" | tail -30
  local flight_status=${PIPESTATUS[0]}

  wait "${pids[@]}" 2>/dev/null

  {
    echo "=== G-O2 flight_only hz survey (flight_status=$flight_status, model=$O2_HZ_MODEL) ==="
    local i=0
    for t in "${topics[@]}"; do
      echo "--- /uav/$O2_UAV_ID/$t ---"
      tail -5 "${logs[$i]}"
      i=$((i + 1))
    done
  } | tee "$O2_HZ_LOG"

  [ "$flight_status" -ne 0 ] && bump 1
  o2_teardown
}

# ---------------------------------------------------------- o2-report
round_o2_report() {
  echo "############################################################"
  echo "=== G-O2 report/verdict ==="
  echo "############################################################"
  [ -f "$O2_MAIN_CSV" ] || { echo "FATAL: $O2_MAIN_CSV missing -- run o2-main first"; return 2; }
  python3 -c "
import csv
import statistics
import sys

def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')

def nz(vals):
    # R27-2: NaN must never survive a filter meant to drop unmeasured values.
    return [v for v in vals if v == v]

rows = list(csv.DictReader(open('$O2_MAIN_CSV')))
on = [r for r in rows if r['tag'] == 'ON']
off = [r for r in rows if r['tag'] == 'OFF']

def col(rows_, key):
    return [fnum(r[key]) for r in rows_]

verdict = [0]

def check(name, cond, detail):
    status = 'PASS' if cond else 'FAIL'
    if not cond:
        verdict[0] = max(verdict[0], 1)
    print('%-42s %-4s  %s' % (name, status, detail))

if len(on) < 3 or len(off) < 3:
    print('FAILED TO MEASURE: expected >=3 ON and >=3 OFF rows in %s, got %d/%d'
          % ('$O2_MAIN_CSV', len(on), len(off)))
    sys.exit(2)

print('--- run order (proves interleaving, not 3-then-3) ---')
for r in rows:
    print('  %s#%s rtf=%s p99=%s flight_status=%s' % (r['tag'], r['iter'], r['rtf_mean'], r['p99'], r['flight_status']))
print()

# CHẶN C7: measurement-integrity precheck BEFORE any statistic is trusted
# (R27-1). A recorder that hit write_error_limit/min_free_bytes mid-flight
# stays alive (pgrep still sees it, CPU sampler still ticks) but STOPS
# writing -- p99(ON)~=p99(OFF) and low CPU would then look identical to
# 'no black box running', turning a degraded/disabled recorder into a false
# 'blackbox is free' PASS. Any of these signals means the round did not
# actually measure a live-recording black box.
integrity_problems = []
for r in on + off:
    tag_iter = '%s#%s' % (r['tag'], r['iter'])
    fs = fnum(r['flight_status'])
    if fs != 0:
        integrity_problems.append('%s: flight_status=%s (expected 0)' % (tag_iter, r['flight_status']))
for r in on:
    tag_iter = '%s#%s' % (r['tag'], r['iter'])
    bb = fnum(r['bag_bytes'])
    if not (bb > 0):
        integrity_problems.append(
            '%s: bag_bytes=%s (blackbox=true but nothing on disk -- recorder alive but not '
            'writing looks identical to DEGRADED/DISABLED-NO-SPACE)' % (tag_iter, r['bag_bytes']))
    dms = str(r.get('died_mid_sample', '')).strip().lower()
    if dms == 'true':
        integrity_problems.append(
            '%s: died_mid_sample=true (a monitored node crashed during CPU sampling -- '
            'p99/CPU for this round are not trustworthy)' % tag_iter)

if integrity_problems:
    print('FAILED TO MEASURE: %d row(s) failed the ON/OFF measurement-integrity precheck (R27-1):'
          % len(integrity_problems))
    for p in integrity_problems:
        print('  - %s' % p)
    sys.exit(2)

rtf_on_vals, rtf_off_vals = nz(col(on, 'rtf_mean')), nz(col(off, 'rtf_mean'))
if not rtf_on_vals or not rtf_off_vals:
    print('FAILED TO MEASURE: rtf_mean all-NaN in %s rows (n_on=%d n_off=%d)'
          % ('ON' if not rtf_on_vals else 'OFF', len(rtf_on_vals), len(rtf_off_vals)))
    sys.exit(2)
rtf_on, rtf_off = statistics.median(rtf_on_vals), statistics.median(rtf_off_vals)
check('RTF |median(ON)-median(OFF)| <= 0.02', abs(rtf_on - rtf_off) <= 0.02,
      'ON=%.4f(n=%d) OFF=%.4f(n=%d) delta=%.4f'
      % (rtf_on, len(rtf_on_vals), rtf_off, len(rtf_off_vals), rtf_on - rtf_off))

p99_on_vals, p99_off_vals = nz(col(on, 'p99')), nz(col(off, 'p99'))
if not p99_on_vals or not p99_off_vals:
    print('FAILED TO MEASURE: p99 all-NaN in %s rows (n_on=%d n_off=%d)'
          % ('ON' if not p99_on_vals else 'OFF', len(p99_on_vals), len(p99_off_vals)))
    sys.exit(2)
p99_on, p99_off = statistics.median(p99_on_vals), statistics.median(p99_off_vals)
check('p99 median(ON)-median(OFF) <= 5ms', (p99_on - p99_off) <= 0.005,
      'ON=%.4f s(n=%d) OFF=%.4f s(n=%d) delta=%.2f ms'
      % (p99_on, len(p99_on_vals), p99_off, len(p99_off_vals), (p99_on - p99_off) * 1000))

# 3/3 nodes, not 'any 1 of 9 cells' -- the contract says PER observability
# node, and the old check passed on a single valid cell out of 9.
cpu_nodes = [('rosbag_manager', 'cpu_rb_avg', 'cpu_rb_peak'),
             ('diagnostics', 'cpu_di_avg', 'cpu_di_peak'),
             ('event_logger', 'cpu_ev_avg', 'cpu_ev_peak')]
node_peak = {label: nz(col(on, peak_c)) for label, _, peak_c in cpu_nodes}
node_avg = {label: nz(col(on, avg_c)) for label, avg_c, _ in cpu_nodes}
missing = [label for label, vals in node_peak.items() if not vals]
if missing:
    print('FAILED TO MEASURE: CPU peak has zero valid samples for node(s) %s across all ON rows '
          '(need 3/3 observability nodes measured, not just any single cell -- R27-1)' % missing)
    sys.exit(2)
worst_peak = max(v for vals in node_peak.values() for v in vals)
all_avgs = [v for vals in node_avg.values() for v in vals]
check('CPU per observability node < 1 core (100%) peak (3/3 nodes measured)', worst_peak < 100.0,
      'rb_peak=%.1f%% di_peak=%.1f%% ev_peak=%.1f%% mean_of_avgs=%.1f%%'
      % (max(node_peak['rosbag_manager']), max(node_peak['diagnostics']), max(node_peak['event_logger']),
         (sum(all_avgs) / len(all_avgs)) if all_avgs else float('nan')))

print()
print('--- bag size (ON rows) ---')
for r in on:
    print('  #%s bag_bytes=%s dir=%s' % (r['iter'], r['bag_bytes'], r['bag_dir']))

try:
    control_rows = list(csv.DictReader(open('$O2_CONTROL_CSV')))
except FileNotFoundError:
    control_rows = None

print()
if control_rows is None:
    print('positive control (R27-3): NOT RUN (run o2-control first) -- p99 gate NOT fully trusted')
    verdict[0] = max(verdict[0], 2)
else:
    load_p99 = nz([fnum(r['p99']) for r in control_rows if r['tag'] == 'LOAD'])
    if not load_p99:
        print('positive control (R27-3): FAILED TO MEASURE (no usable LOAD row)')
        verdict[0] = max(verdict[0], 2)
    else:
        bite = max(load_p99) - p99_on
        check('positive control: p99(LOAD) - p99(ON) >= 10ms (probe bites)', bite >= 0.010,
              'p99(LOAD)=%.4f s p99(ON median)=%.4f s delta=%.1f ms' % (max(load_p99), p99_on, bite * 1000))

sys.exit(verdict[0])
"
  return $?
}

# --------------------------------------------- o2-m5 (standalone, post-yaml)
round_o2_m5() {
  echo "############################################################"
  echo "=== M5 regression, ON config (sim.launch.py defaults) -- confirms observability_params.yaml edit didn't break anything ==="
  echo "############################################################"
  unset ROS_DOMAIN_ID
  o2_teardown || { bump 1; return; }
  UAV_MODEL=uav0 UAV_WORLD=uav_arena bash "$WORKSPACE_WSL/scripts/start_sim.sh" 2>&1 | tail -3
  o2_wait_px4 || { bump 1; return; }
  local log="$LOGDIR/o2_m5_after_yaml.log"
  setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$O2_UAV_ID" \
    > "$log" 2>&1 < /dev/null &
  sleep 25
  grep -q "diagnostics_node ready for" "$log" || {
    echo "FATAL: diagnostics_node never came up (yaml edit may violate V-O1..V-O4, R33)"
    tail -50 "$log"; bump 1; o2_teardown; return
  }
  echo "--- binary this run flies (R27-4: confirm which build the flight proves) ---"
  ls -l "$WORKSPACE_WSL/install/uav_observability/lib/uav_observability/diagnostics_node"
  ros2 run uav_bringup smoke_flight.py --flights 3 --uav-id "$O2_UAV_ID" \
    2>&1 | tee "$LOGDIR/o2_m5_after_yaml_smoke.log" | tail -40
  local v=${PIPESTATUS[0]}
  bump "$v"
  echo "G-O2 M5-confirm verdict: $v"
  o2_teardown
}

# ============================================================ P10.8b -- G-O3
# Task A (o3-camera-health): wire-check for camera_health_node, which was
# BUILT (uav_perception/CMakeLists.txt) but never added to sim.launch.py's
# PERCEPTION_NODES -- state/camera_health and diagnostics/perception had
# ZERO publishers in every bringup config (found during this same gate's
# prep, see observability_params.yaml's diagnostics_node comment block).
# Task B (o3-flight): one indoor_patrol flight, perception:=true, all 3
# observability nodes on (sim.launch.py defaults) -- doubles as the G-M1
# regression for wiring camera_health_node in. Produces the bag+JSONL that
# Tasks C/D consume.
# Task C (o3-replay): replay that bag on ROS_DOMAIN_ID 99 (empty domain,
# R20/Q-P10-8) and compare a live subscriber's counts against a DIRECT
# rosbag2_py read of the same bag (o3_replay_gate.py reference/live/compare).
# Task D (o3-loop / o3-loop-control): standalone control_authority_manager_
# node on domain 99, fed the SAME bag's cmd_* traffic via `ros2 bag play
# --clock [--loop]`. ⚠️ Retired as evidence for the N-c mechanism itself
# (G-O3(c) CHOT 2026-08-24, see the paragraph below and contract Sec2.20's
# last row) -- kept as a liveness/wiring watch only (checks 1-3 in
# round_o3_loop_report), NOT as a source of clock_regressions verdicts.
# Task D' (o3-synth, P10.8c/G-O3(c)): --loop on Task B's bag is NOT valid
# evidence for "the N-c fix bites a real regression" through `ros2 bag play
# --clock` -- but WHY changed twice as the investigation went deeper (both
# earlier explanations below are SUPERSEDED; kept so nobody re-derives them
# from scratch and mistakes either for the current answer):
#   (1) P10.8c's ORIGINAL claim ("MISSION legitimately releases via
#       release_dwell_sec before the bag ends, so there is no channel left
#       holding a stale timestamp for ageSecOrInf to catch") was refuted by
#       re-reading that SAME run's own logs (P10.9a): the arbiter never
#       received a single LIVE cmd_mission that whole session -- a double-
#       writer + QoS-incompatible replay of command_selected/authority (bug
#       C3) meant the observed NONE<->MISSION toggling was the bag replaying
#       its OWN old command_selected samples, not the arbiter's live state.
#   (2) After fixing C3 (--topics excludes command_selected/authority) and
#       re-running (P10.9a), a NEW anomaly appeared: clock_regressions grew
#       on almost EVERY cmd_mission message, --loop or not. CHOT 2026-08-24
#       (G-O3(c)): `ros2 bag play --clock` synthesises /clock as a DISCRETE
#       ~40.03Hz tick, not phase-locked to cmd_mission's own ~20Hz publish
#       schedule -- 29/30 sampled (now, header.stamp) pairs came out
#       negative, magnitude matching the ~25ms tick period almost exactly.
#       `now_sec < stored_stamp` fires from ordinary REPLAY-TOOL RESOLUTION
#       noise, unrelated to any real backward jump -- this is why
#       round_o3_loop_report's old checks 4/5 are retired to [INFO]-only.
# o3_clock_regression_synth.py sidesteps ALL of the above by driving a
# standalone real control_authority_manager_node with its OWN /clock +
# cmd_mission publishers (never goes through `ros2 bag play`), deliberately
# holding MISSION active and THEN silencing it across a real jump -- it IS
# able to observe clock_regressions increase on the same real deployed node
# (reproduced twice: 0->26), so it remains the ONLY deterministic gate for
# the N-c mechanism itself; see docs/interface-contract-v0.1.md Sec2.20's
# G-O3(c) CHOT row (2026-08-24) for the full chain of evidence.
# Reuses o2_teardown/o2_wait_px4/o2_wait_nodes/o2_start_stack for the two
# real-Gazebo tasks (A/B); C/D/D' are pure-ROS domain 99, own light teardown.

O3_UAV_ID=uav0
O3_A_MODEL=uav0_full         # has BOTH front+down cameras -- real hz survey (Task A)
O3_A_WORLD=uav_arena
O3_B_MODEL=uav0_nav_indoor   # SAME combo G-M1 always uses -- Task B doubles as its regression
O3_B_WORLD=uav_arena_indoor
MISSION_PROBE="$WORKSPACE_WIN/src/uav_mission/test/mission_gate.py"
O3_REPLAY_PROBE="$WORKSPACE_WIN/src/uav_observability/test/o3_replay_gate.py"
O3_SYNTH_PROBE="$WORKSPACE_WIN/src/uav_observability/test/o3_clock_regression_synth.py"
O3_CA_PARAMS="$WORKSPACE_WIN/src/uav_control_authority/config/control_authority_params.yaml"
O3_OBS_PARAMS="$WORKSPACE_WIN/src/uav_observability/config/observability_params.yaml"
O3_BAG_STATE="$LOGDIR/o3_bag_dir.txt"
O3_JSONL_STATE="$LOGDIR/o3_jsonl_path.txt"
O3_SYNTH_EXIT_STATE="$LOGDIR/o3_synth_exit_code.txt"
# P10.9a G-O3(c) rerun ("Bay 1"): the arbiter's own 4 INPUT topics only --
# NEVER command_selected/authority (those are the arbiter's OWN single-writer
# OUTPUT; replaying them creates a second publisher on an exclusive-writer
# topic, contract Sec2.20's C3 root-cause row -- "2 publishers on .../
# command_selected, only this node may write it" logged ~60x during the old
# full-bag --loop run). This is the fix for that finding, not a new bug.
O3_ARBITER_INPUT_TOPICS=(
  "/uav/$O3_UAV_ID/control/cmd_mission" "/uav/$O3_UAV_ID/control/cmd_safety"
  "/uav/$O3_UAV_ID/control/cmd_operator" "/uav/$O3_UAV_ID/control/cmd_test"
)

for f in "$MISSION_PROBE" "$O3_REPLAY_PROBE" "$O3_SYNTH_PROBE" "$O3_CA_PARAMS" "$O3_OBS_PARAMS"; do
  [ -f "$f" ] || { echo "FATAL: $f missing"; exit 1; }
done

# o3_stop_pid PID LABEL GRACE_SEC -- SIGINT then a BOUNDED wait, SIGKILL on
# timeout. Found live running this gate: POSIX sets SIGINT/SIGQUIT to be
# IGNORED for asynchronous (`&`) children of a NON-interactive script
# (this whole file, always run via `bash script.sh`, never sourced
# interactively) -- rclcpp's own signal handler installation does not
# reliably override that inherited disposition, so a bare `kill -INT
# $pid; wait $pid` on a standalone `ros2 run ...` node can block forever.
# python3 probes here DO explicitly call signal.signal(SIGINT, ...), which
# DOES override an inherited ignore -- but this helper is used uniformly so
# no caller has to reason about which child type is which.
o3_stop_pid() {
  local pid="$1" label="$2" grace="${3:-10}"
  kill -INT "$pid" 2>/dev/null
  local deadline=$((SECONDS + grace))
  while kill -0 "$pid" 2>/dev/null; do
    if [ $SECONDS -gt $deadline ]; then
      echo "$label ($pid) did not exit within ${grace}s of SIGINT -- SIGKILL"
      kill -9 "$pid" 2>/dev/null
      break
    fi
    sleep 0.5
  done
  wait "$pid" 2>/dev/null
}

# ---------------------------------------------------- o3-camera-health (Task A)
round_o3_camera_health() {
  echo "############################################################"
  echo "=== G-O3 Task A: camera_health_node wire-check ($O3_A_MODEL/$O3_A_WORLD, perception:=true) ==="
  echo "############################################################"
  unset ROS_DOMAIN_ID
  o2_teardown || { bump 1; return; }

  local out rc
  out=$(o2_start_stack "$O3_A_MODEL" "$O3_A_WORLD" true true true true o3_camera_health_bringup.log)
  rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "FATAL: stack never came up for camera_health wire-check"
    bump 1; o2_teardown; return
  fi
  # o2_start_stack's own node list does NOT include camera_health_node
  # (that omission predates this gate) -- confirm it separately.
  o2_wait_nodes camera_health_node || { bump 1; o2_teardown; return; }

  echo "--- ros2 node info /camera_health_node (publishers must be ONLY its own topics) ---"
  timeout 15 ros2 node info /camera_health_node --no-daemon 2>&1 \
    | tee "$LOGDIR/o3_camera_health_node_info.log"

  local hz_health="/tmp/o3_hz_camera_health.log" hz_diag="/tmp/o3_hz_diag_perception.log"
  timeout 20 ros2 topic hz "/uav/$O3_UAV_ID/state/camera_health" > "$hz_health" 2>&1 &
  local hz1_pid=$!
  timeout 20 ros2 topic hz "/uav/$O3_UAV_ID/diagnostics/perception" > "$hz_diag" 2>&1 &
  local hz2_pid=$!
  wait "$hz1_pid" "$hz2_pid" 2>/dev/null

  echo "--- state/camera_health hz ---"; tail -5 "$hz_health"
  echo "--- diagnostics/perception hz ---"; tail -5 "$hz_diag"
  cp "$hz_health" "$hz_diag" "$LOGDIR/" 2>/dev/null

  local avg1 avg2
  avg1=$(grep -oP 'average rate: \K[0-9.]+' "$hz_health" | tail -1)
  avg2=$(grep -oP 'average rate: \K[0-9.]+' "$hz_diag" | tail -1)
  if [ -z "$avg1" ] || [ -z "$avg2" ]; then
    echo "FAILED TO MEASURE: no hz reading on one or both topics (0 publishers reaching the tool?)"
    bump 2
  else
    echo "MEASURED_HZ camera_health=$avg1 diagnostics_perception=$avg2"
  fi
  o2_teardown
}

# ------------------------------------------------------------- o3-flight (Task B)
round_o3_flight() {
  echo "############################################################"
  echo "=== G-O3 Task B: indoor_patrol w/ perception:=true -- bag+JSONL evidence + G-M1 regression ==="
  echo "############################################################"
  unset ROS_DOMAIN_ID
  o2_teardown || { bump 1; return; }

  UAV_MODEL="$O3_B_MODEL" UAV_WORLD="$O3_B_WORLD" bash "$WORKSPACE_WSL/scripts/start_sim.sh" 2>&1 | tail -3
  o2_wait_px4 || { bump 1; return; }

  local log="$LOGDIR/o3_flight_bringup.log"
  setsid nohup ros2 launch uav_bringup sim.launch.py uav_id:="$O3_UAV_ID" \
    perception:=true mission:=true \
    > "$log" 2>&1 < /dev/null &
  sleep 25
  grep -q "mission_executor_node ready for" "$log" || {
    echo "FATAL: mission_executor_node never came up"; tail -40 "$log"; bump 1; o2_teardown; return
  }

  o2_wait_nodes mission_executor_node navigator_action_server_node route_planner_node \
    control_authority_manager_node safety_supervisor_node camera_health_node \
    marker_detector_node obstacle_extractor_node target_tracker_node object_detector_node \
    world_model_node rosbag_manager_node diagnostics_node event_logger_node \
    || { bump 1; o2_teardown; return; }

  python3 -u "$MISSION_PROBE" --bringup-log "$log" patrol --uav-id "$O3_UAV_ID" \
    2>&1 | tee "$LOGDIR/o3_flight_probe.log" | tail -80
  local v=${PIPESTATUS[0]}
  bump "$v"
  echo "G-O3 Task B (== G-M1 w/ perception:=true) verdict: $v"

  # o2_teardown -> stop_sim.sh SIGINTs uav_observability BEFORE SIGTERM'ing
  # everything else (plan S:6d fix) -- bag/JSONL close clean.
  o2_teardown

  local bag_dir jsonl_path
  bag_dir=$(find ~/uav_bags -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null \
    | sort -rn | head -1 | cut -d' ' -f2-)
  jsonl_path=$(find ~/uav_events -maxdepth 1 -name '*_events.jsonl' -printf '%T@ %p\n' 2>/dev/null \
    | sort -rn | head -1 | cut -d' ' -f2-)
  if [ -z "$bag_dir" ] || [ -z "$jsonl_path" ]; then
    echo "FAILED TO MEASURE: bag_dir='$bag_dir' jsonl_path='$jsonl_path' -- one or both missing"
    bump 2; return
  fi
  echo "$bag_dir" > "$O3_BAG_STATE"
  echo "$jsonl_path" > "$O3_JSONL_STATE"
  echo "O3_BAG_DIR=$bag_dir"
  echo "O3_JSONL_PATH=$jsonl_path"
}

# ------------------------------------------------------------- o3-replay (Task C)
round_o3_replay() {
  echo "############################################################"
  echo "=== G-O3 Task C: replay-vs-original (bag from Task B) ==="
  echo "############################################################"
  [ -f "$O3_BAG_STATE" ] || { echo "FATAL: $O3_BAG_STATE missing -- run o3-flight first"; bump 1; return; }
  local bag_dir; bag_dir=$(cat "$O3_BAG_STATE")
  [ -d "$bag_dir" ] || { echo "FATAL: bag dir '$bag_dir' does not exist"; bump 1; return; }

  unset ROS_DOMAIN_ID
  pkill -f 'o3_replay_gate.py' 2>/dev/null
  pkill -f 'ros2 bag play' 2>/dev/null
  pkill -f 'install/uav_observability/lib/uav_observability/diagnostics_node' 2>/dev/null
  sleep 1

  echo "--- ros2 bag info (direct) ---"
  local reindexed=false
  if ! timeout 20 ros2 bag info "$bag_dir" > "$LOGDIR/o3_replay_baginfo_direct.log" 2>&1; then
    echo "direct 'bag info' failed -- reindexing (contract Sec2.20: expected after an unclean close)"
    ros2 bag reindex "$bag_dir" > "$LOGDIR/o3_replay_reindex.log" 2>&1
    reindexed=true
    timeout 20 ros2 bag info "$bag_dir" > "$LOGDIR/o3_replay_baginfo_after_reindex.log" 2>&1 || {
      echo "FAILED TO MEASURE: bag unreadable even after reindex"; bump 2; return
    }
  fi
  echo "reindexed=$reindexed"
  tail -30 "$LOGDIR"/o3_replay_baginfo_*.log 2>/dev/null

  # Sized off the bag's OWN recorded duration -- a fixed guess (first attempt
  # at this gate used 200s against a 248s bag and truncated the replay,
  # under-counting odometry_fused by ~20%, nowhere near the real system).
  # live_duration is live's OWN --duration (self-terminating timer, started
  # ~5s before play): tightly sized to dur+30s, NOT generously oversized --
  # `kill -INT` on a python child backgrounded from this non-interactive
  # script was found live NOT reliably stopping it early (o3_stop_pid's
  # SIGKILL fallback then fires and the JSON is lost, since -9 skips the
  # write-then-exit path). The fix is to let it hit its OWN timer instead
  # of racing a signal against it -- see the plain `wait` below.
  local dur; dur=$(o3_bag_duration_sec "$bag_dir")
  [ -z "$dur" ] && dur=300
  local play_timeout live_duration
  play_timeout=$(python3 -c "print(int($dur*1.3)+30)")
  live_duration=$(python3 -c "print(int($dur)+30)")
  echo "bag duration=${dur}s -> play_timeout=${play_timeout}s live_duration=${live_duration}s"

  local ref_json="/tmp/o3_reference.json"
  python3 "$O3_REPLAY_PROBE" reference --bag-dir "$bag_dir" --storage-id sqlite3 \
    --uav-id "$O3_UAV_ID" --output "$ref_json" | tee "$LOGDIR/o3_reference.log"
  [ -f "$ref_json" ] || { echo "FAILED TO MEASURE: reference json not written"; bump 2; return; }

  export ROS_DOMAIN_ID=99
  local live_json="/tmp/o3_live.json"
  python3 "$O3_REPLAY_PROBE" live --uav-id "$O3_UAV_ID" --duration "$live_duration" \
    --output "$live_json" > "$LOGDIR/o3_live.log" 2>&1 &
  local live_pid=$!
  sleep 3   # discovery + TransientLocal delivery settle before play starts (race #967)

  local diag_log="$LOGDIR/o3_replay_diagnostics_node.log"
  # timeout wraps the node itself -- belt-and-suspenders self-bound on top
  # of o3_stop_pid's SIGKILL fallback below (see o3_stop_pid's comment).
  timeout "$((play_timeout + 60))" ros2 run uav_observability diagnostics_node \
    --ros-args --params-file "$O3_OBS_PARAMS" -p uav_id:="$O3_UAV_ID" -p use_sim_time:=true \
    > "$diag_log" 2>&1 < /dev/null &
  local diag_pid=$!
  sleep 2

  echo "--- ros2 bag play --clock (single pass) ---"
  # bag_path MUST precede --clock: `ros2 bag play --clock <dir>` greedily
  # consumes <dir> as --clock's OPTIONAL Hz value (nargs='?') and fails with
  # "invalid positive_float value" -- found live running this gate.
  timeout "$play_timeout" ros2 bag play "$bag_dir" --clock 2>&1 \
    | tee "$LOGDIR/o3_replay_play.log" | tail -20
  local play_rc=${PIPESTATUS[0]}
  echo "play exit=$play_rc"

  # live_pid: plain wait for its OWN --duration timer (see the sizing note
  # above -- SIGINT was found unreliable here, and SIGKILL would lose the
  # JSON). diag_pid: o3_stop_pid is fine -- SIGKILL losing its state is
  # harmless, nothing downstream reads diagnostics_node's own shutdown data.
  wait "$live_pid" 2>/dev/null
  o3_stop_pid "$diag_pid" "diagnostics_node"
  unset ROS_DOMAIN_ID

  [ -f "$live_json" ] || { echo "FAILED TO MEASURE: live json not written"; bump 2; return; }

  python3 "$O3_REPLAY_PROBE" compare --reference "$ref_json" --live "$live_json" \
    2>&1 | tee "$LOGDIR/o3_compare.log"
  local v=${PIPESTATUS[0]}
  bump "$v"
  echo "G-O3 Task C verdict: $v"
}

# o3_bag_duration_sec BAG_DIR -> echoes seconds (empty if unreadable, caller
# supplies a fallback default).
o3_bag_duration_sec() {
  timeout 20 ros2 bag info "$1" 2>/dev/null | grep -oP 'Duration:\s+\K[0-9.]+' | head -1
}

# o3_run_authority_watch LOOP_FLAG TAG PLAY_TIMEOUT_SEC WATCH_DURATION_SEC
o3_run_authority_watch() {
  local loop_flag="$1" tag="$2" play_timeout="$3" watch_duration="$4"
  echo "############################################################"
  echo "=== G-O3 Task D [$tag]: standalone control_authority_manager_node + bag play $([ "$loop_flag" = "true" ] && echo '--loop' || echo '(no loop)') ==="
  echo "############################################################"
  [ -f "$O3_BAG_STATE" ] || { echo "FATAL: $O3_BAG_STATE missing -- run o3-flight first"; bump 1; return; }
  local bag_dir; bag_dir=$(cat "$O3_BAG_STATE")
  [ -d "$bag_dir" ] || { echo "FATAL: bag dir '$bag_dir' does not exist"; bump 1; return; }

  unset ROS_DOMAIN_ID
  pkill -f 'control_authority_manager_node' 2>/dev/null
  pkill -f 'o3_replay_gate.py' 2>/dev/null
  pkill -f 'ros2 bag play' 2>/dev/null
  sleep 1
  export ROS_DOMAIN_ID=99

  local ca_log="$LOGDIR/o3_ca_${tag}.log"
  # timeout self-bound (o3_stop_pid's comment: SIGINT is unreliable for a
  # background child of this non-interactive script) -- generous margin
  # over play_timeout+watch_duration so it never races the real teardown.
  timeout "$((play_timeout + watch_duration))" \
    ros2 run uav_control_authority control_authority_manager_node --ros-args \
    --params-file "$O3_CA_PARAMS" -p uav_id:="$O3_UAV_ID" -p use_sim_time:=true \
    > "$ca_log" 2>&1 < /dev/null &
  local ca_pid=$!

  local deadline=$((SECONDS + 30))
  until timeout 15 ros2 node list --no-daemon 2>/dev/null | grep -q control_authority_manager_node; do
    if [ $SECONDS -gt $deadline ]; then
      echo "FATAL: control_authority_manager_node never came up"; tail -30 "$ca_log"
      kill -9 "$ca_pid" 2>/dev/null; bump 1; unset ROS_DOMAIN_ID; return
    fi
    sleep 2
  done

  local watch_json="/tmp/o3_watch_${tag}.json"
  python3 "$O3_REPLAY_PROBE" authority-watch --uav-id "$O3_UAV_ID" --duration "$watch_duration" \
    --output "$watch_json" > "$LOGDIR/o3_watch_${tag}_probe.log" 2>&1 &
  local watch_pid=$!
  sleep 3

  local play_log="$LOGDIR/o3_play_${tag}.log"
  # bag_path MUST precede --clock (see o3-replay's comment, same argparse quirk).
  # --topics restricts replay to the arbiter's 4 INPUT topics only (see
  # O3_ARBITER_INPUT_TOPICS comment above) -- this is the P10.9a fix for the
  # double-writer finding in contract Sec2.20's C3 row.
  if [ "$loop_flag" = "true" ]; then
    timeout "$play_timeout" ros2 bag play "$bag_dir" --clock --loop \
      --topics "${O3_ARBITER_INPUT_TOPICS[@]}" > "$play_log" 2>&1 &
  else
    timeout "$play_timeout" ros2 bag play "$bag_dir" --clock \
      --topics "${O3_ARBITER_INPUT_TOPICS[@]}" > "$play_log" 2>&1 &
  fi
  local play_pid=$!
  wait "$play_pid" 2>/dev/null
  echo "bag play (loop=$loop_flag, timeout=${play_timeout}s) ended"

  # Let the arbiter's own dwell/timeout decay run its course post-playback
  # (source_timeout_sec + release_dwell_sec = 0.4s total -- 5s is generous).
  sleep 5
  # watch_pid: plain wait for its OWN --duration timer, same reasoning as
  # o3-replay's live_pid (SIGINT unreliable here; SIGKILL loses the JSON).
  # watch_duration is sized to land ~15-20s after this point (caller math).
  wait "$watch_pid" 2>/dev/null
  o3_stop_pid "$ca_pid" "control_authority_manager_node"
  unset ROS_DOMAIN_ID

  if [ ! -f "$watch_json" ]; then
    echo "FAILED TO MEASURE: $watch_json not written"; bump 2; return
  fi
  cp "$watch_json" "$LOGDIR/o3_watch_${tag}.json"
  echo "--- [$tag] watch result ---"
  cat "$watch_json"
}

round_o3_loop() {
  [ -f "$O3_BAG_STATE" ] || { echo "FATAL: $O3_BAG_STATE missing -- run o3-flight first"; bump 1; return; }
  local bag_dir; bag_dir=$(cat "$O3_BAG_STATE")
  local dur; dur=$(o3_bag_duration_sec "$bag_dir")
  [ -z "$dur" ] && dur=120
  local play_timeout watch_duration
  play_timeout=$(python3 -c "print(int($dur*1.6)+30)")
  watch_duration=$(python3 -c "print(int($dur*1.6)+60)")
  echo "bag duration=${dur}s -> loop play_timeout=${play_timeout}s watch_duration=${watch_duration}s"
  o3_run_authority_watch true loop "$play_timeout" "$watch_duration"
}

round_o3_loop_control() {
  [ -f "$O3_BAG_STATE" ] || { echo "FATAL: $O3_BAG_STATE missing -- run o3-flight first"; bump 1; return; }
  local bag_dir; bag_dir=$(cat "$O3_BAG_STATE")
  local dur; dur=$(o3_bag_duration_sec "$bag_dir")
  [ -z "$dur" ] && dur=120
  local play_timeout watch_duration
  play_timeout=$(python3 -c "print(int($dur*1.3)+30)")
  watch_duration=$(python3 -c "print(int($dur*1.3)+60)")
  echo "bag duration=${dur}s -> no-loop play_timeout=${play_timeout}s watch_duration=${watch_duration}s"
  o3_run_authority_watch false loop_control "$play_timeout" "$watch_duration"
}

# ------------------------------------------------------------- o3-synth (Task D')
# P10.8c: deterministic replacement for the --loop-based "N-c mechanism
# bites a real regression" claim (see the Task D' block comment above this
# section for the full isolation writeup). Drives a standalone REAL
# control_authority_manager_node the same way Task D does, but via
# o3_clock_regression_synth.py's own /clock + cmd_mission publishers
# (deliberately silences MISSION across the jump) instead of `ros2 bag
# play --loop` on a bag whose only channel never goes silent.
round_o3_synth() {
  echo "############################################################"
  echo "=== G-O3 Task D' (P10.8c): deterministic N-c regression proof (o3_clock_regression_synth.py) ==="
  echo "############################################################"
  unset ROS_DOMAIN_ID
  pkill -f 'control_authority_manager_node' 2>/dev/null
  pkill -f 'o3_clock_regression_synth.py' 2>/dev/null
  sleep 1
  export ROS_DOMAIN_ID=99

  local ca_log="$LOGDIR/o3_synth_ca.log"
  timeout 60 ros2 run uav_control_authority control_authority_manager_node --ros-args \
    --params-file "$O3_CA_PARAMS" -p uav_id:="$O3_UAV_ID" -p use_sim_time:=true \
    > "$ca_log" 2>&1 < /dev/null &
  local ca_pid=$!

  local deadline=$((SECONDS + 30))
  until timeout 15 ros2 node list --no-daemon 2>/dev/null | grep -q control_authority_manager_node; do
    if [ $SECONDS -gt $deadline ]; then
      echo "FATAL: control_authority_manager_node never came up"; tail -30 "$ca_log"
      kill -9 "$ca_pid" 2>/dev/null; bump 1; unset ROS_DOMAIN_ID; return
    fi
    sleep 2
  done

  local synth_log="$LOGDIR/o3_synth_result.log"
  python3 "$O3_SYNTH_PROBE" --uav-id "$O3_UAV_ID" 2>&1 | tee "$synth_log"
  local synth_rc=${PIPESTATUS[0]}

  o3_stop_pid "$ca_pid" "control_authority_manager_node (o3-synth)"
  unset ROS_DOMAIN_ID
  echo "$synth_rc" > "$O3_SYNTH_EXIT_STATE"
  echo "G-O3 Task D' (deterministic N-c proof) verdict: $synth_rc"
  bump "$synth_rc"
}

round_o3_loop_report() {
  echo "############################################################"
  echo "=== G-O3 Task D report/verdict ==="
  echo "############################################################"
  local loop_json="$LOGDIR/o3_watch_loop.json"
  local control_json="$LOGDIR/o3_watch_loop_control.json"
  for f in "$loop_json" "$control_json" "$O3_SYNTH_EXIT_STATE"; do
    [ -f "$f" ] || {
      echo "FATAL: $f missing -- run o3-synth, o3-loop and o3-loop-control first"; bump 1; return
    }
  done
  local synth_exit; synth_exit=$(cat "$O3_SYNTH_EXIT_STATE")
  python3 -c "
import json, sys
loop = json.load(open('$loop_json'))
ctrl = json.load(open('$control_json'))
synth_exit = $synth_exit
verdict = 0
measurement_ok = True

def check(name, ok, detail):
    global verdict
    status = 'PASS' if ok else 'FAIL'
    if not ok:
        verdict = max(verdict, 1)
    print('[%s] %-55s %s' % (status, name, detail))

# 1. Deterministic proof the N-c mechanism (ageSecOrInf/clock_regressions_)
#    genuinely catches a real regression, run against the REAL deployed
#    node -- unchanged by the P10.9a rerun below, kept as the primary
#    evidence for the MECHANISM itself (contract Sec2.20).
check('1. N-c mechanism catches a real regression (o3_clock_regression_synth.py)',
      synth_exit == 0,
      'exit=%d (0 == clock_regressions increased when a channel was silenced across a real jump)'
      % synth_exit)

# 2/3. P10.9a rerun, positive assertions (R27-3): the OLD --loop run replayed
# the WHOLE bag, including command_selected/authority -- the arbiter's own
# SINGLE-WRITER output -- creating a second publisher on an exclusive-writer
# topic (contract Sec2.20 C3 root-cause row: '2 publishers on .../
# command_selected' logged ~60x) AND hitting the (now-fixed) offered_qos_
# profiles bug, so the arbiter received ZERO live cmd_mission the whole
# session. --topics (O3_ARBITER_INPUT_TOPICS above) fixes the double-writer;
# these two checks confirm the fix actually lands traffic on the arbiter
# BEFORE any clock_regressions number below is trusted (R27-1: a measurement
# that never touched what it claims to measure is not evidence either way).
ctrl_mission = ctrl.get('ever_saw_mission')
loop_mission = loop.get('ever_saw_mission')
check('2. no --loop: arbiter actually reached MISSION (positive assertion)',
      ctrl_mission is True,
      'ever_saw_mission=%s first_mission_at_t=%s' % (ctrl_mission, ctrl.get('first_mission_at_t')))
check('3. --loop: arbiter also reached MISSION during THIS run (positive assertion)',
      loop_mission is True,
      'ever_saw_mission=%s first_mission_at_t=%s' % (loop_mission, loop.get('first_mission_at_t')))
if ctrl_mission is not True or loop_mission is not True:
    measurement_ok = False

# 4/5 RETIRED from the verdict -- G-O3(c) CHOT 2026-08-24 (contract Sec2.20,
# last row): /clock as synthesised by 'ros2 bag play --clock' is a DISCRETE
# tick at ~40.03 Hz (~25ms period), not phase-locked to cmd_mission's own
# ~20Hz (50ms) publish schedule. Measured directly: 30 consecutive samples
# of (now, header.stamp) via a live use_sim_time subscriber -- 29/30 (96.7%)
# came out NEGATIVE (now < stamp), magnitude -0.000..-0.023s, matching the
# ~25ms tick period almost exactly. That means now_sec < stored_stamp
# fires on ALMOST EVERY tick in ANY 'ros2 bag play --clock' session with
# cmd_mission flowing, REGARDLESS of --loop -- it is REPLAY-TOOL
# RESOLUTION NOISE, not the loop wrap this round originally set out to
# detect. Consequence for the two old checks below: #4 (loop_cr > 0) PASSES
# every time, but for the wrong reason (tick noise, not the wrap); #5 (no-
# loop ctrl_cr == 0) FAILS every time, permanently, for the same reason --
# neither number is evidence about the N-c mechanism one way or the other
# through this path. They are printed as INFO ONLY below and do NOT affect
# verdict. Check 1 (o3_clock_regression_synth.py, does not go through bag
# play) remains the ONLY official evidence for the N-c mechanism itself --
# see docs/interface-contract-v0.1.md Sec2.20's G-O3(c) CHOT row.
loop_cr = loop.get('final_clock_regressions')
ctrl_cr = ctrl.get('final_clock_regressions')
print('[INFO] --loop final_clock_regressions=%s -- NOT evidence either way (replay-clock tick '
      'noise, see contract Sec2.20 G-O3(c))' % loop_cr)
print('[INFO] no --loop final_clock_regressions=%s -- NOT evidence either way (same tick noise, '
      'contract Sec2.20 G-O3(c))' % ctrl_cr)

if not measurement_ok:
    verdict = max(verdict, 2)
    print('measurement_ok=False -- checks 2/3 above (positive assertion that traffic reached '
          'the arbiter) failed; the two [INFO] clock_regressions lines above are even less '
          'meaningful without live traffic, on top of already being retired from the verdict')
print('G-O3 Task D verdict: %d' % verdict)
sys.exit(verdict)
"
  bump "$?"
}

# ================================================================ P10.9b -- D0
# Preflight baseline capture (plan panel .claude/plan/P10-gonogo-design-
# panel.md S:4, "thứ tự thi công" D0): parked, healthy stack, 2 configs
# (perception:=false/:=true), D0_DURATION_SEC each, Sub-B ONLY -- the
# waiver mechanism (S:4.a's 7-rule table) is Sub-B-scoped; Sub-A liveness
# always gates and is never waivable, nothing there for a human to review.
D0_UAV_ID=uav0
D0_WORLD=uav_arena
D0_MODEL_NOPERC=uav0_nav
D0_MODEL_PERC=uav0_full
D0_DURATION_SEC=60
D0_CAPTURE="$WORKSPACE_WIN/src/uav_observability/test/preflight_baseline_capture.py"
D0_REPORT="$WORKSPACE_WIN/src/uav_observability/test/preflight_baseline_report.py"
D0_PARAMS_YAML="$WORKSPACE_WIN/src/uav_observability/config/observability_params.yaml"

round_d0_baseline() {
  echo "############################################################"
  echo "=== P10.9b D0: preflight baseline capture (parked, 2 configs x ${D0_DURATION_SEC}s) ==="
  echo "############################################################"
  unset ROS_DOMAIN_ID
  for f in "$D0_CAPTURE" "$D0_REPORT" "$D0_PARAMS_YAML"; do
    [ -f "$f" ] || { echo "FATAL: $f missing"; bump 1; return; }
  done

  local jsonl_false="$LOGDIR/p10_9b_baseline_perception_false.jsonl"
  local jsonl_true="$LOGDIR/p10_9b_baseline_perception_true.jsonl"
  : > "$jsonl_false"
  : > "$jsonl_true"

  echo "--- config 1/2: perception:=false ---"
  o2_teardown || { bump 1; return; }
  o2_start_stack "$D0_MODEL_NOPERC" "$D0_WORLD" false true true false d0_false_bringup.log
  local rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "FATAL: stack never came up (perception:=false)"; bump 1; o2_teardown; return
  fi
  python3 "$D0_CAPTURE" --uav-id "$D0_UAV_ID" --duration "$D0_DURATION_SEC" \
    --params-yaml "$D0_PARAMS_YAML" --output "$jsonl_false" \
    2>&1 | tee "$LOGDIR/p10_9b_baseline_perception_false.log"
  o2_teardown

  echo "--- config 2/2: perception:=true ---"
  o2_teardown || { bump 1; return; }
  o2_start_stack "$D0_MODEL_PERC" "$D0_WORLD" true true true false d0_true_bringup.log
  rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "FATAL: stack never came up (perception:=true)"; bump 1; o2_teardown; return
  fi
  o2_wait_nodes camera_health_node || { bump 1; o2_teardown; return; }
  python3 "$D0_CAPTURE" --uav-id "$D0_UAV_ID" --duration "$D0_DURATION_SEC" \
    --params-yaml "$D0_PARAMS_YAML" --output "$jsonl_true" \
    2>&1 | tee "$LOGDIR/p10_9b_baseline_perception_true.log"
  o2_teardown

  echo
  echo "=== candidate waiver table (both configs combined) -- for project owner review ==="
  python3 "$D0_REPORT" --jsonl "$jsonl_false" "$jsonl_true" \
    2>&1 | tee "$LOGDIR/p10_9b_baseline_candidates.txt"
}

# ============================================================ P10.9a -- G-O4
# go/no-go on the wire, go_no_go semantics redesigned P10.9b (design panel
# .claude/plan/P10-gonogo-design-panel.md). Reuses o2_start_stack/
# o2_wait_nodes/o2_teardown (P10.7) -- a real Gazebo+PX4+ROS bringup, domain
# 0. Every sub-part below stays disarmed throughout, so gate_mode settles to
# "preflight" BY MEASUREMENT (armed=false off /state/vehicle, P10.9b -- no
# more yaml/launch-arg default to point at) -- flight_only items are
# therefore NEVER counted here on purpose: go_no_go is a PRE-FLIGHT gate,
# this round never has to fly. event_log is left off in every scenario
# on purpose: event_logger_node's diagnostics/observability_events source has
# owner="blackbox", which ownerIsCounted() NEVER counts (O1) -- it cannot
# change any verdict below either way, so leaving it off only sheds load.

O4_UAV_ID=uav0
O4_WORLD=uav_arena
O4_MODEL_NOPERC=uav0_nav      # matches G-M1/M5 combo, no camera render load
O4_MODEL_PERC=uav0_full       # BOTH cameras -- (c) needs a real perception owner
O4_SAMPLER="$WORKSPACE_WIN/src/uav_observability/test/o4_system_health_sampler.py"
O4_REPORT="$WORKSPACE_WIN/src/uav_observability/test/o4_report.py"
O4_WAIVER_GATE="$WORKSPACE_WIN/src/uav_observability/test/o4_waiver_gate.py"
O4_SETTLE_SEC=25          # let discovery + every "always" item's cadence settle
O4_KILL_WAIT_SEC=8        # >> the 2s deadline under test, generous margin
# N7 (P10-gate-debt review round 2): feeds o4_report.py's min-samples/
# freshness precheck (see that file). V2 (P10 review lượt 3): this is now
# only the FALLBACK -- round_o4_gate() derives the real value straight off
# config/observability_params.yaml's publish_period_sec and overrides it,
# WARNing (never silently) if the two ever disagree (R34 spirit -- a
# hand-typed copy going stale fails OPEN: too small widens the fresh window
# up to 4x, too large falsely reports STALE right after ARM).
O4_PUBLISH_PERIOD_SEC=1.0
# localization_mux_node's own 2 owned "always" items (state/estimator_source
# is NOT in always_names -- diagnostics_node never watches it) PLUS the one
# Sub-B content child localization_health_node can raise off the SAME kill
# (design panel warned this exact race, .claude/plan/P10-gonogo-design-
# panel.md:225 -- confirmed live 2026-08-24, worst_item landed on
# "diagnostics/localization:localization: fused output" instead of a Sub-A
# name). Sub-A (cadence liveness on state/odometry_fused) and Sub-B (content
# check in localization_health_node.cpp:172, same topic) are TWO VALID
# detectors of the ONE injected fault -- which one's tick lands first is a
# scheduling race, not a correctness question, so both must be accepted.
# This is NOT a general widening: still exactly 3 names, all pinned to the
# localization domain this kill actually touches (R27-3 negative injection
# below proves an unrelated domain, e.g. diagnostics/safety:*, still FAILs).
O4_ALLOWED_WORST="state/odometry_fused,state/localization_status,diagnostics/localization:localization: fused output"
# P10.9b (f): the ONLY "<source>:<child>" pairs a waiver may legitimately
# exempt right now -- MUST match config/preflight_waivers.yaml's 5 signed
# rows exactly (see that file's own header + docs/interface-contract-
# v0.1.md S:2.20). Adding a row to the yaml WITHOUT adding it here too is a
# FAIL on this gate on purpose (G6).
O4_ALLOWED_WAIVED="diagnostics/safety:safety: OFFBOARD_UNHEALTHY,diagnostics/safety:safety: BLIND_COMMAND,diagnostics/safety:safety: planning/trajectory plan_state,diagnostics/safety:safety: CAMERA_STREAM_UNHEALTHY,diagnostics/safety:safety: OBSTACLE_TOO_CLOSE"

for f in "$O4_SAMPLER" "$O4_REPORT" "$O4_WAIVER_GATE"; do
  [ -f "$f" ] || { echo "FATAL: $f missing"; exit 1; }
done

# o4_go_check LABEL JSONL [WANT] [WORST_CONTAINS] -- fresh sampler for
# O4_SETTLE_SEC, then verdict. WANT defaults to GO (o4_report.py's own
# default) -- (c) passes NO_GO + a worst_item substring (D4, S:6).
o4_go_check() {
  local label="$1" jsonl="$2" want="${3:-GO}" worst_contains="${4:-}"
  : > "$jsonl"
  python3 "$O4_SAMPLER" --uav-id "$O4_UAV_ID" --duration "$O4_SETTLE_SEC" --output "$jsonl" \
    > "$LOGDIR/o4_sampler_${label}.log" 2>&1
  local extra_args=(--want "$want")
  [ -n "$worst_contains" ] && extra_args+=(--worst-contains "$worst_contains")
  python3 "$O4_REPORT" go --jsonl "$jsonl" --label "$label" \
    --duration-sec "$O4_SETTLE_SEC" --publish-period-sec "$O4_PUBLISH_PERIOD_SEC" \
    "${extra_args[@]}" \
    2>&1 | tee "$LOGDIR/o4_verdict_${label}.log"
  return "${PIPESTATUS[0]}"
}

round_o4_gate() {
  echo "############################################################"
  echo "=== G-O4: go/no-go on the wire (plan P10.4, run for the first time) ==="
  echo "############################################################"
  unset ROS_DOMAIN_ID

  # V2 (P10 review lượt 3): derive O4_PUBLISH_PERIOD_SEC off the SAME yaml
  # diagnostics_node actually loads (R34 spirit) instead of trusting the
  # hand-typed fallback declared above -- cross-check and WARN (never
  # silently override) if the two disagree.
  local yaml_publish_period
  yaml_publish_period=$(python3 -c "
import sys
try:
    import yaml
except ImportError:
    sys.exit(1)
try:
    with open('$D0_PARAMS_YAML') as f:
        doc = yaml.safe_load(f)
    print(doc['diagnostics_node']['ros__parameters']['publish_period_sec'])
except Exception:
    sys.exit(1)
" 2>/dev/null)
  if [ -n "$yaml_publish_period" ]; then
    if [ "$yaml_publish_period" != "$O4_PUBLISH_PERIOD_SEC" ]; then
      echo "WARN: hand-typed O4_PUBLISH_PERIOD_SEC ($O4_PUBLISH_PERIOD_SEC) != " \
        "$D0_PARAMS_YAML's publish_period_sec ($yaml_publish_period) -- using the yaml value"
    fi
    O4_PUBLISH_PERIOD_SEC="$yaml_publish_period"
  else
    echo "WARN: could not derive O4_PUBLISH_PERIOD_SEC from $D0_PARAMS_YAML (python3/PyYAML " \
      "missing, or yaml unreadable) -- falling back to hand-typed $O4_PUBLISH_PERIOD_SEC, " \
      "verify it still matches config/observability_params.yaml's publish_period_sec"
  fi

  # --------------------------------------------- (a)+(b) perception:=false
  echo "--- (a)+(b): perception:=false default, then SIGKILL localization_mux_node ---"
  o2_teardown || { bump 1; return; }
  local out rc
  out=$(o2_start_stack "$O4_MODEL_NOPERC" "$O4_WORLD" false true true false o4_ab_bringup.log)
  rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "FATAL: stack never came up for (a)/(b)"; bump 1; o2_teardown; return
  fi
  # diagnostics_node + localization_mux_node are already in o2_start_stack's
  # own wait list (diagnostics:=true / the base "always" node set) -- no
  # extra wait needed here.

  local ab_jsonl="/tmp/o4_ab.jsonl" kill_time_file="/tmp/o4_kill_time.txt"
  : > "$ab_jsonl"
  local sampler_duration=$((O4_SETTLE_SEC + O4_KILL_WAIT_SEC + 5))
  python3 "$O4_SAMPLER" --uav-id "$O4_UAV_ID" --duration "$sampler_duration" --output "$ab_jsonl" \
    > "$LOGDIR/o4_sampler_ab.log" 2>&1 &
  local sampler_pid=$!

  echo "settling ${O4_SETTLE_SEC}s before reading the (a) baseline..."
  sleep "$O4_SETTLE_SEC"

  local mux_pid
  mux_pid=$(pgrep -f 'install/uav_localization/lib/uav_localization/localization_mux_node' | head -1)
  if [ -z "$mux_pid" ]; then
    echo "FAILED TO MEASURE: localization_mux_node PID not found -- cannot run the (b) positive control"
    bump 2; kill -9 "$sampler_pid" 2>/dev/null; wait "$sampler_pid" 2>/dev/null; o2_teardown; return
  fi
  python3 -c 'import time; print(f"{time.time():.6f}")' > "$kill_time_file"
  echo "SIGKILL localization_mux_node (pid=$mux_pid) at t_kill=$(cat "$kill_time_file")"
  kill -9 "$mux_pid"

  sleep "$O4_KILL_WAIT_SEC"
  wait "$sampler_pid" 2>/dev/null

  python3 "$O4_REPORT" ab --jsonl "$ab_jsonl" --kill-time-file "$kill_time_file" \
    --allowed-worst "$O4_ALLOWED_WORST" --deadline-sec 2.0 \
    --settle-sec "$O4_SETTLE_SEC" --publish-period-sec "$O4_PUBLISH_PERIOD_SEC" \
    2>&1 | tee "$LOGDIR/o4_verdict_ab.log"
  bump "${PIPESTATUS[0]}"
  cp "$ab_jsonl" "$LOGDIR/o4_ab.jsonl" 2>/dev/null

  o2_teardown

  # ------------------------------------------------- (c) perception:=true
  echo "--- (c): perception:=true (world/semantic_landmarks now has a real owner) ---"
  o2_teardown || { bump 1; return; }
  out=$(o2_start_stack "$O4_MODEL_PERC" "$O4_WORLD" true true true false o4_c_bringup.log)
  rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "FATAL: stack never came up for (c)"; bump 1; o2_teardown; return
  fi
  # camera_health_node is the one node o2_start_stack's own list omits
  # (same gap o3-camera-health's comment already documents); diagnostics_node
  # + world_model_node are already covered by its perception/diagnostics flags.
  o2_wait_nodes camera_health_node || { bump 1; o2_teardown; return; }
  # P10.9b (D4): perception:=true in THIS sim is EXPECTED NO_GO -- the
  # camera bridge really does drop frames (nợ #10, delivered_fraction
  # 0.40/0.45). Cấm nới: this asserts worst_item actually names
  # diagnostics/perception, not just "some NO_GO reason". When nợ #10 is
  # fixed this round goes red on purpose and the expectation must change.
  o4_go_check "c" "/tmp/o4_c.jsonl" "NO_GO" "diagnostics/perception"
  bump "$?"
  cp "/tmp/o4_c.jsonl" "$LOGDIR/o4_c.jsonl" 2>/dev/null
  o2_teardown

  # --------------------------------------- (d) blackbox:=false diagnostics:=true
  echo "--- (d): blackbox:=false diagnostics:=true (D-4's real-flight combo) ---"
  o2_teardown || { bump 1; return; }
  out=$(o2_start_stack "$O4_MODEL_NOPERC" "$O4_WORLD" false false true false o4_d_bringup.log)
  rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "FATAL: stack never came up for (d)"; bump 1; o2_teardown; return
  fi
  o4_go_check "d" "/tmp/o4_d.jsonl"
  bump "$?"
  cp "/tmp/o4_d.jsonl" "$LOGDIR/o4_d.jsonl" 2>/dev/null
  o2_teardown

  # ---------------------------------------------- (e) stamp_sec / wall_stamp_sec
  echo "--- (e): stamp_sec/wall_stamp_sec present and monotonic (C6), off (a)-(d)'s own JSONL ---"
  python3 "$O4_REPORT" monotonic \
    --jsonl "$LOGDIR/o4_ab.jsonl" "$LOGDIR/o4_c.jsonl" "$LOGDIR/o4_d.jsonl" --min-samples 10 \
    2>&1 | tee "$LOGDIR/o4_verdict_e.log"
  bump "${PIPESTATUS[0]}"

  # ------------------------------- (f) MỚI (P10.9b, G6): waiver path + allow-list
  echo "--- (f): preflight GO with the waiver path ACTUALLY exercised + allow-list (G6) ---"
  o2_teardown || { bump 1; return; }
  out=$(o2_start_stack "$O4_MODEL_NOPERC" "$O4_WORLD" false true true false o4_f_bringup.log)
  rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "FATAL: stack never came up for (f)"; bump 1; o2_teardown; return
  fi
  python3 "$O4_WAIVER_GATE" --uav-id "$O4_UAV_ID" --timeout "$O4_SETTLE_SEC" \
    --allowed-waived "$O4_ALLOWED_WAIVED" \
    2>&1 | tee "$LOGDIR/o4_verdict_f.log"
  bump "${PIPESTATUS[0]}"
  o2_teardown
}

# --------------------------------------------------------------- dispatch
ROUNDS=("$@")
[ ${#ROUNDS[@]} -eq 0 ] && ROUNDS=(o1-control o1-mcap-zstdfile o1-mcap-none o1-sqlite3-none o1-report)
[ "${ROUNDS[0]}" = "o1" ] && ROUNDS=(o1-control o1-mcap-zstdfile o1-mcap-none o1-sqlite3-none o1-report)

# Fresh CSV whenever the run starts from o1-control (i.e. a full/from-
# scratch run) -- re-running a single config round APPENDS instead, so a
# partial re-run doesn't erase the other 2 configs' data.
[ "${ROUNDS[0]}" = "o1-control" ] && : > "$RESULTS_CSV"

[ "${ROUNDS[0]}" = "o2" ] && ROUNDS=(o2-main o2-control o2-hz o2-report)

[ "${ROUNDS[0]}" = "o3" ] &&
  ROUNDS=(o3-camera-health o3-flight o3-replay o3-loop o3-loop-control o3-synth o3-loop-report)

for r in "${ROUNDS[@]}"; do
  case "$r" in
    o1-control) round_o1_control ;;
    o1-mcap-zstdfile) round_o1_config mcap_zstdfile mcap file zstd ;;
    o1-mcap-none) round_o1_config mcap_none mcap none none ;;
    o1-sqlite3-none) round_o1_config sqlite3_none sqlite3 none none ;;
    o1-prod) round_o1_prod ;;
    o1-report) round_o1_report; bump "$?" ;;
    o2-main) round_o2_main ;;
    o2-extra) round_o2_extra ;;
    o2-control) round_o2_control ;;
    o2-hz) round_o2_hz ;;
    o2-report) round_o2_report; bump "$?" ;;
    o2-m5) round_o2_m5 ;;
    o3-camera-health) round_o3_camera_health ;;
    o3-flight) round_o3_flight ;;
    o3-replay) round_o3_replay ;;
    o3-loop) round_o3_loop ;;
    o3-loop-control) round_o3_loop_control ;;
    o3-synth) round_o3_synth ;;
    o3-loop-report) round_o3_loop_report ;;
    d0-baseline) round_d0_baseline ;;
    o4-gate) round_o4_gate ;;
    *) echo "FATAL: unknown round '$r'"; bump 1 ;;
  esac
done

echo
echo "############################################################"
echo "verify_observability.sh OVERALL exit status: $OVERALL   (0=PASS 1=FAIL 2=FAILED TO MEASURE)"
echo "results CSV: $RESULTS_CSV"
exit "$OVERALL"
