#!/bin/bash
# GATE R0 -- may the large-samples DDS profile be wired into the sim bringup?
#
# WHY THIS GATE EXISTS. config/fastdds_large_samples.xml fixes debt #10 by raising
# the Fast DDS shared-memory segment from the stock 549 408 B to 16 MiB, which is
# what lets a 640x480 camera frame travel in one piece. Measured 2026-08-24: depth
# 4.8 -> 15.2 Hz, zero loss, RTF 0.889 -> 0.963, no sensor degraded.
#
# But the profile is exported BEFORE start_sim.sh, so the PX4 uXRCE-DDS agent is a
# child of that shell and inherits it -- and useBuiltinTransports=false then
# REPLACES the agent's transports too. That is the exact position where
# config/fastdds_shm_only.xml died: it severed every /fmu/out/* topic, because the
# agent links Fast DDS 2.12.2 while ROS Humble links 2.6.11 and their SHM segment
# layouts do not interoperate. /fmu/* is the flight-controller link. Losing it in
# flight is not a degraded picture, it is a lost aircraft. Hence R0.
#
# The trial script only waited for /fmu/out/vehicle_odometry to APPEAR. Appearing is
# not health: a link that discovers fine and then delivers 12 of 100 samples looks
# identical to a healthy one at that level of proof.
#
# WHAT IS MEASURED, AND THE TWO TRAPS IT AVOIDS.
#
#  (1) R27-1, gate the INSTRUMENT before the object. A Fast DDS profile that fails
#      to parse is SILENT -- both arms would then run stock transport and the gate
#      would "pass" for the wrong reason. So each arm reads back the largest
#      /dev/shm/fastrtps_* segment actually allocated and refuses to conclude unless
#      the control shows the stock size and the profile arm shows 16 MiB.
#
#  (2) RATE IN WALL TIME IS NOT A FAIR COMPARISON. PX4 publishes on SIM time, so a
#      run at RTF 0.963 delivers about 8% more wall-clock samples than one at 0.889
#      with an identical transport. Comparing raw Hz would credit the profile for
#      the RTF it also happens to improve. Every rate is therefore normalised by the
#      RTF measured in the SAME window and the verdict is taken on the normalised
#      number. The raw number is still printed, because the worst-case GAP is a
#      wall-clock quantity and must be judged as one.
#
# Both arms run the full autonomy stack (perception:=true). That is the production
# condition and the harsher one; a transport verdict taken on an idle rig would be
# R31 exactly -- a number valid only in the condition it was forced in.
#
# WHY EVERY PATH IS INSIDE THIS FILE. stop_sim.sh derives its kill patterns from the
# built packages (R34) and runs pkill -f "<pkg>/". A command line carrying a path
# like uav_bringup/config/... matches, and the caller kills ITSELF (measured: SIGHUP,
# exit 15). Keeping the paths in the file keeps them off the command line.
#
# Usage: bash scripts/gate_r0_dds_profile.sh
# Env:   WINDOW (default 60) seconds of measurement per arm
#        ARMS   (default "control profile")
set -o pipefail

CANON=/mnt/c/code/PX4_ROS2
WORKSPACE=$HOME/PX4_ROS2
PROFILE=$CANON/src/uav_bringup/config/fastdds_large_samples.xml
WINDOW=${WINDOW:-60}
ARMS=${ARMS:-"control profile"}
export UAV_MODEL=uav0_full
export UAV_WORLD=${UAV_WORLD:-uav_arena}

STOCK_SEGMENT=549408
ASKED_SEGMENT=16777216

# Baseline of record, M2 smoke test 2026-08-03: vehicle_odometry ~100 Hz,
# vehicle_status 1.98 Hz. sensor_combined measured 106 Hz on the AirSim rig (M-A0).
TOPICS="/fmu/out/vehicle_odometry /fmu/out/vehicle_status /fmu/out/sensor_combined"

# Evidence lives under the workspace, not /tmp: the first run's whole evidence tree
# (both arms' hz logs and the two .txt records) disappeared from /tmp between the
# verdict printing and the follow-up read, with systemd-tmpfiles last active hours
# earlier and stop_sim.sh touching nothing under /tmp. Cause not established -- which
# is exactly why the evidence must not live somewhere that can do that.
OUT=$WORKSPACE/gate_logs/r0
mkdir -p "$OUT"
rm -f "$OUT"/*.txt "$OUT"/*.log 2>/dev/null

STACK_PID=""
unmeasured=0

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

[ -f "$PROFILE" ] || { echo "FATAL: profile missing at the canonical path"; exit 2; }

# A parallel build swaps install/ mid-experiment; that produced ten void rounds at
# G-O1 before the rule was written down (R15, brief item 2).
if pgrep -x colcon >/dev/null || pgrep -x ctest >/dev/null; then
  echo "FATAL: a build/test run holds install/"; exit 2
fi

stop_stack() {
  [ -n "$STACK_PID" ] || return 0
  kill "$STACK_PID" 2>/dev/null
  wait "$STACK_PID" 2>/dev/null
  STACK_PID=""
}
trap 'stop_stack; bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1' EXIT

# Stale port/segment files cannot be reused across different transport parameters
# (observed: "Failed init_port fastrtps_port7455: open_and_lock_file"). Clean them
# only here, with nothing alive -- those files belong to live participants.
clean_shm() {
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
  ros2 daemon stop >/dev/null 2>&1
  sleep 3
  if ps -eo comm | grep -qE '^gz$|^px4$'; then
    echo "  FATAL: something is still alive; refusing to touch /dev/shm"
    return 1
  fi
  local n
  n=$(find /dev/shm -maxdepth 1 -name 'fastrtps_*' -print -delete 2>/dev/null | wc -l)
  echo "  cleared $n stale Fast DDS shm files"
}

largest_segment() {
  find /dev/shm -maxdepth 1 -name 'fastrtps_*' ! -name '*_el' -printf '%s\n' 2>/dev/null \
    | sort -n | tail -1
}

# Confirms the autonomy stack is genuinely imposing load, and -- the part the first
# version got wrong -- keeps "the CLI did not answer" separate from "the nodes are
# not there". Round 1 of this gate collapsed those two into "stack did not come up"
# and voided the profile arm, while stack_profile.log showed rosbag_manager_node
# RECORDING 46/46 topics and world_model_node accepting 303 obstacles / 602 targets
# in that very window. scripts/diagnose_ros2cli_under_profile.sh then refuted both
# candidate explanations outright: under this profile `ros2 node list --no-daemon`
# returns 5/5 perception nodes and 29 nodes total at every spin time from 1 s to
# 30 s with empty stderr. So the single reading was a transient, and mapping it onto
# a definite verdict is the same not-measurable-becomes-a-value fault that produced
# 9 of the 16 review findings in P10 (R30).
start_stack() {
  local arm=$1
  ros2 launch uav_bringup sim.launch.py perception:=true > "$OUT/stack_$arm.log" 2>&1 &
  STACK_PID=$!
  sleep 35
  local nodes="" attempt
  for attempt in 1 2 3; do
    nodes=$(timeout 60 ros2 node list --no-daemon --spin-time 5 \
            2>"$OUT/nodelist_${arm}_${attempt}.err" \
            | grep -cE 'camera_health_node|marker_detector_node|object_detector_node|target_tracker_node|obstacle_extractor_node')
    [ "${nodes:-0}" -ge 4 ] && break
    echo "  attempt $attempt saw ${nodes:-0}/5; stderr: $(tr '\n' ' ' < "$OUT/nodelist_${arm}_${attempt}.err" | cut -c1-160)"
    sleep 10
  done
  echo "  perception nodes up: ${nodes:-0}/5"

  # Independent, CLI-free witness that images are really flowing: world_model_node
  # only counts an observation it actually accepted.
  local accepted
  accepted=$(grep -oE 'accepted markers=[0-9]+ obstacles=[0-9]+ targets=[0-9]+' "$OUT/stack_$arm.log" | tail -1)
  echo "  world_model throughput: ${accepted:-none reported yet}"

  if [ "${nodes:-0}" -lt 4 ]; then
    if [ -n "$accepted" ]; then
      echo "  FAILED TO MEASURE: the CLI could not enumerate the nodes, but the launch"
      echo "                     log proves the stack is alive and processing. This is a"
      echo "                     measurement failure, NOT a stack failure -- do not read"
      echo "                     it as evidence against the profile."
    else
      echo "  FAILED TO MEASURE: stack did not come up (no throughput in the launch log"
      echo "                     either, so this one really may be the stack)."
    fi
    return 1
  fi
  echo "  stack confirmed live"
}

# Parses the LAST reporting block of ros2 topic hz, which carries the widest
# averaging window. Prints "rate maxperiod" or "none".
parse_hz() {
  local f=$1 rate mx
  rate=$(grep -oE 'average rate: [0-9.]+' "$f" | tail -1 | grep -oE '[0-9.]+$')
  mx=$(grep -oE 'max: [0-9.]+s' "$f" | tail -1 | grep -oE '[0-9.]+')
  if [ -z "$rate" ]; then echo "none"; else echo "$rate ${mx:-nan}"; fi
}

run_arm() {
  local arm=$1
  echo
  echo "############ ARM: $arm ############"

  if [ "$arm" = "profile" ]; then
    unset UAV_DDS_PROFILE
    export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE"
    echo "  profile in force"
  else
    # Since 2026-08-24 the profile is wired into start_sim.sh AND sim.launch.py, so
    # merely unsetting the Fast DDS variable no longer yields a stock arm -- both
    # would put it back and the control would silently become a second profile arm.
    # UAV_DDS_PROFILE=none is the opt-out both honour. The instrument gate below
    # still verifies the result rather than trusting this.
    export UAV_DDS_PROFILE=none
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    echo "  no profile (stock transport) -- this arm is the positive control:"
    echo "  it proves the instrument can see the link at full rate on this machine."
  fi

  clean_shm || { unmeasured=$((unmeasured+1)); return; }

  bash "$WORKSPACE/scripts/start_sim.sh" > "$OUT/start_$arm.log" 2>&1
  local deadline=$((SECONDS + 420))
  until timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; do
    if [ $SECONDS -gt $deadline ]; then
      echo "  FAILED TO MEASURE: PX4 never published in arm $arm"
      echo "  (this is itself the R0 failure if it happens only in the profile arm)"
      echo "topics_absent" > "$OUT/$arm.txt"
      unmeasured=$((unmeasured+1)); return
    fi
    sleep 10
  done
  echo "  /fmu/out/vehicle_odometry discovered"
  sleep 10

  start_stack "$arm" || { unmeasured=$((unmeasured+1)); stop_stack; return; }

  # R27-1 instrument gate: read what the transport ACTUALLY is, never trust the env.
  local seg
  seg=$(largest_segment)
  echo "  largest shm segment: ${seg:-none} B (stock $STOCK_SEGMENT, profile asks $ASKED_SEGMENT)"
  if [ "$arm" = "profile" ] && [ "${seg:-0}" -lt "$ASKED_SEGMENT" ]; then
    echo "  FAILED TO MEASURE: the XML did not take effect; a profile that fails to"
    echo "                     parse is silent, so this arm is stock wearing a label."
    unmeasured=$((unmeasured+1)); stop_stack; return
  fi
  if [ "$arm" = "control" ] && [ "${seg:-0}" -ge "$ASKED_SEGMENT" ]; then
    echo "  FAILED TO MEASURE: the control arm is running the profile -- the two arms"
    echo "                     would be the same experiment run twice."
    unmeasured=$((unmeasured+1)); stop_stack; return
  fi

  # The daemon may predate this arm's environment; drop it so discovery is fresh.
  ros2 daemon stop >/dev/null 2>&1
  sleep 2

  echo "  measuring ${WINDOW}s ..."
  local pids=""
  for t in $TOPICS; do
    local name=${t##*/}
    ( timeout "$WINDOW" ros2 topic hz "$t" --wall-time > "$OUT/hz_${arm}_${name}.log" 2>&1 ) &
    pids="$pids $!"
  done
  bash "$WORKSPACE/scripts/sample_rtf.sh" "$UAV_WORLD" 120 > "$OUT/rtf_$arm.log" 2>&1 &
  pids="$pids $!"
  for p in $pids; do wait "$p" 2>/dev/null; done

  local rtf
  rtf=$(grep -oE 'mean [0-9.]+' "$OUT/rtf_$arm.log" | grep -oE '[0-9.]+$')
  echo "  RTF: $(cat "$OUT/rtf_$arm.log")"
  : > "$OUT/$arm.txt"
  echo "rtf ${rtf:-nan}" >> "$OUT/$arm.txt"
  for t in $TOPICS; do
    local name=${t##*/} res
    res=$(parse_hz "$OUT/hz_${arm}_${name}.log")
    if [ "$res" = "none" ]; then
      echo "  $name: NO MESSAGES"
      echo "$name none none none" >> "$OUT/$arm.txt"
    else
      local r m norm
      r=$(echo "$res" | awk '{print $1}')
      m=$(echo "$res" | awk '{print $2}')
      norm=$(awk -v r="$r" -v f="${rtf:-0}" 'BEGIN{if(f+0>0) printf "%.2f", r/f; else print "nan"}')
      printf '  %-20s %8s Hz wall | %8s Hz sim-normalised | worst gap %ss\n' "$name" "$r" "$norm" "$m"
      echo "$name $r $m $norm" >> "$OUT/$arm.txt"
    fi
  done
  stop_stack
  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
}

for a in $ARMS; do run_arm "$a"; done

echo
echo "############ VERDICT ############"
if [ "$unmeasured" -ne 0 ]; then
  echo "  RESULT: FAILED TO MEASURE in $unmeasured arm(s) -- NO CONCLUSION DRAWN."
  echo "  A gate that cannot measure must not report a pass (R30)."
  exit 2
fi

# The verdict is a separate file so a change of CRITERION shows up as a diff instead
# of an edit buried in this script, and so it can be re-run on saved evidence without
# re-flying both arms. It also documents why the absolute floors are declared on the
# sim-normalised rate rather than the wall rate -- the control arm read vehicle_status
# at 1.476 Hz wall and would have failed a wall floor taken from the M2 bare-rig
# baseline, which means that floor was measuring RTF, not the transport.
python3 "$CANON/scripts/gate_r0_verdict.py" "$OUT/control.txt" "$OUT/profile.txt"
rc=$?
echo "  (evidence: $OUT)"
exit $rc
