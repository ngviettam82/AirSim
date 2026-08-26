#!/bin/bash
# Debt #10: identify the UNIT of the bridge ceiling -- messages/s or MiB/s?
# Every configuration measured before this varied WHICH streams competed, never
# how many bytes a message carries. With every image ~1 MiB those two numbers
# are identical, so no earlier run could tell them apart. Resolution is the
# discriminator: quarter the pixels and see which aggregate stays put.
#
# 🔴 R27-1 (learned the hard way on 2026-08-24, first run of this script): editing
# the SDF under src/ changes NOTHING at run time -- Gazebo loads the model from
# install/ via the ament resource hook. The first run therefore reported all three
# configurations delivering 0.879 MiB/msg, i.e. it measured the SAME setup three
# times, and the invariance would have been read as "message-rate ceiling". So this
# script now (a) edits the INSTALLED sdf and (b) refuses to report an aggregate
# unless the per-message byte count actually MOVED as configured.
#
# 🔴 PERCEPTION=1 (added 2026-08-24) is what makes the numbers DECISION-GRADE.
# Without the autonomy stack the bridge delivers ~50 msg/s; with it, P5.2 measured
# ~29.7 (front 5.5 / depth 7.0 / down 17.2). The shortfall is CPU contention, not
# bytes. So a reallocation that looks affordable on the idle rig can be impossible
# under load -- and depth is the stream that feeds obstacle avoidance, which
# obstacle_extractor_node was designed against at ~7 Hz with gaps to 1.8 s
# (package-status S4). Measuring without the stack and deciding from it would be
# R31 exactly: a number valid only in the condition it was forced in.
#
# Usage: bash scripts/measure_image_budget.sh
# Env:   SECONDS_WINDOW (default 30.0)  UAV_WORLD (default uav_arena)
#        PERCEPTION=1   run the autonomy stack (perception:=true) during the window
#        CONFIGS="A B"  which configurations to run (default "A B C")
# Docs:  docs/package-status.md S4 (debt #10)
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
CANONICAL=/mnt/c/code/PX4_ROS2/src/uav_sim_gz/models/sensor_camera_down/model.sdf
DOWN_SDF=$WORKSPACE/install/uav_sim_gz/share/uav_sim_gz/models/sensor_camera_down/model.sdf
DEPTH_CANON=/mnt/c/code/PX4_ROS2/src/uav_sim_gz/models/sensor_depth_front/model.sdf
DEPTH_SDF=$WORKSPACE/install/uav_sim_gz/share/uav_sim_gz/models/sensor_depth_front/model.sdf
export UAV_MODEL=uav0_full
export UAV_WORLD=${UAV_WORLD:-uav_arena}
SECONDS_WINDOW=${SECONDS_WINDOW:-30.0}
# `-p seconds:=60` types the value from its LITERAL form, so an integer for a
# double parameter aborts the node at declare time -- the probe then prints
# nothing at all and the run reads as "no frames delivered" (2026-08-24).
case "$SECONDS_WINDOW" in *.*) ;; *) SECONDS_WINDOW="${SECONDS_WINDOW}.0" ;; esac
PERCEPTION=${PERCEPTION:-0}
CONFIGS=${CONFIGS:-"A B C"}
STACK_PID=""

ps -eo comm | grep -qE '^ctest$|^colcon$' && { echo "FATAL: a build/test run holds install/"; exit 2; }

# DDS_PROFILE=<path> exports FASTRTPS_DEFAULT_PROFILES_FILE for the simulator, the
# bridge and the stack alike (they are all children of this shell). Used to test the
# transport hypothesis without touching a single sensor.
if [ -n "$DDS_PROFILE" ]; then
  [ -f "$DDS_PROFILE" ] || { echo "FATAL: DDS_PROFILE=$DDS_PROFILE not found"; exit 2; }
  export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE"
  echo "DDS profile in force: $DDS_PROFILE"
fi

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

[ -f "$DOWN_SDF" ] || { echo "FATAL: $DOWN_SDF missing (build uav_sim_gz)"; exit 2; }
[ -f "$CANONICAL" ] || { echo "FATAL: $CANONICAL missing"; exit 2; }

restore() {
  cp "$CANONICAL" "$DOWN_SDF"
  [ -f "$DEPTH_CANON" ] && cp "$DEPTH_CANON" "$DEPTH_SDF"
  echo "installed camera SDFs restored from the canonical tree"
}
trap 'stop_stack; restore' EXIT

unmeasured=0

# Expected payload for an RGB8 frame, in MiB, straight from the SDF being used.
expected_mib_per_msg() {
  local w h
  w=$(grep -oE '<width>[0-9]+</width>' "$DOWN_SDF" | head -1 | grep -oE '[0-9]+')
  h=$(grep -oE '<height>[0-9]+</height>' "$DOWN_SDF" | head -1 | grep -oE '[0-9]+')
  awk -v w="$w" -v h="$h" 'BEGIN {printf "%.3f", w * h * 3 / 1048576}'
}

# Depth is R_FLOAT32: 4 bytes per pixel, not 3. Same guard, different arithmetic --
# without it, configuration D could change the SDF and measure the old setup, which
# is exactly the empty experiment this script was rewritten to make impossible.
expected_depth_mib_per_msg() {
  local w h
  w=$(grep -oE '<width>[0-9]+</width>' "$DEPTH_SDF" | head -1 | grep -oE '[0-9]+')
  h=$(grep -oE '<height>[0-9]+</height>' "$DEPTH_SDF" | head -1 | grep -oE '[0-9]+')
  awk -v w="$w" -v h="$h" 'BEGIN {printf "%.3f", w * h * 4 / 1048576}'
}

start_stack() {
  [ "$PERCEPTION" = "1" ] || return 0
  ros2 launch uav_bringup sim.launch.py perception:=true     > /tmp/budget_stack.log 2>&1 &
  STACK_PID=$!
  sleep 35
  # R27-1: "launched" is not "running". If the perception nodes are not actually
  # up and publishing, the window measures the idle rig again -- which is the
  # very condition this flag exists to leave behind.
  local nodes
  # --no-daemon is mandatory: a daemon started under a different DDS profile cannot
  # see the new participants and reports 0 nodes while the stack is perfectly alive
  # (measured 2026-08-24 -- the launch log showed those nodes running).
  nodes=$(timeout 25 ros2 node list --no-daemon 2>/dev/null     | grep -cE 'camera_health_node|marker_detector_node|object_detector_node|target_tracker_node|obstacle_extractor_node')
  echo "  perception nodes up: ${nodes:-0}/5"
  if [ "${nodes:-0}" -lt 4 ]; then
    echo "  FAILED TO MEASURE: perception stack did not come up"
    return 1
  fi
  if ! timeout 25 ros2 topic echo /uav/uav0/state/camera_health --once >/dev/null 2>&1; then
    echo "  FAILED TO MEASURE: perception nodes exist but camera_health never spoke"
    return 1
  fi
  echo "  stack confirmed live"
}

stop_stack() {
  [ -n "$STACK_PID" ] || return 0
  kill "$STACK_PID" 2>/dev/null
  wait "$STACK_PID" 2>/dev/null
  STACK_PID=""
}

run_one() {
  local label=$1
  local want
  want=$(expected_mib_per_msg)
  echo
  echo "############ $label ############"
  echo -n "  installed sdf says: "
  grep -E '<update_rate>|<width>|<height>' "$DOWN_SDF" | tr -d ' ' | tr '\n' ' '
  echo "| expecting ${want} MiB/msg on down/image_raw"

  bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
  sleep 3
  bash "$WORKSPACE/scripts/start_sim.sh" >/tmp/budget_start.log 2>&1
  local deadline=$((SECONDS + 420))
  until timeout 20 ros2 topic list --no-daemon 2>/dev/null \
      | grep -q '/fmu/out/vehicle_odometry'; do
    if [ $SECONDS -gt $deadline ]; then
      echo "  FAILED TO MEASURE: PX4 never published for $label"
      unmeasured=$((unmeasured + 1))
      return
    fi
    sleep 10
  done
  sleep 10

  if ! start_stack; then
    unmeasured=$((unmeasured + 1))
    stop_stack
    return
  fi

  local out
  out=$(ros2 run uav_sim_gz image_rate_probe --ros-args -p seconds:="$SECONDS_WINDOW" 2>&1)
  # Keep the raw output. The grep below is a READING aid, not the record: on
  # 2026-08-24 it swallowed the probe's own failure text and the run reported
  # "never delivered a frame" with no way to see why.
  printf '%s
' "$out" > "/tmp/budget_probe_${label%% *}.log"
  echo "$out" | grep -E 'MiB|TOTAL|only [0-9]+ messages'

  # 🔴 The guard that the first run of this script did not have.
  local got
  got=$(echo "$out" | grep 'down/image_raw' | grep -oE '[0-9]+\.[0-9]+ MiB/msg' \
        | grep -oE '^[0-9.]+' | head -1)
  if [ -z "$got" ]; then
    echo "  FAILED TO MEASURE: down/image_raw never delivered a frame"
    echo "                     first lines of what the probe actually said:"
    printf '%s
' "$out" | head -8 | sed 's/^/                       /'
    echo "                     (full output: /tmp/budget_probe_${label%% *}.log)"
    unmeasured=$((unmeasured + 1))
    return
  fi
  if ! awk -v a="$got" -v b="$want" 'BEGIN {exit !(a > b - 0.02 && a < b + 0.02)}'; then
    echo "  FAILED TO MEASURE: down/image_raw carries ${got} MiB/msg but the sdf in use"
    echo "                     says ${want} -- the configuration change did NOT reach the"
    echo "                     simulator, so this round measures the previous setup."
    unmeasured=$((unmeasured + 1))
    return
  fi
  echo "  stimulus confirmed on the wire: ${got} MiB/msg matches the down sdf"

  local want_d got_d
  want_d=$(expected_depth_mib_per_msg)
  got_d=$(echo "$out" | grep 'front/depth_image' | grep -oE '[0-9]+\.[0-9]+ MiB/msg'           | grep -oE '^[0-9.]+' | head -1)
  if [ -z "$got_d" ]; then
    echo "  FAILED TO MEASURE: front/depth_image never delivered a frame"
    unmeasured=$((unmeasured + 1))
    return
  fi
  if ! awk -v a="$got_d" -v b="$want_d" 'BEGIN {exit !(a > b - 0.02 && a < b + 0.02)}'; then
    echo "  FAILED TO MEASURE: front/depth_image carries ${got_d} MiB/msg but the depth sdf"
    echo "                     in use says ${want_d} -- the change did NOT reach the simulator."
    unmeasured=$((unmeasured + 1))
    return
  fi
  echo "  stimulus confirmed on the wire: ${got_d} MiB/msg matches the depth sdf"
  # R27-1 for the transport change: an XML that failed to load is silent, so read
  # the segment size actually allocated rather than trusting the env var.
  # Do NOT delete stale segments to tidy the evidence: they belong to other live
  # participants and removing them under a running process is destructive. Read the
  # LARGEST segment instead -- if the profile loaded, one at least is 16777216.
  #
  # 🔴 REPORTED UNCONDITIONALLY since 2026-08-24. Until that day, an unset DDS_PROFILE
  # meant stock transport and this block was skipped. It no longer does: the profile is
  # now wired into start_sim.sh and sim.launch.py, so a run that sets nothing gets the
  # LARGE transport. Every A/B/C/D/E/F number recorded before that date was taken on
  # stock transport and is NOT comparable to a default run today. Set UAV_DDS_PROFILE=none
  # to reproduce those. Printing the segment on every run is what makes that visible
  # instead of a silent apples-to-oranges comparison.
  local biggest
  biggest=$(find /dev/shm -maxdepth 1 -name 'fastrtps_*' ! -name '*_el' -printf '%s
'     2>/dev/null | sort -n | tail -1)
  echo "  largest shm segment: ${biggest:-none} bytes (stock default 549408, large profile 16777216)"
  if [ "${biggest:-0}" -ge 16777216 ]; then
    echo "  transport in force: LARGE-SAMPLES (pre-2026-08-24 numbers are not comparable)"
  else
    echo "  transport in force: STOCK"
  fi
  if [ -n "$DDS_PROFILE" ] && [ "${biggest:-0}" -lt 16777216 ]; then
    echo "  FAILED TO MEASURE: the XML did not take effect -- a profile that fails to"
    echo "                     parse is SILENT, so the rates below would be the stock"
    echo "                     transport wearing a new label."
    unmeasured=$((unmeasured + 1))
    return
  fi
  echo -n "  RTF: "
  bash "$WORKSPACE/scripts/sample_rtf.sh" "$UAV_WORLD" 120
  stop_stack
}

reset_sdfs() { cp "$CANONICAL" "$DOWN_SDF"; cp "$DEPTH_CANON" "$DEPTH_SDF"; }

case " $CONFIGS " in *" A "*)
  reset_sdfs
  run_one "A baseline 640x480 @30Hz (as shipped)" ;;
esac

case " $CONFIGS " in *" B "*)
  reset_sdfs
  sed -i 's#<width>640</width>#<width>320</width>#; s#<height>480</height>#<height>240</height>#' "$DOWN_SDF"
  run_one "B 320x240 @30Hz (quarter the bytes, same rate)" ;;
esac

case " $CONFIGS " in *" C "*)
  reset_sdfs
  sed -i 's#<update_rate>30</update_rate>#<update_rate>15</update_rate>#' "$DOWN_SDF"
  run_one "C 640x480 @15Hz (half the messages, same pixels)" ;;
esac

bash "$WORKSPACE/scripts/stop_sim.sh" >/dev/null 2>&1
# D exists because the PERCEPTION=1 run showed the ceiling under load is BYTES,
# not messages -- and the most expensive message is not the down camera at all:
# depth is 640x480 R_FLOAT32 = 1.172 MiB, vs 0.879 for an RGB frame. D shrinks
# only depth, so it costs the marker/flow path NOTHING.
case " $CONFIGS " in *" D "*)
  reset_sdfs
  sed -i 's#<width>640</width>#<width>320</width>#; s#<height>480</height>#<height>240</height>#' "$DEPTH_SDF"
  grep -q '<width>320</width>' "$DEPTH_SDF" || { echo "FATAL: depth patch did not apply"; exit 2; }
  run_one "D depth 320x240 (down UNTOUCHED at 640x480)" ;;
esac

# F keeps 640x480 (so the clusterer grid, and therefore obstacle geometry, is
# untouched -- proven identical to the control by verify_depth_two_routes.sh) and
# halves the SOURCE rate instead. Open question is only what the bridge delivers.
case " $CONFIGS " in *" F "*)
  reset_sdfs
  sed -i 's#<update_rate>15</update_rate>#<update_rate>8</update_rate>#' "$DEPTH_SDF"
  grep -q '<update_rate>8</update_rate>' "$DEPTH_SDF" || { echo "FATAL: depth rate patch failed"; exit 2; }
  run_one "F depth 640x480 @8Hz (half the rate, full resolution)" ;;
esac

case " $CONFIGS " in *" E "*)
  reset_sdfs
  sed -i 's#<width>640</width>#<width>320</width>#; s#<height>480</height>#<height>240</height>#' "$DOWN_SDF"
  sed -i 's#<width>640</width>#<width>320</width>#; s#<height>480</height>#<height>240</height>#' "$DEPTH_SDF"
  run_one "E both down AND depth at 320x240" ;;
esac

echo
if [ "$PERCEPTION" = "1" ]; then
  cat <<'CRIT'
############ DECISION CRITERIA (perception stack running) ############
  The question this run answers: can the down camera be given a bigger share of
  the bridge WITHOUT starving the obstacle path?

  Baseline of record, P5.2 2026-08-13, same stack shape (package-status S4):
      front 5.5  |  depth 7.0  |  down 17.2  |  total 29.7 msg/s  |  RTF 0.877

  B is VIABLE only if all three hold in configuration B:
    1. depth >= 7.0 msg/s          -- obstacle_extractor_node was designed to
                                      exactly this delivered rate, not to the SDF rate
    2. depth gap max <= 1800 ms    -- the other half of that same design point
    3. down rises materially       -- B's whole premise is 70% -> 99% delivery.
                                      If down does not actually gain under load,
                                      B is all cost and no benefit: reject it.

  Read A first. If A does not land near the 2026-08-13 baseline, the budget has
  MOVED since then (the stack has gained safety/mission/observability nodes) --
  that is itself a finding, and A/B stays a valid comparison because both halves
  were measured in this same run.
CRIT
fi
echo "############ READ IT LIKE THIS ############"
echo "  total MiB/s ~equal in A and B  -> BANDWIDTH ceiling: resolution is the lever,"
echo "     and the frame rate never has to move (which is what optical flow needs)."
echo "  total msg/s ~equal in A and B  -> MESSAGE-RATE ceiling: only the rate helps,"
echo "     and lowering the down camera forces the P4.5/G6 re-verification."
echo "  C is the control: halving the rate must move whichever aggregate is NOT the ceiling."
if [ "$unmeasured" -ne 0 ]; then
  echo
  echo "  RESULT: FAILED TO MEASURE in $unmeasured round(s) -- no conclusion drawn."
  exit 2
fi
echo
echo "  RESULT: all three rounds measured what they claimed to."
