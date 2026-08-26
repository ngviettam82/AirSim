#!/bin/bash
# Stop every sim process started by start_sim.sh.
# See docs/ops-playbook.md S1 (must be a file, not inline pkill).
set -o pipefail

WORKSPACE="$HOME/PX4_ROS2"

# VÀNG#2 / R34: DERIVE the uav_* package kill-pattern list from install/
# instead of hand-keeping it -- a hardcoded list has already missed a
# newly-added package (twice now, per memory.md R34), and a missed package
# here means its action server + services survive as orphans and can be
# zombie-swallowed by the SAME node re-launched on the next bus (worst case:
# a service call like /mission/abort silently goes to a dead process).
# uav_interfaces (msg/srv/action-only, no lib/<pkg> dir) is correctly
# excluded BY CONSTRUCTION, not by an exclusion list.
UAV_PACKAGES_ALL=()
UAV_KILL_PATTERNS=()
if [ -d "$WORKSPACE/install" ]; then
  for pkg_dir in "$WORKSPACE"/install/uav_*/; do
    [ -d "$pkg_dir" ] || continue
    pkg=$(basename "$pkg_dir")
    UAV_PACKAGES_ALL+=("$pkg")
    if [ -d "${pkg_dir}lib/$pkg" ] \
       && find "${pkg_dir}lib/$pkg" -maxdepth 1 -type f -executable -print -quit 2>/dev/null | grep -q .; then
      UAV_KILL_PATTERNS+=("$pkg/")
    fi
  done
else
  echo "WARNING: $WORKSPACE/install missing -- cannot derive uav_* package kill patterns" \
    "(nothing built yet, or wrong WORKSPACE); falling back to infra-only cleanup"
fi
echo "derived uav_* packages (${#UAV_PACKAGES_ALL[@]}): ${UAV_PACKAGES_ALL[*]}"
echo "derived kill patterns, i.e. packages with a built node executable (${#UAV_KILL_PATTERNS[@]}): ${UAV_KILL_PATTERNS[*]}"

# P10.6: SIGINT first so rosbag_manager_node flushes + closes its bag
# (metadata.yaml is only written on a clean close, G-O1) before the
# unconditional SIGTERM sweep below would otherwise hard-kill it.
pkill -INT -f 'uav_observability/rosbag_manager_node' 2>/dev/null
pkill -INT -f 'uav_observability/diagnostics_node' 2>/dev/null
pkill -INT -f 'uav_observability/event_logger_node' 2>/dev/null
sleep 2

for pattern in \
  'MicroXRCEAgent' \
  'px4_sitl_default/bin/px4' \
  'gz sim' \
  'parameter_bridge' \
  'image_bridge' \
  'ros2 launch uav_sim_gz' \
  'ros2 launch uav_bringup' \
  "${UAV_KILL_PATTERNS[@]}" \
  'make px4_sitl' \
  'sleep infinity'
do
  pkill -f "$pattern" 2>/dev/null
done

sleep 3

# A wedged gz server ignores SIGTERM and holds the GPU.
survivors=$(pgrep -f 'gz sim|px4_sitl_default/bin/px4|MicroXRCEAgent' 2>/dev/null)
if [ -n "$survivors" ]; then
  echo "$survivors" | xargs -r kill -9 2>/dev/null
  sleep 2
fi

leftover=$(pgrep -af 'gz sim|px4_sitl_default/bin/px4|MicroXRCEAgent' 2>/dev/null)
if [ -n "$leftover" ]; then
  echo "still running:"
  echo "$leftover"
  exit 1
fi
echo "sim stopped, clean"
