#!/bin/bash
# Tests the transport hypothesis for debt #10: the ros_gz image bridge starves any
# stream whose samples do not fit the Fast DDS shared-memory segment. Measured
# 2026-08-24: every /dev/shm/fastrtps_* segment is exactly 549408 B (stock default),
# the two streams under it are delivered at 97% with zero loss, and the two over it
# at 25-53%. This run leaves EVERY sensor exactly as shipped and changes only the
# transport, so a gain here costs no fidelity anywhere.
#
# 🔴 WHY THIS IS A FILE AND NOT A COMMAND LINE. stop_sim.sh derives its kill patterns
# from the built packages (R34) and issues `pkill -f "<pkg>/"`. Any command line
# containing a path like `uav_bringup/config/...` therefore matches and the caller
# kills ITSELF -- exactly what happened on the first attempt at this trial (SIGHUP at
# the stop_sim line, exit 15). The old warning only listed infrastructure names such
# as `gz sim`; since R34 made the patterns derived, EVERY `uav_<pkg>/` substring is
# lethal on a command line. Keeping the paths inside this file keeps them off it.
#
# Usage: bash scripts/run_dds_transport_trial.sh
set -o pipefail

CANON=/mnt/c/code/PX4_ROS2
PROFILE=$CANON/src/uav_bringup/config/fastdds_large_samples.xml

[ -f "$PROFILE" ] || { echo "FATAL: profile missing"; exit 2; }

source /opt/ros/humble/setup.bash
source "$HOME/PX4_ROS2/install/setup.bash"

# Stale Fast DDS port/segment files cannot be reused with different transport
# parameters: the first attempt failed with
#   RTPS_TRANSPORT_SHM Error: Failed init_port fastrtps_port7455: open_and_lock_file
# Clean them HERE, where stop_sim has just run and nothing is alive -- never while
# processes are running, because those files belong to live participants.
bash "$CANON/scripts/stop_sim.sh" >/dev/null 2>&1
ros2 daemon stop >/dev/null 2>&1
sleep 2
if ps -eo comm | grep -qE '^gz$|^px4$|^ros2$'; then
  echo "FATAL: something is still alive; refusing to touch /dev/shm"
  exit 2
fi
removed=$(find /dev/shm -maxdepth 1 -name 'fastrtps_*' -print -delete 2>/dev/null | wc -l)
echo "cleared $removed stale Fast DDS shm files"

export DDS_PROFILE="$PROFILE"
export PERCEPTION=1
export CONFIGS="A"
export SECONDS_WINDOW=60

bash "$CANON/scripts/measure_image_budget.sh"
