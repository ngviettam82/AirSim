#!/bin/bash
# P10.9b: operator-facing "read the go/no-go light" tool. NEVER replace this
# with `ros2 topic echo --once /uav/<id>/state/system_health` -- that topic
# is Reliable/KeepLast(1)/TransientLocal, so a dead diagnostics_node's
# last-known-GO sample stays latched forever and echo has no way to tell you
# the node itself is gone. This script's own age self-check (R32, off the
# payload's wall_stamp_sec, see preflight_light.py) is the only thing that
# catches that -- docs/interface-contract-v0.1.md S:2.20's own required
# reading path, and the checklist obligation "read the light AGAIN after
# ARM, BEFORE takeoff" (same S:2.20 entry).
# Usage: preflight_light.sh [--uav-id uav0] [--max-age-sec 2.0] [--timeout 3.0]
set -o pipefail

source /opt/ros/humble/setup.bash
WORKSPACE=$HOME/PX4_ROS2
[ -f "$WORKSPACE/install/setup.bash" ] && source "$WORKSPACE/install/setup.bash"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/preflight_light.py" "$@"
exit "$?"
