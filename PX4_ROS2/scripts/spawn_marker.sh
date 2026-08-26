#!/bin/bash
# Spawn an ArUco marker board at a known pose (extracted from P5.3, reused by P9.5).
# Usage: spawn_marker.sh <world> <marker_id> <x> <y> [z] [name]
# Env:   none
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
UAV_WORLD=${1:?usage: spawn_marker.sh <world> <marker_id> <x> <y> [z] [name]}
MARKER_ID=${2:?marker id required}
X=${3:?x required}
Y=${4:?y required}
Z=${5:-0.01}
NAME=${6:-aruco_$MARKER_ID}

MARKER_SDF=$WORKSPACE/install/uav_sim_gz/share/uav_sim_gz/models/aruco_$MARKER_ID/model.sdf
[ -f "$MARKER_SDF" ] || { echo "FATAL: $MARKER_SDF missing, build uav_sim_gz" >&2; exit 1; }

REQ_FILE=$(mktemp)
cat > "$REQ_FILE" <<EOF
sdf_filename: "$MARKER_SDF"
name: "$NAME"
pose: {position: {x: $X, y: $Y, z: $Z}}
EOF
REPLY=$(gz service -s "/world/$UAV_WORLD/create" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
  --req "$(cat "$REQ_FILE")")
rm -f "$REQ_FILE"
echo "spawn reply: $REPLY"
sleep 3
echo -n "  marker present in the world: "
gz model --list 2>/dev/null | grep -c "$NAME"
echo "$REPLY" | grep -q 'data: true'
