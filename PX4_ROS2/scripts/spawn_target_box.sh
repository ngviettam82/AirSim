#!/bin/bash
# Spawn a mobile target_box (P9.5) and set it moving at constant velocity.
# Usage: spawn_target_box.sh <world> <x> <y> [vx] [vy] [name]
# Env:   none
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
UAV_WORLD=${1:?usage: spawn_target_box.sh <world> <x> <y> [vx] [vy] [name]}
X=${2:?x required}
Y=${3:?y required}
VX=${4:-0.2}
VY=${5:-0.0}
NAME=${6:-target_box}

# 0.5x navigator max_speed_mps (navigation_params.yaml) -- P9-mission.md S3.
SPEED_CAP=0.275
read -r SPEED OVER_CAP <<< "$(python3 -c "
vx, vy, cap = $VX, $VY, $SPEED_CAP
speed = (vx ** 2 + vy ** 2) ** 0.5
print('%.4f %d' % (speed, 1 if speed > cap else 0))
")"
if [ "$OVER_CAP" = "1" ]; then
  echo "ERROR: requested speed ${SPEED} m/s exceeds hard cap ${SPEED_CAP} m/s" >&2
  exit 1
fi

BOX_SDF=$WORKSPACE/install/uav_sim_gz/share/uav_sim_gz/models/target_box/model.sdf
[ -f "$BOX_SDF" ] || { echo "FATAL: $BOX_SDF missing, build uav_sim_gz" >&2; exit 1; }

REQ_FILE=$(mktemp)
cat > "$REQ_FILE" <<EOF
sdf_filename: "$BOX_SDF"
name: "$NAME"
pose: {position: {x: $X, y: $Y, z: 0}}
EOF
REPLY=$(gz service -s "/world/$UAV_WORLD/create" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
  --req "$(cat "$REQ_FILE")")
rm -f "$REQ_FILE"
echo "spawn reply: $REPLY"
echo "$REPLY" | grep -q 'data: true' || { echo "FATAL: spawn not confirmed" >&2; exit 1; }

sleep 1
gz topic -t "/model/$NAME/cmd_vel" -m gz.msgs.Twist -p "linear: {x: $VX, y: $VY, z: 0.0}"
echo "target_box '$NAME' moving at (${VX}, ${VY}) m/s, speed ${SPEED} m/s, from ($X, $Y)"
