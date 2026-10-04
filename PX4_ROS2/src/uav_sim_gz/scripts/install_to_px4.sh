#!/bin/bash
# Register this package's airframes and worlds with a PX4-Autopilot checkout.
#
# Models are NOT installed here: PX4 appends its own paths to GZ_SIM_RESOURCE_PATH,
# so the workspace env hook is enough. Airframes must live in ROMFS (path-based
# lookup + CMake manifest) and worlds are resolved as $PX4_GZ_WORLDS/<name>.sdf.
#
# Safe to re-run. Usage: install_to_px4.sh [px4_dir] [package_source_dir]

set -euo pipefail

PX4_DIR="${1:-$HOME/PX4-Autopilot}"
PKG_DIR="${2:-$HOME/PX4_ROS2/src/uav_sim_gz}"

AIRFRAME_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
AIRFRAME_MANIFEST="$AIRFRAME_DIR/CMakeLists.txt"
WORLD_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
ANCHOR="4011_gz_lawnmower"

for required in "$AIRFRAME_DIR" "$WORLD_DIR" "$PKG_DIR/airframes" "$PKG_DIR/worlds"; do
  if [ ! -d "$required" ]; then
    echo "ERROR: missing directory: $required" >&2
    exit 1
  fi
done

if ! grep -q "$ANCHOR" "$AIRFRAME_MANIFEST"; then
  echo "ERROR: anchor '$ANCHOR' not found in $AIRFRAME_MANIFEST." >&2
  echo "       PX4 layout changed; update this script instead of guessing." >&2
  exit 1
fi

needs_px4_rebuild=0

echo "== airframes =="
for airframe_path in "$PKG_DIR"/airframes/[0-9]*; do
  airframe="$(basename "$airframe_path")"
  ln -sfn "$airframe_path" "$AIRFRAME_DIR/$airframe"
  echo "  linked $airframe"

  if grep -qE "^[[:space:]]*$airframe[[:space:]]*$" "$AIRFRAME_MANIFEST"; then
    echo "  already in CMakeLists"
  else
    # Keep the entry adjacent to the other gz airframes.
    sed -i "/^[[:space:]]*$ANCHOR[[:space:]]*$/a\\\t$airframe" "$AIRFRAME_MANIFEST"
    echo "  added to CMakeLists"
    needs_px4_rebuild=1
  fi
done

echo "== worlds =="
for world_path in "$PKG_DIR"/worlds/*.sdf; do
  world="$(basename "$world_path")"
  ln -sfn "$world_path" "$WORLD_DIR/$world"
  echo "  linked $world"
done

echo
echo "Symlinks point into $PKG_DIR - editing the package updates PX4 with no reinstall."
if [ "$needs_px4_rebuild" -eq 1 ]; then
  echo "ACTION REQUIRED: new airframes were registered, PX4 must be rebuilt:"
  echo "  cd $PX4_DIR && make px4_sitl"
else
  echo "No new airframes registered; PX4 rebuild not required."
fi
