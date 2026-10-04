#!/bin/bash
# Freeze/thaw the running ODM container via cgroup freezer.
# See docs/ops-playbook.md S5 (does not survive a WSL shutdown).
# Usage: bash odm_pause.sh pause|resume|status
set -uo pipefail

action=${1:-status}
id=$(docker ps -a --filter ancestor=opendronemap/odm --format '{{.ID}}' | head -1)

if [ -z "$id" ]; then
  echo "No ODM container found."
  exit 1
fi

case "$action" in
  pause)  docker pause "$id" >/dev/null && echo "PAUSED $id" ;;
  resume) docker unpause "$id" >/dev/null && echo "RESUMED $id" ;;
  status) ;;
  *) echo "usage: $0 pause|resume|status"; exit 2 ;;
esac

docker ps -a --filter "id=$id" --format 'container {{.ID}}  {{.Status}}'
DIR=$HOME/photogrammetry/bk_campus
echo "depth maps on disk: $(find "$DIR" -name '*.dmap' 2>/dev/null | wc -l)"
echo "openmvs work dir  : $(du -sh "$DIR/opensfm/undistorted/openmvs" 2>/dev/null | cut -f1)"
