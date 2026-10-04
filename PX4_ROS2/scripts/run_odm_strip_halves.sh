#!/bin/bash
# Gate G1''': rebuild from odd/even strips for cross-validation (DSM only).
# See docs/ops-playbook.md S5 (hard links, not copies).
set -uo pipefail

ROOT=${ROOT:-$HOME/photogrammetry}
SRC=$ROOT/bk_campus/images
LISTS=${LISTS:-/mnt/c/Users/ASUS/AppData/Local/Temp/claude/C--code-PX4-ROS2/949c26a1-315d-4ceb-9bd1-c75cbad1462b/scratchpad/splits}

for half in odd even; do
  project=bk_$half
  dir=$ROOT/$project/images
  list=$LISTS/strips_$half.txt
  echo "=== staging $project from $list ==="
  mkdir -p "$dir"
  staged=0
  # tr -d '\r': Windows line endings break filename lookup.
  while read -r name; do
    [ -z "$name" ] && continue
    if [ ! -e "$dir/$name" ]; then
      ln "$SRC/$name" "$dir/$name" 2>/dev/null || cp "$SRC/$name" "$dir/$name"
      staged=$((staged + 1))
    fi
  done < <(tr -d '\r' < "$list")
  present=$(ls "$dir" | wc -l)
  echo "staged $staged; total $present images"
  if [ "$present" -lt 100 ]; then
    echo "FATAL: staging produced $present images - refusing to run ODM on nothing"
    exit 1
  fi
done

df -h / | tail -1

for half in odd even; do
  project=bk_$half
  log=$ROOT/$project/odm_$(date +%Y%m%d_%H%M%S).log
  echo
  echo "=== reconstructing $project ==="
  docker run --rm -v "$ROOT":/datasets opendronemap/odm \
    --project-path /datasets "$project" \
    --sfm-algorithm planar \
    --gps-accuracy 3 \
    --feature-quality high \
    --max-concurrency 2 \
    --pc-quality medium \
    --pc-rectify \
    --auto-boundary \
    --dsm --dtm \
    --smrf-window 30 \
    --dem-resolution 25 \
    --skip-orthophoto 2>&1 | tee "$log"
  echo "$project exit ${PIPESTATUS[0]}"
  ls -la "$ROOT/$project/odm_dem/" 2>/dev/null | head -5
done

echo
echo "=== done ==="
for half in odd even; do
  f=$ROOT/bk_$half/odm_dem/dsm.tif
  printf "  bk_%-5s dsm.tif: %s\n" "$half" "$([ -f "$f" ] && du -h "$f" | cut -f1 || echo ABSENT)"
done
df -h / | tail -1
