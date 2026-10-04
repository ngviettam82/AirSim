#!/bin/bash
# Stage S1.3: dense reconstruction, DSM/DTM, orthophoto (continues sparse block).
# See docs/ops-playbook.md S5 for smrf-window/dem-resolution sizing.
set -uo pipefail

PROJECT_ROOT=${PROJECT_ROOT:-$HOME/photogrammetry}
PROJECT=${PROJECT:-bk_campus}
DIR="$PROJECT_ROOT/$PROJECT"
LOG="$DIR/odm_dense_$(date +%Y%m%d_%H%M%S).log"

if [ ! -f "$DIR/opensfm/reconstruction.json" ]; then
  echo "No sparse reconstruction; run run_odm_sfm.sh first."
  exit 1
fi
echo "log: $LOG"
df -h / | tail -1

docker run --rm \
  -v "$PROJECT_ROOT":/datasets \
  opendronemap/odm \
  --project-path /datasets "$PROJECT" \
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
  --orthophoto-resolution 5 2>&1 | tee "$LOG"
status=${PIPESTATUS[0]}

echo
echo "=== products ==="
for f in odm_dem/dsm.tif odm_dem/dtm.tif odm_orthophoto/odm_orthophoto.tif \
         odm_georeferencing/odm_georeferenced_model.laz; do
  if [ -f "$DIR/$f" ]; then
    echo "  OK      $f  ($(stat -c%s "$DIR/$f") bytes)"
  else
    echo "  ABSENT  $f"
  fi
done
df -h / | tail -1
echo "exit status: $status"
exit "$status"
