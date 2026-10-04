#!/bin/bash
# Stage S1.1: sparse reconstruction only, ahead of gate G1'.
# See docs/ops-playbook.md S5 for planar/concurrency rationale.
set -uo pipefail

PROJECT_ROOT=${PROJECT_ROOT:-$HOME/photogrammetry}
PROJECT=${PROJECT:-bk_campus}
DIR="$PROJECT_ROOT/$PROJECT"
FEATURES="$DIR/opensfm/features"

n_img=$(ls "$DIR"/images/*.JPG 2>/dev/null | wc -l)
n_npz=$(ls "$FEATURES"/*.features.npz 2>/dev/null | wc -l)
echo "images=$n_img  cached features=$n_npz"

# Partial feature cache: force --rerun-from opensfm (see ops-playbook S5).
EXTRA=()
if [ -d "$FEATURES" ] && [ "$n_npz" -lt "$n_img" ]; then
  echo "PARTIAL feature cache ($((n_img - n_npz)) missing) -> forcing --rerun-from opensfm"
  EXTRA+=(--rerun-from opensfm)
fi

LOG="$DIR/odm_sfm_$(date +%Y%m%d_%H%M%S).log"
echo "log: $LOG"

docker run --rm \
  -v "$PROJECT_ROOT":/datasets \
  opendronemap/odm \
  --project-path /datasets "$PROJECT" \
  --sfm-algorithm planar \
  --gps-accuracy 3 \
  --feature-quality high \
  --max-concurrency 2 \
  --end-with opensfm \
  "${EXTRA[@]}" 2>&1 | tee "$LOG"
status=${PIPESTATUS[0]}

n_npz_after=$(ls "$FEATURES"/*.features.npz 2>/dev/null | wc -l)
echo
echo "=== post-run check ==="
echo "features: $n_npz_after / $n_img"
if [ -f "$DIR/opensfm/reconstruction.json" ]; then
  echo "reconstruction.json: present ($(stat -c%s "$DIR/opensfm/reconstruction.json") bytes)"
else
  echo "reconstruction.json: ABSENT"
fi
echo "exit status: $status"
exit "$status"
