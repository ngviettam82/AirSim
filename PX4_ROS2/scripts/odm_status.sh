#!/bin/bash
# Progress of the running ODM job, measured rather than guessed.
DIR=${DIR:-$HOME/photogrammetry/bk_campus}
F="$DIR/opensfm/features"

echo "=== container ==="
docker ps --format '{{.Image}}  {{.Status}}' || true

echo
echo "=== feature extraction ==="
n_img=$(ls "$DIR"/images/*.JPG 2>/dev/null | wc -l)
a=$(ls "$F"/*.features.npz 2>/dev/null | wc -l)
sleep 30
b=$(ls "$F"/*.features.npz 2>/dev/null | wc -l)
rate=$(awk "BEGIN{printf \"%.2f\", ($b-$a)/30.0}")
echo "features: $b / $n_img   (+$((b - a)) in 30 s = $rate img/s)"
if [ "$b" -gt "$a" ] && [ "$b" -lt "$n_img" ]; then
  awk "BEGIN{printf \"eta for extraction: %.0f min\n\", ($n_img-$b)/(($b-$a)/30.0)/60.0}"
elif [ "$b" -ge "$n_img" ]; then
  echo "extraction COMPLETE - now in matching/reconstruction"
else
  echo "no growth in 30 s - either past extraction, or stalled"
fi

echo
echo "=== stage reached ==="
newest=$(ls -t "$DIR"/odm_sfm_*.log 2>/dev/null | head -1)
echo "log: $newest"
grep -oE "running \"[^\"]*opensfm\" [a-z_]+" "$newest" 2>/dev/null | tail -3
tail -3 "$newest" 2>/dev/null

echo
echo "=== matching progress (if reached) ==="
grep -c "Matching .* and " "$newest" 2>/dev/null || echo 0
grep -oE "Matching [0-9]+ image pairs" "$newest" 2>/dev/null | tail -1

echo
echo "=== memory headroom ==="
free -h | head -2
echo "reconstruction.json: $([ -f "$DIR/opensfm/reconstruction.json" ] && stat -c%s "$DIR/opensfm/reconstruction.json" || echo ABSENT)"
