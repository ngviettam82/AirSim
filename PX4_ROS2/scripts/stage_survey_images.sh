#!/bin/bash
# Copy survey images into ext4; verify bytes + JPEG markers.
# See docs/ops-playbook.md S5 (a truncated file once cost a run).
set -uo pipefail

SRC=${SRC:-/mnt/c/code/Python/MapDienHong_v2}
DST=${DST:-$HOME/photogrammetry/bk_campus/images}

mkdir -p "$DST"

echo "=== copying missing or mismatched files ==="
copied=0
for path in "$SRC"/*.JPG; do
  name=$(basename "$path")
  want=$(stat -c%s "$path")
  have=$(stat -c%s "$DST/$name" 2>/dev/null || echo -1)
  if [ "$want" != "$have" ]; then
    cp -f "$path" "$DST/$name"
    copied=$((copied + 1))
    echo "  copied $name ($want bytes)"
  fi
done
echo "files (re)copied: $copied"

echo
echo "=== verification: size + JPEG SOI/EOI markers ==="
bad=0
for path in "$SRC"/*.JPG; do
  name=$(basename "$path")
  want=$(stat -c%s "$path")
  have=$(stat -c%s "$DST/$name" 2>/dev/null || echo -1)
  if [ "$want" != "$have" ]; then
    echo "  FAIL size    $name  src=$want dst=$have"; bad=$((bad + 1)); continue
  fi
  soi=$(head -c2 "$DST/$name" | od -An -tx1 | tr -d ' \n')
  eoi=$(tail -c2 "$DST/$name" | od -An -tx1 | tr -d ' \n')
  if [ "$soi" != "ffd8" ]; then echo "  FAIL SOI     $name"; bad=$((bad + 1)); fi
  if [ "$eoi" != "ffd9" ]; then echo "  FAIL EOI     $name"; bad=$((bad + 1)); fi
done

total=$(ls "$SRC"/*.JPG | wc -l)
echo
echo "checked $total files, $bad problems"
if [ "$bad" -ne 0 ]; then
  echo "STAGING FAILED - do not start ODM"
  exit 1
fi
echo "STAGING OK - every file matches source byte count and is a complete JPEG"
