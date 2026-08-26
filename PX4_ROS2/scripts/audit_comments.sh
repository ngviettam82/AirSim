#!/bin/bash
# R16 audit: comment share per package + long comment-block runs.
set -o pipefail

cd /mnt/c/code/PX4_ROS2 || exit 1

echo "=== comment share by package ==="
# Derived, never hand-copied: a hand-kept list silently narrowed scope (missed 3 newest packages).
for dir in $(find src -maxdepth 1 -mindepth 1 -type d | sort) scripts; do
  total=0
  comments=0
  files=0
  while IFS= read -r file; do
    lines=$(wc -l < "$file")
    marked=$(grep -cE '^[[:space:]]*(//|#|\*|/\*)' "$file")
    total=$((total + lines))
    comments=$((comments + marked))
    files=$((files + 1))
  done < <(find "$dir" -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.py' \
             -o -name '*.sh' \) 2>/dev/null | grep -v __pycache__)
  if [ "$total" -gt 0 ]; then
    printf '  %-24s %3d files %6d lines %5d comment %3d%%\n' \
      "$dir" "$files" "$total" "$comments" $((100 * comments / total))
  fi
done

echo
echo "=== files with a run of 4+ consecutive comment lines ==="
find src scripts -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.py' -o -name '*.sh' \) \
  2>/dev/null | grep -v __pycache__ | sort | while IFS= read -r file; do
  worst=$(awk '
    /^[[:space:]]*(\/\/|#)/ {run++; if (run > max) max = run; next}
    {run = 0}
    END {print max + 0}' "$file")
  if [ "$worst" -ge 4 ]; then
    printf '  %-58s longest block %2d lines\n' "$file" "$worst"
  fi
done
