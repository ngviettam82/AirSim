#!/bin/bash
# R1/R25: prove px4_msgs stays inside uav_px4_backend.
# Usage: bash scripts/check_px4_msgs_boundary.sh
# Docs: .claude/memory.md S1 (R1, R25)
set -o pipefail

cd "$(dirname "$0")/.." || exit 1

# R34: the scanned set is derived, never hand-listed.
# An import/link/dependency is a real crossing; the word appearing inside a
# comment that says "never px4_msgs" is the opposite of a crossing.
#
# P12.0 (2026-08-25): the depend forms were hand-listed as three literals and
# <test_depend> was missing, so any package could take a px4_msgs dependency
# through its test tree and this gate still printed "violations: 0". Hand-listing
# a set that the ecosystem defines is the R34 shape; <[a-z_]*depend> now covers
# every REP-149 variant at once -- test_depend, build_export_depend,
# buildtool_depend, doc_depend included, plus the CMake target-link form.
#
# The pattern is assembled from $PKG rather than written out. A scanner that
# scans the directory it lives in will otherwise report itself -- and the old
# regex only escaped that by accident, because its own '^[^#/*]*' prefix could
# not step over the '#', '/' and '*' inside the pattern text on that same line.
#
# The comment guard moved out of the regex prefix in the same patch. It used to be a
# leading '^[^#/*]*', which also refused to step over the '/' of an XML closing tag --
# so a package.xml with two tags on one line slipped through entirely. The awk filter
# below drops comment lines instead, and '#include' is exempted there because it is the
# one crossing form that legitimately starts with '#'.
PKG=px4_msgs
CROSSING="(import ${PKG}|from ${PKG}|#include[[:space:]]*[<\"]${PKG}|<[a-z_]*depend>${PKG}|find_package[(]${PKG}|${PKG}::)"

# Declared exceptions. Each line: <path>  # <why it may cross>
# Adding a line here is a design decision, not a formality -- every entry is one
# more place to fix when PX4 changes its API, which is exactly what R1 buys.
ALLOWED_FILE=scripts/px4_msgs_boundary_allowlist.txt

echo "=== files outside uav_px4_backend that cross the px4_msgs boundary ==="
mapfile -t hits < <(
  grep -rEn "$CROSSING" src scripts \
    --include='*.py' --include='*.cpp' --include='*.hpp' \
    --include='*.xml' --include='*.txt' --include='*.sh' 2>/dev/null \
  | awk '{
      content = $0
      sub(/^[^:]*:[0-9]+:/, "", content)
      sub(/^[ \t]+/, "", content)
      if (content ~ /^(\/\/|\*|\/\*|<!--)/) next
      if (content ~ /^#/ && content !~ /^#include/) next
      sub(/:[0-9]+:.*$/, "")
      print
    }' \
  | grep -v '^src/uav_px4_backend/' | sort -u
)

allowed=0
violations=0
for hit in "${hits[@]}"; do
  [ -z "$hit" ] && continue
  if [ -f "$ALLOWED_FILE" ] && grep -q "^${hit}[[:space:]]" "$ALLOWED_FILE"; then
    reason=$(grep "^${hit}[[:space:]]" "$ALLOWED_FILE" | sed 's/^[^#]*# *//')
    echo "  ALLOWED   $hit"
    echo "            reason: $reason"
    allowed=$((allowed + 1))
  else
    echo "  VIOLATION $hit"
    grep -nE "$CROSSING" "$hit" | head -3 | sed 's/^/            /'
    violations=$((violations + 1))
  fi
done
[ ${#hits[@]} -eq 0 ] && echo "  (none)"

# An allowlist entry pointing at a file that no longer crosses is rot: it would
# silently keep permitting a path nobody re-reads.
stale=0
if [ -f "$ALLOWED_FILE" ]; then
  while read -r path _; do
    case "$path" in ''|'#'*) continue ;; esac
    if ! printf '%s\n' "${hits[@]}" | grep -qx "$path"; then
      echo "  STALE ALLOWLIST ENTRY: $path no longer crosses -- remove it"
      stale=$((stale + 1))
    fi
  done < "$ALLOWED_FILE"
fi

echo
echo "crossings: $((allowed + violations))  allowed: $allowed  violations: $violations  stale: $stale"
if [ "$violations" -ne 0 ] || [ "$stale" -ne 0 ]; then
  echo "RESULT: FAIL"
  exit 1
fi
echo "RESULT: PASS"
