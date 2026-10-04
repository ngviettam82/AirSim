#!/bin/bash
# Measure line and branch coverage, and print the list this project actually gates on.
#
# WHAT THE NUMBER IS FOR (P12.3, 2026-08-25). The percentage is an INVENTORY TOOL, not a
# gate. This project already refuses percentage thresholds, and for a reason it paid for:
# CLAUDE.md section 5 records peakAcceleration() reporting 0.897 m/s2 for a real 55.7 --
# "a safety number that lies is more dangerous than no number at all". 90% coverage with
# the safety branch uncovered is exactly that shape. So what this script is really for is
# uncovered_by_file.txt: the list a human classifies into (a) safety-relevant, must be
# tested, (b) not safety-relevant, reason recorded, (c) dead, delete. G-SIM S3 asks for
# zero lines left unclassified -- it never asks for a percentage.
#
# SIX THINGS THAT WILL SILENTLY GIVE A WRONG ANSWER. Each flag below is here because
# leaving it out produces a plausible number rather than an error:
#
#   build-cov/install-cov      A separate build tree. Instrumenting build/ leaves
#                              --coverage in CMakeCache forever, and verify_workspace.sh
#                              calls plain `colcon build` -- it would quietly measure the
#                              instrumented binaries from then on, with nothing saying so.
#                              Not hypothetical: build/uav_navigation was found stuck on
#                              CMAKE_BUILD_TYPE=Release the same way.
#   -DCMAKE_BUILD_TYPE=        Force "no build type", matching 13 of 14 packages, so -O3
#                              -DNDEBUG cannot leak in from a stale cache. NDEBUG has
#                              caused a real bug here before (bt_nodes.cpp).
#   -O0 -g                     gcov attributes lines correctly. Any -O inlines and unrolls,
#                              so "uncovered line" then points at the wrong place.
#   -fprofile-abs-path         .gcno records absolute paths, so geninfo does not get lost
#                              when the build tree is not the source tree.
#   -fprofile-update=atomic    Ten test files and seven mains use MultiThreadedExecutor.
#                              Without this, counts are LOST under concurrency and lines
#                              that did run are reported as never run.
#   --executor sequential      Not caution -- required. uav_safety and uav_mission load
#                              uav_control_authority's library, and uav_world_model calls
#                              into uav_perception. Run in parallel they write the same
#                              .gcda file.
#
# AND THE ONE THAT MATTERS MOST: `lcov --capture --initial`. Without it, code that no test
# ever entered is ABSENT from the report instead of showing 0% -- roughly 3300 lines of
# node code, including all five uav_px4_backend nodes, the package closest to the flight
# controller. The total would go UP by hiding the worst of it.
#
# Usage: bash scripts/measure_coverage.sh [outdir]
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
OUT=${1:-$WORKSPACE/gate_logs/coverage}

# R34: DERIVE the package set. A hand-kept list of nine sat here until 2026-08-26 and it
# had already gone wrong the quiet way: uav_sim_gz ships one .cpp and was simply absent,
# so its lines could never appear in uncovered_by_file.txt and S3 could answer "every
# uncovered line is classified" without ever having looked at that file. Any package with
# a non-test .cpp is measured; the three with none (uav_bringup, uav_interfaces) drop out
# by measurement, not by opinion.
PACKAGES=$(
  for d in "$WORKSPACE"/src/*/; do
    pkg=$(basename "$d")
    if find "$d" -name '*.cpp' -not -path '*/test/*' -print -quit | grep -q .; then
      printf '%s ' "$pkg"
    fi
  done
)
echo "  packages measured: $PACKAGES"

# R15 #2: never share install/ with another build. `pgrep -af '[c]olcon'` was tried first
# and matched THIS script's own command line through bash -lc; comparing process names is
# the version that does not accuse itself.
running=$(ps -eo comm | grep -cE '^(colcon|ctest|cc1plus)$')
if [ "$running" -ne 0 ]; then
  echo "FATAL: $running build/test process(es) running. Coverage needs the workspace to itself."
  exit 2
fi

mkdir -p "$OUT"
cd "$WORKSPACE" || exit 1
source /opt/ros/humble/setup.bash
# Underlay first: px4_msgs and the interface package stay UNinstrumented on purpose. They
# are generated code, and letting rosidl output into the report dilutes it with hundreds of
# trivially-covered lines that mean nothing about hand-written logic.
source install/setup.bash

COV_FLAGS="-O0 -g --coverage -fprofile-abs-path -fprofile-update=atomic"

# Stamped BEFORE the build, not after the run. A coverage pass takes ~20 minutes; if the
# staleness check downstream compared the age of the OUTPUT file, an edit made while the
# run was in flight would look older than the result and pass. What matters is the source
# the binaries were compiled from.
find "$WORKSPACE/src" -type f \( -name '*.cpp' -o -name '*.hpp' \) -printf '%T@\n' \
  2>/dev/null | sort -rn | head -1 | cut -d. -f1 > "$OUT/source_epoch.txt"
echo "  source epoch     : $(date -d @"$(cat "$OUT/source_epoch.txt")" '+%F %T')"

echo "=== 1/5 build (separate tree: build-cov) ==="
colcon build \
  --build-base build-cov --install-base install-cov \
  --packages-select $PACKAGES \
  --cmake-args \
    -DCMAKE_BUILD_TYPE= \
    -DCMAKE_C_FLAGS="$COV_FLAGS" \
    -DCMAKE_CXX_FLAGS="$COV_FLAGS" \
    -DCMAKE_EXE_LINKER_FLAGS="--coverage" \
    -DCMAKE_SHARED_LINKER_FLAGS="--coverage" \
  > "$OUT/build.log" 2>&1 || { echo "BUILD FAILED"; tail -25 "$OUT/build.log"; exit 1; }
tail -n 2 "$OUT/build.log"

echo "=== 2/5 test (sequential, mandatory) ==="
source install-cov/setup.bash
colcon test --build-base build-cov --install-base install-cov \
  --executor sequential --packages-select $PACKAGES \
  > "$OUT/test.log" 2>&1 || true
colcon test-result --test-result-base build-cov --all 2>&1 | tail -n 3 | tee "$OUT/test-result.txt"
# A test that died wrote no .gcda, so its lines read as "never run". Say so rather than
# letting the report imply the code was reached and skipped.
echo "  (any failure above means those lines are UNMEASURED, not uncovered)"

echo "=== 3/5 capture, with the baseline ==="
R="--rc lcov_branch_coverage=1"
lcov $R --capture --initial --directory build-cov -o "$OUT/base.info" > "$OUT/lcov.log" 2>&1
lcov $R --capture           --directory build-cov -o "$OUT/test.info" >> "$OUT/lcov.log" 2>&1
lcov $R -a "$OUT/base.info" -a "$OUT/test.info" -o "$OUT/all.info" >> "$OUT/lcov.log" 2>&1

echo "=== 4/5 keep only hand-written product code ==="
lcov $R --extract "$OUT/all.info" \
  "$WORKSPACE/src/uav_*/src/*" "$WORKSPACE/src/uav_*/include/*" \
  -o "$OUT/prod.info" >> "$OUT/lcov.log" 2>&1
lcov $R --remove "$OUT/prod.info" "*/test/*" -o "$OUT/final.info" >> "$OUT/lcov.log" 2>&1
genhtml "$OUT/final.info" --branch-coverage -o "$OUT/html" >> "$OUT/lcov.log" 2>&1

echo "=== 5/5 the list that is actually gated on ==="
# Every line gcov saw zero times, grouped by file. THIS is S3's input; the percentages
# below are context for reading it, nothing more.
awk '
  /^SF:/ { file = substr($0, 4); next }
  /^DA:/ {
    split(substr($0, 4), a, ",")
    if (a[2] == 0) { print file ":" a[1] }
  }
' "$OUT/final.info" | sort > "$OUT/uncovered_by_file.txt"

uncovered=$(wc -l < "$OUT/uncovered_by_file.txt")
files=$(cut -d: -f1 "$OUT/uncovered_by_file.txt" | sort -u | wc -l)
echo "  uncovered lines: $uncovered  across $files file(s)"
echo "  full list      : $OUT/uncovered_by_file.txt"
echo "  browsable      : $OUT/html/index.html"
echo
lcov $R --summary "$OUT/final.info" 2>&1 | sed 's/^/  /'
echo
echo "🔴 The percentages above are INVENTORY, not a verdict. G-SIM S3 asks for something"
echo "   else: every line in uncovered_by_file.txt classified as (a) safety-relevant and"
echo "   now tested, (b) not safety-relevant with the reason written down, or (c) dead and"
echo "   deleted. Zero lines left unclassified. No threshold, ever."
