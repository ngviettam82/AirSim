#!/bin/bash
# Prove each ROS-node unit test really runs with its own ROS_DOMAIN_ID (R20).
#
# 🔴 NOT a cheap static check: stage 3/3 runs a FULL `colcon test` over every
# package that has a ctest target (~11 min). Learned the hard way 2026-08-24 --
# it was launched twice while verify_workspace.sh was testing, three ctest runs
# then stomped each other's Testing/*.xml and the cross-check reported 1105 vs
# 1116. Nothing was wrong with the code; the MEASUREMENT was wrong. Hence the
# guard below: never let this run beside another test run, in either direction.
#
# Usage: check_test_domain_isolation.sh   (env: SKIP_SUITE=1 for stages 1-2 only)
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2

# The guard exists for stage 3/3 only, so it applies only when stage 3/3 will run.
# Until 2026-08-25 it fired unconditionally -- including when SKIP_SUITE=1 had already
# turned that stage off -- so the remedy printed in its own message did not work, and the
# check became unrunnable whenever anything else was testing. That is how a gate teaches
# people to skip it. Stages 1-2 read declarations and start no test of their own.
if [ -z "$SKIP_SUITE" ] && ps -eo comm | grep -qE '^ctest$'; then
  echo "FATAL: a ctest run is already in progress -- stage 3/3 of this script would"
  echo "       interleave with it and corrupt BOTH sets of results. Wait, or run with"
  echo "       SKIP_SUITE=1 for the static stages only."
  exit 2
fi
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
cd "$WORKSPACE" || exit 1

# R34: derive both the packages and the domains. The old hand-kept list froze at
# 5 packages and stopped covering 96/97/98/99 as they were added -- an R20 check
# that reports green on packages it never looked at is worse than no check.
PACKAGES=$(find src -maxdepth 1 -mindepth 1 -type d -printf '%f\n' | sort)
if [ -z "$PACKAGES" ]; then
  echo "FATAL: no packages found under $WORKSPACE/src"
  exit 1
fi

violations=0
domains=""
isolated_packages=""

echo "=== 1/3 what ctest was told to run ($(echo "$PACKAGES" | wc -w) packages derived) ==="
for package in $PACKAGES; do
  file=build/$package/CTestTestfile.cmake
  # Comments are stripped: a line saying "no rclcpp::init here" is not a call.
  creates_participant=no
  if ls src/"$package"/test/*.cpp >/dev/null 2>&1; then
    if sed 's://.*::' src/"$package"/test/*.cpp | grep -q 'rclcpp::init'; then
      creates_participant=yes
    fi
  fi

  if [ ! -f "$file" ]; then
    if [ "$creates_participant" = yes ]; then
      echo "  $package: NO CTestTestfile but tests call rclcpp::init -- build it before trusting this"
      violations=$((violations + 1))
    else
      echo "  $package: no CTestTestfile (no ctest target)"
    fi
    continue
  fi

  found=$(grep -o 'ROS_DOMAIN_ID=[0-9]*' "$file" | sort -u | tr '\n' ' ')
  if [ -n "$found" ]; then
    domains="$domains $(echo "$found" | grep -o '[0-9]*')"
    isolated_packages="$isolated_packages $package"
    echo "  $package: $found"
  elif [ "$creates_participant" = yes ]; then
    echo "  $package: none -- VIOLATION (tests call rclcpp::init on the ambient domain)"
    violations=$((violations + 1))
  else
    echo "  $package: none (ROS-free tests, nothing to isolate)"
  fi
done

domains=$(echo "$domains" | tr ' ' '\n' | grep -v '^$' | sort -un | tr '\n' ' ')

echo
echo "=== 2/3 what a test process actually inherits ==="
echo "ambient ROS_DOMAIN_ID: '${ROS_DOMAIN_ID:-unset}'"
for domain in $domains; do
  value=$(python3 -u /opt/ros/humble/share/ament_cmake_test/cmake/run_test.py \
    /tmp/domain_probe.xml --package-name probe --output-file /tmp/domain_probe.txt \
    --env "ROS_DOMAIN_ID=$domain" --command /usr/bin/printenv ROS_DOMAIN_ID 2>&1 |
    grep -E '^[0-9]+$' | head -1)
  echo "  asked $domain -> process saw ${value:-NOTHING}"
  if [ "$value" != "$domain" ]; then
    violations=$((violations + 1))
  fi
done

echo
if [ -n "$SKIP_SUITE" ]; then
  echo "=== 3/3 SKIPPED by SKIP_SUITE -- the domains above are declared, not exercised ==="
else
  echo "=== 3/3 the suites still pass on those domains (full colcon test, ~11 min) ==="
  # shellcheck disable=SC2086
  colcon test --packages-select $isolated_packages 2>&1 | tail -4
fi

echo
echo "domains in use: ${domains:-none}"
if [ "$violations" -ne 0 ]; then
  echo "RESULT: FAIL ($violations violation(s) above)"
  exit 1
fi
echo "RESULT: PASS"
