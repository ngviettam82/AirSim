#!/bin/bash
# Shared helpers for scripts/verify_*.sh and scripts/run_*.sh gate scripts.
# SOURCE this file, do not execute it directly -- it defines functions only:
#   source "$WORKSPACE/scripts/gate_common.sh"
#
# History: three gate scripts (verify_boot_no_latch.sh, verify_obstacle_hold.sh,
# verify_mission.sh round m4c) had each hand-rolled the same "count publishers
# on a topic before I inject on it" check (R27-1), with slightly different
# wording and no shared home. P9-debt patch (2026-08-24): consolidated here so
# the next gate that needs it does not roll a fourth copy, and two probes that
# were publishing without checking at all (gn4b_avoidance_gate.py,
# gn5_followtrack_gate.py) got the rclpy-side twin instead (that logic lives
# in those files directly -- rclpy's count_publishers() is not a shell call).

# wait_nodes NAME [NAME ...] -- block until every node name shows up in the
# ROS graph. Moved here from verify_mission.sh unchanged.
#
# BUG FOUND 2026-08-22 (G-M3 rerun): a single deadline shared across the whole
# node list starves later nodes in a LONG list -- discovery lag on each
# earlier node eats into the budget cumulatively, so a 7-node list can fail on
# node #6 even though it demonstrably appeared moments later (confirmed by the
# debug dump printed right after the false FATAL). Each node now gets its OWN
# fresh 90s budget.
wait_nodes() {
  for node in "$@"; do
    local deadline=$((SECONDS + 90))
    until timeout 15 ros2 node list --no-daemon 2>/dev/null | grep -q "$node"; do
      if [ $SECONDS -gt $deadline ]; then
        echo "FATAL: $node never appeared in ros2 node list"
        timeout 15 ros2 node list --no-daemon 2>/dev/null
        return 1
      fi
      sleep 3
    done
  done
  echo "all requested nodes confirmed: $*"
}

# assert_single_writer TOPIC EXPECTED [DEADLINE_SEC] -- confirm TOPIC has
# exactly EXPECTED publisher(s), held STEADY across 3 samples spaced >=2s
# apart -- not a single early read (N5 fix, P10-gate-debt review round 2,
# 2026-08-24). Each sample individually retries up to DEADLINE_SEC (default
# 30) since ROS graph discovery lags the process table -- ops-playbook Sec12.
#
# N5 BUG (fixed here): the old version read ONE sample and returned the
# instant that read looked numeric, even "0". But a FRESH `ros2 topic info
# --no-daemon` process has its OWN discovery race independent of how long
# the caller has already waited -- "0 publishers" on the very first read is
# indistinguishable from "a real remote writer that this particular query
# has not discovered yet". A double-writer starting moments after that
# first read (the exact case this function exists to catch) used to read
# as a silent PASS -- proven live: with a scripted [0,1,1,1] sequence on
# TOPIC and expected=0, the old code returned 0 (PASS) off sample #1 alone.
# Two changes:
#   1. POSITIVE CONTROL: before trusting ANY reading of TOPIC, derive
#      "<uav-prefix>/state/vehicle" from TOPIC's own /uav/<id>/... prefix
#      (R2 namespace) and confirm it shows >=1 publisher first --
#      px4_state_adapter_node publishes it unconditionally in every
#      bringup this function is ever called against, so it is a topic
#      PROVEN to have a remote writer whenever the stack is really up. If
#      TOPIC has no /uav/<id>/ prefix (e.g. this file's own selftest
#      topics), the control is skipped -- nothing safe to derive. Control
#      never converges within DEADLINE_SEC -> return 2: at that point
#      discovery itself is unproven, so no reading of TOPIC means anything
#      either way (R27-1/R27-3).
#   2. HOLD ACROSS TIME: once discovery is proven, TOPIC's own count must
#      read == EXPECTED on 3 samples spaced >=2s apart, not just once -- a
#      mismatch on ANY sample fails immediately, no need to wait out the
#      rest.
#
# Three distinct outcomes, distinguished by return code (unchanged contract):
#   0  -- OK: publisher count matched EXPECTED on every sample.
#   1  -- FAIL: a stable count was read, but it does not match EXPECTED (the
#         real double-writer case: prints the topic name, count and which
#         sample caught it).
#   2  -- FAILED TO MEASURE: either the positive control never converged, or
#         `ros2 topic info` on TOPIC never returned a numeric Publisher
#         count within DEADLINE_SEC on some sample (topic unknown / daemon
#         not ready). This is NOT a pass -- callers must treat 2 the same as
#         1, just report it distinctly (matches the 0/1/2 convention already
#         used by every gate's own exit status).
#
# 🔴 This is a DETECTION net, not a prevention mechanism: it catches a
# double-writer that already exists across the sampling window, it does not
# stop one from being started later still. Fixing the root cause (e.g.
# `navigator:=false` when the caller starts its own navigator, or
# `perception:=false` when the caller publishes the world/* topic itself)
# is still mandatory -- this function only proves the fix worked.
assert_single_writer() {
  local topic="$1" expected="$2" deadline_sec="${3:-30}"
  local sample_gap_sec=2 required_samples=3

  local uav_prefix control_topic control_info control_count control_deadline
  uav_prefix=$(echo "$topic" | grep -oE '^/uav/[^/]+')
  if [ -n "$uav_prefix" ]; then
    control_topic="$uav_prefix/state/vehicle"
    control_deadline=$((SECONDS + deadline_sec))
    while :; do
      control_info=$(timeout 15 ros2 topic info "$control_topic" --no-daemon 2>&1)
      control_count=$(echo "$control_info" | grep 'Publisher count' | grep -oE '[0-9]+')
      [ -n "$control_count" ] && [ "$control_count" -ge 1 ] && break
      if [ $SECONDS -gt $control_deadline ]; then
        echo "$control_info"
        echo "FAILED TO MEASURE: positive-control topic $control_topic never showed >=1 publisher within ${deadline_sec}s -- discovery not proven to have converged, so no reading of $topic is trustworthy either way"
        return 2
      fi
      sleep 2
    done
    echo "discovery proven converged: $control_topic has $control_count publisher(s)"
  else
    echo "note: $topic has no /uav/<id>/ prefix -- skipping positive-control probe (nothing safe to derive)"
  fi

  local n info pubcount deadline
  for ((n = 1; n <= required_samples; n++)); do
    deadline=$((SECONDS + deadline_sec))
    while :; do
      info=$(timeout 15 ros2 topic info "$topic" --no-daemon 2>&1)
      pubcount=$(echo "$info" | grep 'Publisher count' | grep -oE '[0-9]+')
      [ -n "$pubcount" ] && break
      if [ $SECONDS -gt $deadline ]; then
        echo "$info"
        echo "FAILED TO MEASURE: $topic publisher count not readable via ros2 topic info within ${deadline_sec}s (sample $n/$required_samples)"
        return 2
      fi
      sleep 2
    done
    echo "$info"
    if [ "$pubcount" != "$expected" ]; then
      echo "FAIL: expected $expected publisher(s) on $topic, got $pubcount (sample $n/$required_samples)"
      return 1
    fi
    echo "confirmed: $pubcount publisher(s) on $topic (expected $expected, sample $n/$required_samples)"
    [ "$n" -lt "$required_samples" ] && sleep "$sample_gap_sec"
  done
  echo "confirmed: $expected publisher(s) on $topic held steady across $required_samples samples (>= $((sample_gap_sec * (required_samples - 1)))s apart)"
  return 0
}
