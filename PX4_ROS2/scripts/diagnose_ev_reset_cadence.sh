#!/bin/bash
# P11.0 step 0a -- is the "EKF2 restarts all EV aiding once per second" debt REAL, or is
# it an artefact of how estimator_event_flags is published?
#
# THE DEBT AS RECORDED (package-status, "Nợ mở: EKF2 khởi động lại toàn bộ EV aiding
# đúng 1 lần/giây"): starting_vision_{pos,vel,yaw}_fusion + reset_{pos,vel}_to_vision +
# reset_hgt_to_ev all fire 71 times in 70 s at a period of exactly 1.004 s. Read as a
# product fault, that means the estimator's position/velocity/height states and their
# covariances are re-initialised at 1 Hz, and in flight the velocity is yanked to the EV
# measurement every second. With real VIO that is dangerous. It is why P11 opens here.
#
# WHY IT MIGHT NOT BE REAL. EKF2.cpp:1129 (v1.15.4):
#
#     } else if ((_last_event_flags_publish != 0) && (timestamp >= _last_event_flags_publish + 1_s)) {
#             // continue publishing periodically
#             _estimator_event_flags_pub.update();
#     }
#
# When NO new event occurs, EKF2 RE-PUBLISHES THE PREVIOUS MESSAGE UNCHANGED, once per
# second, forever -- every latched bit still set, only the timestamp moved. So one real
# event at startup produces an unbroken 1 Hz train of messages that all say
# "reset_pos_to_vision = true". Counting those messages counts republications, not resets.
# The 1.004 s period is the giveaway: that is the `+ 1_s` republish cadence with
# scheduling jitter, not a property of external vision.
#
# The debt record already contains the contradiction: it also states that in steady
# state cs_ev_{pos,hgt,vel,yaw} = 100%. If EV fusion really stopped and restarted every
# second, those control-status flags COULD NOT be 100% -- and they are state, sampled,
# not latched events, so they are the trustworthy half.
#
# THE DISCRIMINATOR. estimator_event_flags carries `information_event_changes`, a
# COUNTER that EKF2 increments only when it actually has an event to report
# (EKF2.cpp:1072). A republished message carries the OLD counter value.
#
#     counter constant across N messages that all say reset_pos_to_vision=true
#         => N-1 of them are republications. ONE real event. Debt is a measurement artefact.
#     counter increments once per message
#         => N real events. Debt is real and P11.0 proceeds to the replay step.
#
# This is the same shape as the peakAcceleration case in CLAUDE.md S:5: a derived field
# that describes a quantity is not the quantity. Here the latched bit describes an event;
# the counter is the event.
#
# Usage: bash scripts/diagnose_ev_reset_cadence.sh [path/to/log.ulg]
#        with no argument it picks the newest ulog that actually contains EV data.
set -o pipefail

WORKSPACE=$HOME/PX4_ROS2
LOGROOT=$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs/log
OUT=$WORKSPACE/gate_logs/p11_0
mkdir -p "$OUT"

ULG=$1
if [ -z "$ULG" ]; then
  echo "=== picking the newest ulog that contains external-vision aiding ==="
  # Newest first; take the first one whose message list has the EV aid source. A log
  # from a run with no vision stack would answer a different question entirely.
  while read -r cand; do
    if ulog_info "$cand" 2>/dev/null | grep -q 'estimator_event_flags'; then
      ULG=$cand
      break
    fi
  done < <(find "$LOGROOT" -name '*.ulg' -printf '%T@ %p\n' 2>/dev/null | sort -rn | cut -d' ' -f2-)
fi

[ -n "$ULG" ] && [ -f "$ULG" ] || { echo "FATAL: no usable ulog found under $LOGROOT"; exit 2; }
echo "log: $ULG"
echo "size: $(du -h "$ULG" | cut -f1)"

python3 "$WORKSPACE/scripts/ev_reset_cadence_report.py" "$ULG" | tee "$OUT/report.txt"
rc=${PIPESTATUS[0]}
echo
echo "(evidence: $OUT)"
exit "$rc"
