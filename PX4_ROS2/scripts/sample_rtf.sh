#!/bin/bash
# Sample RTF of a running world; reports mean, not median.
# See docs/ops-playbook.md S2 for why mean, not median.
#
# TWO NUMBERS ON PURPOSE (2026-08-26). The `real_time_factor` field is what Gazebo
# publishes, and P12.5 measured it disagreeing with the message it arrives in: one
# sample reported 0.151 while sim_time/real_time OF THAT SAME MESSAGE gave 0.748. It
# is a one-step ratio, so its arithmetic mean is not the ratio over the window -- and
# "did the simulation keep up over this flight" is a question about the window.
#
# The first line is unchanged and is still what the gates parse: nothing here moves a
# bar. The second line is the same window measured from the two clocks the message
# carries, which is the CLAUDE.md section 5 rule applied to this gate: read the thing
# itself, not a convenience field that describes it.
#
# Usage: sample_rtf.sh [world] [samples]
set -o pipefail

WORLD=${1:-uav_arena}
SAMPLES=${2:-120}
RAW=$(mktemp)
trap 'rm -f "$RAW"' EXIT

timeout 200 gz topic -e -t "/world/$WORLD/stats" -n "$SAMPLES" > "$RAW" 2>/dev/null

grep real_time_factor "$RAW" | awk '{print $2}' | sort -g \
  | awk '{v[NR]=$1; s+=$1}
         END {if (!NR) {print "no data - is the world running?"; exit 1}
              printf "RTF  min %.3f | p50 %.3f | max %.3f | mean %.3f  (n=%d)\n",
                     v[1], v[int((NR+1)/2)], v[NR], s/NR, NR}' || exit 1

# sim_time { sec nsec } and real_time { sec nsec } arrive as named blocks, so the
# parser has to know which block it is inside; a bare grep for `sec:` reads both.
awk '
  /^sim_time \{/       { blk = "sim";  sec = ""; next }
  /^real_time \{/      { blk = "real"; sec = ""; next }
  /^[a-z_]+ \{/        { blk = "";     next }
  /^\}/                { blk = "";     next }
  blk != "" && /sec:/ && !/nsec:/ { sec = $2; next }
  blk != "" && /nsec:/ {
    t = (sec == "" ? 0 : sec) + $2 / 1e9
    if (blk == "sim")  { if (n_sim++ == 0) sim0 = t; sim1 = t }
    if (blk == "real") { if (n_real++ == 0) real0 = t; real1 = t }
    blk = ""
    next
  }
  END {
    if (n_sim < 2 || n_real < 2) { print "  window RTF: not enough clock samples"; exit 0 }
    ds = sim1 - sim0; dr = real1 - real0
    if (dr <= 0) { print "  window RTF: real clock did not advance"; exit 0 }
    printf "  window RTF %.3f  (sim %.2f s over real %.2f s, from the same messages)\n",
           ds / dr, ds, dr
  }
' "$RAW"
