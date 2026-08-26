#!/bin/bash
# One-off: trim R16-violating comments in uav_bringup/test after the agent sweep.
# Rationale already moved to src/uav_bringup/README.md.
set -o pipefail

cd /mnt/c/code/PX4_ROS2/src/uav_bringup/test || exit 1

python3 - <<'PY'
import re

REPLACEMENTS = {
    'smoke_flight.py': [
        ("""        # The setpoint stream has to be >= 2 Hz in the clock PX4 uses, so this
        # node has to share that clock with the backend it is exercising.
""",
         "        # Must share the backend's clock; see README.\n"),
        ("""        # Forget the previous flight's status, or a stale ACTIVE from the last
        # session satisfies the wait below and arming happens while PX4 has
        # already fallen back to its boot mode.
""",
         "        # Stale ACTIVE would satisfy the wait below; see README.\n"),
        ("""        # Offboard first, then arm. The boot mode is LOITER, which demands a
        # global position; with no GPS that never comes and arming in it is
        # refused. Offboard needs only the local position vision provides.
""",
         "        # Offboard first, then arm; see README.\n"),
        ("""        # Shuttle between two fixed points instead of walking further east on
        # every flight: three flights in a row would otherwise end up 9 m from
        # the start, which is through the wall of the indoor arena.
""",
         "        # Shuttle, not walk east: 3 flights would exit the room.\n"),
        ("""        # Order matters: change mode first, stop streaming after. Stopping first
        # would starve offboard and trip a failsafe.
""",
         "        # Mode first, stop streaming after; reverse trips failsafe.\n"),
        ("""        # Hold on the spot so the gateway has something to stream while priming.
""",
         "        # Gives the gateway something to stream while priming.\n"),
    ],
    'g2_fused_accuracy.py': [
        ("""# A square with a height change on the far side: every axis gets excited, and
# the legs are timed rather than arrival-gated so the window is always 60 s.
""",
         "# Timed legs keep the window at 60 s; see README.\n"),
        ("""        # The mux labels its own output SOURCE_FUSED, so the status topic does not
        # say which input it actually selected. Without this the error cannot be
        # attributed to a source.
""",
         "        # Status says FUSED, not which input won; see README.\n"),
        ("""        # Not gated on `recording`: this topic is transient_local, so the current
        # value is delivered once at subscription time, long before recording
        # starts. Gating it drops the only message most flights ever see.
""",
         "        # Ungated: transient_local delivers once, before recording.\n"),
        ("""        # Offboard before arm: the boot mode needs a global position that indoor
        # flights never get, and offboard only needs the local one.
""",
         "        # Offboard before arm; see README.\n"),
    ],
}

for name, pairs in REPLACEMENTS.items():
    text = open(name, encoding='utf-8').read()
    for old, new in pairs:
        if old not in text:
            print('  MISS %s: %r' % (name, old.strip().splitlines()[0][:60]))
            continue
        text = text.replace(old, new)
    text = re.sub(r'^[ \t]*# -{5,}.*\n', '', text, flags=re.MULTILINE)
    open(name, 'w', encoding='utf-8', newline='\n').write(text)
    print('  wrote %s' % name)
PY

for file in smoke_flight.py g2_fused_accuracy.py; do
  python3 -m py_compile "$file" && printf '  %-24s py_compile OK  %d comment lines\n' \
    "$file" "$(grep -c '^[[:space:]]*#' "$file")"
done
