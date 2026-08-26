#!/bin/bash
# Positive control for the safety-code explanation table in smoke_flight.py (2026-08-26).
#
# WHY THIS EXISTS. On 2026-08-26 CAMERA_STREAM_UNHEALTHY was traced to one cause and
# written down, which is what M5 asks for. But a code has many possible causes -- a dead
# bridge and an empty sky raise the same one -- so an explanation that covers all of them
# turns the shield into an amnesty. The guard accepts the code only while the detailed
# diagnostics report the reason that was actually traced; these cases pin that both ways,
# including the case the project keeps paying for: no detail at all must NOT read as
# "the benign cause applies" (O3, not measured is not OK).
#
# Runs the shipped functions out of smoke_flight.py, not a copy of them.
#
# Usage: bash scripts/selftest_smoke_flight_codes.sh
set -o pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
python3 "$HERE/smoke_flight_codes_selftest.py"
