#!/usr/bin/env python3
"""G4: with GNSS drifting, the stack must NOT silently follow bad GPS.

The claim (P4-localization.md S:83): injecting Gauss-Markov drift (tau=60 s,
sigma=5 m) into the GPS adapter must be CAUGHT -- localization_health_node
reports it as source disagreement -- and odometry_fused must stay on VIO.

Scope, stated so nobody reads more into a PASS than it earned: this measures
the DETECTION path, parked. It does not fly. GPS drift accumulates and the
health cross-check runs identically whether or not the aircraft is airborne,
and the mux selects (never blends), so flight adds no term to this claim --
the flying case is already covered by the indoor/outdoor M5 regressions.

Separating stimulus from response (the spec's own words): the stimulus is
configured at launch (drift.enabled/seed, sim-only params) and is NEVER read
back from the thing being measured; the response is read only from
/diagnostics/localization and /state/estimator_source.

R27-1 preconditions -- any of these is FAILED TO MEASURE (exit 2), never PASS:
  * the key <a>_vs_<b>_m never appears (nothing was cross-checked at all)
  * /state/estimator_source never arrives
  * with drift ON, the drift never grew past the disagreement threshold: the
    STIMULUS did not happen, so there is no verdict to give. A Gauss-Markov
    path is stochastic; a small realisation is not a product failure.

R1/R20/R25: only uav_interfaces + diagnostic_msgs + std_msgs, never px4_msgs.
Shares the sim domain on purpose (must see the real stack).

Usage: python3 g4_gps_disagreement_gate.py --uav-id uav0 --window-sec 120
       [--expect-detection | --expect-quiet]
Exit 0 = the expectation held with evidence; 1 = it did not; 2 = FAILED TO MEASURE.
"""
import argparse
import re
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# localization_health_node names the pair <a>_vs_<b>_m, keeping each source's
# own name; do not hard-code "gps_vs_vio_m" -- the names come from config.
PAIR_KEY = re.compile(r'^(?P<a>[a-z_]+)_vs_(?P<b>[a-z_]+)_m$')

OK, WARN, ERROR = 0, 1, 2


def level_int(level):
    """DiagnosticStatus.level is ROS `byte`; rclpy hands back bytes, not int.

    Comparing that to 0 is always False and silently classifies everything --
    the exact bug D0 of P10.9b hit (ops-playbook S:7).
    """
    if isinstance(level, (bytes, bytearray)):
        return level[0]
    return int(level)


class Watcher(Node):
    def __init__(self, uav_id):
        super().__init__('g4_gps_disagreement_gate')
        prefix = '/uav/%s' % uav_id
        self.worst_level = OK
        self.max_gap_m = None
        self.pair_seen = None
        self.samples = 0
        self.sources = set()
        self.last_source = None
        self.create_subscription(
            DiagnosticArray, prefix + '/diagnostics/localization', self._on_diag, 10)
        # estimator_source is latched AND on-change: the mux announces once when
        # it picks a source and then stays quiet. A VOLATILE subscriber that joins
        # after that announcement sees NOTHING FOR THE WHOLE RUN -- which is
        # exactly how this gate first reported "estimator_source never arrived"
        # while the mux was healthy and had selected vio. Match the publisher's
        # durability or the topic is invisible (same family as the /state/
        # system_health trap, contract 2.20).
        latched = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            String, prefix + '/state/estimator_source', self._on_source, latched)

    def _on_diag(self, message):
        for status in message.status:
            for pair in status.values:
                found = PAIR_KEY.match(pair.key)
                if not found:
                    continue
                try:
                    gap = float(pair.value)
                except ValueError:
                    continue
                self.pair_seen = pair.key
                self.samples += 1
                if self.max_gap_m is None or gap > self.max_gap_m:
                    self.max_gap_m = gap
                self.worst_level = max(self.worst_level, level_int(status.level))

    def _on_source(self, message):
        # TRANSIENT_LOCAL replays retained samples, so the startup 'none' arrives
        # too. Keep the LAST value as the verdict and the set as the history --
        # 'none then vio' is healthy, 'none' alone is not.
        value = message.data.strip()
        self.sources.add(value)
        self.last_source = value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--window-sec', type=float, default=120.0)
    parser.add_argument('--min-gap-m', type=float, default=1.0,
                        help='below this the injected drift never became a stimulus')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--expect-detection', action='store_true')
    group.add_argument('--expect-quiet', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = Watcher(args.uav_id)
    deadline = time.time() + args.window_sec
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)

    pair = node.pair_seen
    gap = node.max_gap_m
    worst = node.worst_level
    sources = sorted(node.sources)
    node_last_source = node.last_source
    print('  pair key          : %s' % (pair or 'NEVER SEEN'))
    print('  cross-check samples: %d' % node.samples)
    print('  max disagreement  : %s m' % ('%.3f' % gap if gap is not None else 'n/a'))
    print('  worst level       : %d (0=OK 1=WARN 2=ERROR)' % worst)
    print('  estimator_source  : %s' % (', '.join(sources) or 'NEVER SEEN'))

    node.destroy_node()
    rclpy.shutdown()

    if pair is None or node.samples == 0:
        print('  FAILED TO MEASURE: no source pair was ever cross-checked')
        return 2
    if not sources:
        print('  FAILED TO MEASURE: estimator_source never arrived')
        return 2

    if args.expect_detection:
        if gap is None or gap < args.min_gap_m:
            print('  FAILED TO MEASURE: injected drift only reached %.3f m (< %.3f) -- '
                  'the stimulus did not happen, so there is no verdict'
                  % (gap or 0.0, args.min_gap_m))
            return 2
        if worst < WARN:
            print('  FAIL: drift reached %.3f m and health still reported OK -- '
                  'this is the "silently follows bad GPS" failure' % gap)
            return 1
        if node_last_source != 'vio':
            print('  FAIL: estimator_source settled on %r, expected vio -- '
                  'the mux followed a source it had been told disagreed'
                  % node_last_source)
            return 1
        print('  PASS: drift %.3f m was caught at level %d, source settled on %r '
              '(history %s)' % (gap, worst, node_last_source, sources))
        return 0

    # --expect-quiet: the R27-3 control. Without this, "detected!" proves nothing,
    # because a monitor that always alarms is indistinguishable from one that works.
    if worst >= WARN:
        print('  FAIL: control run with drift OFF still escalated to level %d '
              '(max gap %.3f m) -- the gate cannot tell a fault from quiet' % (worst, gap))
        return 1
    print('  PASS (control): drift off stayed OK, max gap %.3f m' % gap)
    return 0


if __name__ == '__main__':
    sys.exit(main())
