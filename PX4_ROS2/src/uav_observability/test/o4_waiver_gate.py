#!/usr/bin/env python3
"""P10.9b D4/(f): asserts a REAL preflight GO where the waiver exemption
path actually ran (R27-1: waived_count>0 is not optional here -- 0 means
the whole mechanism never fired, that is FAILED TO MEASURE, not a pass),
and every waived child is inside a project-owner-signed allow-list (design
panel G6, .claude/plan/P10-gonogo-design-panel.md S:3): a script that could
silently accept "one more waiver row" is exactly the organisational failure
mode (F3) waivers are the riskiest part of this whole design.

/state/system_health only carries waived_count (a NUMBER) -- it does NOT
say WHICH children were waived, so this also reads /diagnostics/aggregated
(the same eval tick's per-child `waived`/`source`/`child` KeyValues) to
recover that set.

Usage:
  o4_waiver_gate.py --uav-id uav0 --timeout 30 \\
    --allowed-waived "diagnostics/safety:safety: OFFBOARD_UNHEALTHY"
Exit: 0=PASS 1=FAIL(over allow-list) 2=FAILED TO MEASURE
"""
import argparse
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def kv(status, key):
    for pair in status.values:
        if pair.key == key:
            return pair.value
    return None


class WaiverGateProbe(Node):

    def __init__(self, uav_id):
        super().__init__('o4_waiver_gate_probe')
        health_qos = QoSProfile(depth=1)
        health_qos.reliability = ReliabilityPolicy.RELIABLE
        health_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        health_qos.history = HistoryPolicy.KEEP_LAST
        self.last_health = None
        self.create_subscription(
            DiagnosticStatus, '/uav/%s/state/system_health' % uav_id, self._on_health, health_qos)
        self.last_aggregated = None
        self.create_subscription(
            DiagnosticArray, '/uav/%s/diagnostics/aggregated' % uav_id, self._on_aggregated,
            QoSProfile(depth=10))

    def _on_health(self, msg):
        self.last_health = msg

    def _on_aggregated(self, msg):
        self.last_aggregated = msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument(
        '--allowed-waived', default='',
        help='comma-separated "<source>:<child>" allow-list (G6) -- '
             'e.g. "diagnostics/safety:safety: OFFBOARD_UNHEALTHY"')
    args = parser.parse_args()
    allowed = set(s for s in args.allowed_waived.split(',') if s)

    rclpy.init()
    node = WaiverGateProbe(args.uav_id)
    deadline = time.time() + args.timeout
    ok = False
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        h = node.last_health
        if (h is not None and kv(h, 'go_no_go') == 'GO' and kv(h, 'gate_mode') == 'preflight' and
                kv(h, 'gate_mode_source') == 'measured'):
            ok = True
            break

    if not ok:
        h = node.last_health
        print('FAILED TO MEASURE: never reached go_no_go=GO/gate_mode=preflight/measured '
              'within %.1fs -- last health: go_no_go=%s gate_mode=%s gate_mode_source=%s' %
              (args.timeout, kv(h, 'go_no_go') if h else None, kv(h, 'gate_mode') if h else None,
               kv(h, 'gate_mode_source') if h else None))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    h = node.last_health
    try:
        waived_count = float(kv(h, 'waived_count'))
    except (TypeError, ValueError):
        waived_count = 0.0
    print(
        '[health] go_no_go=%s gate_mode=%s(%s) waived_count=%s waiver_unmatched=%s '
        'worst_item=%r blocking=%r' %
        (kv(h, 'go_no_go'), kv(h, 'gate_mode'), kv(h, 'gate_mode_source'), kv(h, 'waived_count'),
         kv(h, 'waiver_unmatched'), kv(h, 'worst_item'), kv(h, 'blocking')))

    if waived_count <= 0:
        print(
            'FAILED TO MEASURE: waived_count=%s -- the waiver exemption path never actually ran '
            '(R27-1: reaching GO some OTHER way does not prove the waiver mechanism itself works)'
            % waived_count)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    # /diagnostics/aggregated publishes on the SAME publish_period_sec as
    # system_health, but discovery/subscription setup for the two topics
    # can race independently -- health reaching GO first (possibly via an
    # on-change publish, ahead of the next periodic aggregated tick) does
    # NOT guarantee an aggregated sample has arrived yet. Give it its own
    # short grace window instead of failing on the very first check.
    agg_deadline = time.time() + 5.0
    while rclpy.ok() and node.last_aggregated is None and time.time() < agg_deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    agg = node.last_aggregated
    if agg is None:
        print('FAILED TO MEASURE: no /diagnostics/aggregated sample received within 5s of '
              'reaching GO -- cannot recover WHICH children were waived to check the allow-list '
              '(G6)')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    waived_keys = set()
    for status in agg.status:
        if kv(status, 'waived') == 'true':
            waived_keys.add('%s:%s' % (kv(status, 'source'), kv(status, 'child')))

    print('[aggregated] waived children this tick: %s' % sorted(waived_keys))
    over_allowed = waived_keys - allowed
    node.destroy_node()
    rclpy.shutdown()

    if over_allowed:
        print(
            'FAIL: waived child(ren) NOT in --allowed-waived (G6 -- nới lỏng ngoài chữ ký '
            'chủ dự án): %s' % sorted(over_allowed))
        sys.exit(1)

    print(
        'PASS: GO + preflight + measured + waived_count=%s + every waived child is inside the '
        'signed allow-list' % waived_count)
    sys.exit(0)


if __name__ == '__main__':
    main()
