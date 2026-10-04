#!/usr/bin/env python3
"""P10.9b: reads /uav/<id>/state/system_health ONCE, verifying its OWN age
via the in-payload stamp (R32) before trusting it -- NEVER use `ros2 topic
echo --once` for this. /state/system_health is Reliable/KeepLast(1)/
TransientLocal: a dead diagnostics_node's last-known-GO sample stays
latched forever with nothing EXTERNAL saying the node itself is gone.
DiagnosticStatus has no header, so wall_stamp_sec (real UTC epoch seconds,
carried as a payload KeyValue) is the only thing that can catch a frozen
node -- docs/interface-contract-v0.1.md S:2.20.

Prints exactly one line, exit code matches verify_*.sh's convention:
  0 = GO
  1 = NO_GO
  2 = UNKNOWN / STALE (age check failed) / FAILED TO MEASURE (no sample at all)
"""
import argparse
import os
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from gate_freshness import is_fresh  # N8: shared with o4_report.py, see that file

# V2 (P10 review lượt 3): the shipped yaml -- used ONLY to DERIVE --max-age-
# sec's default (2x publish_period_sec) instead of a hand-typed literal that
# can silently drift from it (R34 spirit). A real bringup overriding
# publish_period_sec at launch time still works correctly: pass --max-age-sec
# explicitly (documented on that argument).
_DEFAULT_PARAMS_YAML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, 'src', 'uav_observability', 'config',
    'observability_params.yaml')


def derive_default_max_age_sec(params_yaml_path):
    """2x publish_period_sec read straight off params_yaml_path. Falls back
    to the old hand-typed 2.0s -- WARNing on stderr -- when that yaml is not
    reachable/parseable (missing PyYAML, missing file, unexpected shape)."""
    try:
        import yaml
    except ImportError:
        print('WARN: PyYAML not available, cannot derive --max-age-sec default from %s -- '
              'falling back to 2.0s (verify it matches the deployed publish_period_sec)' %
              params_yaml_path, file=sys.stderr)
        return 2.0
    try:
        with open(params_yaml_path) as f:
            doc = yaml.safe_load(f)
        publish_period_sec = float(doc['diagnostics_node']['ros__parameters']['publish_period_sec'])
        return 2.0 * publish_period_sec
    except (OSError, KeyError, TypeError, ValueError) as error:
        print('WARN: could not derive --max-age-sec default from %s (%s) -- falling back to '
              '2.0s (verify it matches the deployed publish_period_sec)' %
              (params_yaml_path, error), file=sys.stderr)
        return 2.0


def kv(status, key):
    for pair in status.values:
        if pair.key == key:
            return pair.value
    return None


class OnceSubscriber(Node):

    def __init__(self, uav_id):
        super().__init__('preflight_light')
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.history = HistoryPolicy.KEEP_LAST
        self.msg = None
        self.create_subscription(
            DiagnosticStatus, '/uav/%s/state/system_health' % uav_id, self._on_msg, qos)

    def _on_msg(self, msg):
        self.msg = msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--timeout', type=float, default=3.0,
                         help='how long to wait for a sample to arrive at all')
    parser.add_argument(
        '--max-age-sec', type=float, default=None,
        help='reject the sample if wall-clock age exceeds this (R32). Default: 2x '
             '--params-yaml\'s publish_period_sec (V2, derived -- falls back to 2.0s, WARNing, '
             'if that cannot be read) -- pass this explicitly if a bringup overrides '
             'publish_period_sec at launch time')
    parser.add_argument(
        '--params-yaml', default=_DEFAULT_PARAMS_YAML,
        help='observability_params.yaml used ONLY to derive --max-age-sec\'s default')
    args = parser.parse_args()
    if args.max_age_sec is None:
        args.max_age_sec = derive_default_max_age_sec(args.params_yaml)

    rclpy.init()
    node = OnceSubscriber(args.uav_id)
    deadline = time.time() + args.timeout
    while rclpy.ok() and node.msg is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.msg is None:
        print('FAILED TO MEASURE: no /uav/%s/state/system_health sample received within %.1fs '
              '(diagnostics_node not running, or never published a single sample yet)' %
              (args.uav_id, args.timeout))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    msg = node.msg
    go_no_go = kv(msg, 'go_no_go')
    worst_item = kv(msg, 'worst_item')
    blocking = kv(msg, 'blocking')
    gate_mode = kv(msg, 'gate_mode')
    gate_mode_source = kv(msg, 'gate_mode_source')
    wall_stamp_sec = kv(msg, 'wall_stamp_sec')

    try:
        age_sec = time.time() - float(wall_stamp_sec)
    except (TypeError, ValueError):
        age_sec = float('inf')

    node.destroy_node()
    rclpy.shutdown()

    if not is_fresh(age_sec, args.max_age_sec):
        # N8 (P10-gate-debt review round 2): is_fresh() rejects age_sec < 0
        # too, not just age_sec > max_age_sec -- a one-sided check let a
        # companion clock ahead of the publisher's clock (NTP not synced,
        # or a backward step) read as "GO age=-12.345s" and exit 0, exactly
        # at the moment the checklist says to read this light (after ARM,
        # before takeoff, contract Sec2.20).
        if age_sec < 0:
            print('CLOCK SKEW: last sample age is %.3fs (NEGATIVE) -- this companion\'s clock is '
                  'behind the publisher\'s wall clock (NTP not synced, or stepped backward). The '
                  'go_no_go value carried in it (%r) must NOT be trusted until the clocks agree.' %
                  (age_sec, go_no_go))
        else:
            print('STALE: last sample is %.3fs old (> --max-age-sec=%.1fs) -- diagnostics_node is '
                  'likely dead. This was a LATCHED (TransientLocal) sample, NOT a live reading. '
                  'The go_no_go value carried in it (%r) must NOT be trusted.' %
                  (age_sec, args.max_age_sec, go_no_go))
        sys.exit(2)

    print(
        '%s age=%.3fs gate_mode=%s(%s) worst_item=%r blocking=%r' %
        (go_no_go, age_sec, gate_mode, gate_mode_source, worst_item, blocking))
    if go_no_go == 'GO':
        sys.exit(0)
    if go_no_go == 'NO_GO':
        sys.exit(1)
    sys.exit(2)   # UNKNOWN, or any unrecognised value -- never a silent 0/1


if __name__ == '__main__':
    main()
