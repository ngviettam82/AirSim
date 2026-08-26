#!/usr/bin/env python3
"""P10.9a G-O4: continuous /state/system_health sampler for verify_observability.sh's
o4-gate round. Subscribes Reliable+TransientLocal(depth1) -- matching diagnostics_
node.cpp's real publisher QoS exactly, so a late-joining subscriber still gets the
latched last sample immediately (same reasoning as o3_replay_gate.py's live mode).
Appends one JSON line PER REAL PUBLISH EVENT (R21: anchored on the actual message
arrival, not a fixed poll interval) so downstream analysis (o4_report.py) can measure
go/no-go transition latency directly off the wire.
Usage: o4_system_health_sampler.py --uav-id uav0 --duration 40 --output FILE
"""
import argparse
import json
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def kv(status, key):
    for pair in status.values:
        if pair.key == key:
            return pair.value
    return None


class SystemHealthSampler(Node):

    def __init__(self, uav_id, output_path):
        super().__init__('o4_system_health_sampler')
        self._out = open(output_path, 'a', buffering=1)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.history = HistoryPolicy.KEEP_LAST
        self._sub = self.create_subscription(
            DiagnosticStatus, '/uav/%s/state/system_health' % uav_id, self._on_msg, qos)
        self._count = 0

    def _on_msg(self, msg):
        record = {
            't_wall': time.time(),
            'go_no_go': kv(msg, 'go_no_go'),
            'unknown_count': kv(msg, 'unknown_count'),
            'error_count': kv(msg, 'error_count'),
            'warn_count': kv(msg, 'warn_count'),
            'stale_count': kv(msg, 'stale_count'),
            'worst_item': kv(msg, 'worst_item'),
            'clock_regressions': kv(msg, 'clock_regressions'),
            'gate_mode': kv(msg, 'gate_mode'),
            # P10.9b: gate_mode is now MEASURED (docs/interface-contract-
            # v0.1.md S:2.20) -- these 3 KeyValues did not exist before.
            'gate_mode_source': kv(msg, 'gate_mode_source'),
            'waived_count': kv(msg, 'waived_count'),
            'waiver_unmatched': kv(msg, 'waiver_unmatched'),
            'blocking': kv(msg, 'blocking'),
            'watch_failed': kv(msg, 'watch_failed'),
            'stamp_sec': kv(msg, 'stamp_sec'),
            'wall_stamp_sec': kv(msg, 'wall_stamp_sec'),
        }
        self._out.write(json.dumps(record) + '\n')
        self._out.flush()
        self._count += 1

    def sample_count(self):
        return self._count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    rclpy.init()
    node = SystemHealthSampler(args.uav_id, args.output)
    deadline = time.time() + args.duration
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        print('SAMPLES=%d' % node.sample_count())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
