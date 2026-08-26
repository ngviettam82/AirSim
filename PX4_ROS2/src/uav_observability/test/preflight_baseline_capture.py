#!/usr/bin/env python3
"""P10.9b D0: captures every NON-OK Sub-B child DiagnosticStatus seen on a
parked, healthy stack over `--duration` seconds. Produces a raw JSONL log --
one line PER OBSERVATION (not deduplicated; preflight_baseline_report.py
reduces it to a candidate waiver table). This is the ONLY input the project
owner reviews before any waiver_* row is loaded into
config/preflight_waivers.yaml (docs/interface-contract-v0.1.md S:2.20,
.claude/plan/P10-gonogo-design-panel.md S:4 "operator-checklist").

Sub-B ONLY: the waiver mechanism (design panel S:4.a's 7-rule table) only
ever applies to Sub-B per-child content -- Sub-A liveness always gates and
is never waivable, so there is nothing for a human to review there. Reads
the SAME diag_source_names/diag_source_topics straight off
observability_params.yaml so this script can never drift from what
diagnostics_node actually watches (its own source of truth, not a copy).

Usage:
  preflight_baseline_capture.py --uav-id uav0 --duration 60 \\
    --params-yaml <path>/observability_params.yaml --output FILE.jsonl
"""
import argparse
import json
import time

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_LEVEL_NAME = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}


def load_diag_sources(params_yaml_path, uav_id):
    with open(params_yaml_path) as f:
        doc = yaml.safe_load(f)
    params = doc['diagnostics_node']['ros__parameters']
    names = params['diag_source_names']
    topics = params['diag_source_topics']
    if len(names) != len(topics):
        raise ValueError('diag_source_names/diag_source_topics length mismatch in %s' %
                          params_yaml_path)
    return list(zip(names, ['/uav/%s/%s' % (uav_id, t) for t in topics]))


def kv(status, key):
    for pair in status.values:
        if pair.key == key:
            return pair.value
    return None


class BaselineCapture(Node):

    def __init__(self, sources, output_path):
        super().__init__('preflight_baseline_capture')
        self._out = open(output_path, 'a', buffering=1)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self._subs = []
        for source_name, topic in sources:
            sub = self.create_subscription(
                DiagnosticArray, topic,
                (lambda msg, name=source_name: self._on_msg(name, msg)), qos)
            self._subs.append(sub)
        self._non_ok_count = 0

    def _on_msg(self, source_name, msg):
        for status in msg.status:
            # diagnostic_msgs/DiagnosticStatus.level is a ROS `byte`, which
            # rclpy exposes as a 1-byte `bytes` object, NOT a plain int --
            # `status.level == 0` is ALWAYS False against bytes (compares
            # unequal types), so this filter silently captured EVERYTHING
            # (including genuine OK) until this int() conversion was added
            # (caught by inspecting the first real D0 run's own output).
            level_int = status.level[0] if isinstance(status.level, (bytes, bytearray)) \
                else int(status.level)
            if level_int == 0:   # DiagnosticStatus.OK
                continue
            record = {
                't_wall': time.time(),
                'source': source_name,
                'child': status.name,
                'level': _LEVEL_NAME.get(level_int, str(level_int)),
                'action': kv(status, 'action'),
                'message': status.message,
            }
            self._out.write(json.dumps(record) + '\n')
            self._out.flush()
            self._non_ok_count += 1

    def non_ok_count(self):
        return self._non_ok_count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--params-yaml', required=True)
    args = parser.parse_args()

    sources = load_diag_sources(args.params_yaml, args.uav_id)

    rclpy.init()
    node = BaselineCapture(sources, args.output)
    deadline = time.time() + args.duration
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        print('SOURCES_WATCHED=%d' % len(sources))
        for name, topic in sources:
            print('  %s -> %s' % (name, topic))
        print('NON_OK_RECORDS=%d' % node.non_ok_count())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
