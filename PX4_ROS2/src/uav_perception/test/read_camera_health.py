#!/usr/bin/env python3
"""Print what camera_health_node is reporting.

Uses an rclpy subscription rather than `ros2 topic echo --once`, which returns
nothing in this environment often enough to be useless as evidence - it already
made a working node look dead once.
"""

import argparse
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node

LEVELS = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}


def level_name(level):
    """DiagnosticStatus.level arrives as a single byte, not an int."""
    if isinstance(level, (bytes, bytearray)):
        level = level[0] if level else 0
    return LEVELS.get(level, str(level))


class HealthReader(Node):
    def __init__(self, uav_id):
        super().__init__('read_camera_health')
        prefix = '/uav/' + uav_id
        self.reports = []
        self.summaries = []
        self.create_subscription(
            DiagnosticArray, prefix + '/diagnostics/perception', self.reports.append, 10)
        self.create_subscription(
            DiagnosticStatus, prefix + '/state/camera_health', self.summaries.append, 10)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--seconds', type=float, default=8.0)
    args = parser.parse_args()

    rclpy.init()
    node = HealthReader(args.uav_id)
    end = time.time() + args.seconds
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not node.reports:
        print('  no diagnostics received')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    latest = node.reports[-1]
    for status in latest.status:
        detail = ' '.join('%s=%s' % (kv.key, kv.value) for kv in status.values)
        print('  %-22s %-5s %-38s %s'
              % (status.name, level_name(status.level), status.message, detail))

    if node.summaries:
        summary = node.summaries[-1]
        print('  SUMMARY                %-5s %s'
              % (level_name(summary.level), summary.message))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
