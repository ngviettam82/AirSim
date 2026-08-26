#!/usr/bin/env python3
"""G-O1 (P10.3) fixed-rate std_msgs/String publisher, used by
scripts/verify_observability.sh to feed rosbag_manager_node synthetic
traffic on ROS_DOMAIN_ID 99. Runs up to `duration_sec` wall-clock seconds
OR until SIGTERM/SIGINT, whichever first, then prints the exact count it
sent as `PUBLISHED=<n>` -- the caller uses this as the ground truth for
message-loss, not an estimate from rate*duration.

Usage: go1_publisher.py <topic> <rate_hz> <duration_sec>
"""
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

_stop = False


def _request_stop(_signum, _frame):
    global _stop
    _stop = True


def main() -> None:
    topic, rate_hz, duration_sec = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    # rclpy.init()'s OWN default SIGTERM/SIGINT handler would otherwise race
    # our _stop flag and shut the context down before we reach the final
    # rclpy.shutdown() below -- NO disables it so only this handler decides.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _request_stop)
    signal.signal(signal.SIGINT, _request_stop)

    node = Node('go1_publisher_' + topic.strip('/').replace('/', '_'))
    qos = QoSProfile(
        depth=20, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    publisher = node.create_publisher(String, topic, qos)

    period = 1.0 / rate_hz
    deadline = time.monotonic() + duration_sec
    count = 0
    next_tick = time.monotonic()
    # time.sleep() auto-retries on the signal (PEP 475) since the handler
    # above never raises -- worst-case response lag to stop is one period.
    while not _stop and time.monotonic() < deadline:
        msg = String()
        msg.data = f'{topic}:{count}'
        publisher.publish(msg)
        count += 1
        next_tick += period
        sleep_for = next_tick - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)

    print(f'PUBLISHED={count}', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
