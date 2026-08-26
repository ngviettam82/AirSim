#!/usr/bin/env python3
"""P10.7 G-O2: wall-clock inter-arrival latency probe for command_selected.

R27: measures the moment a message ARRIVES at a subscriber callback (wall
clock, time.monotonic()) -- never header.stamp, which is sim time and blind
to any real scheduling/DDS delay the observability load might add.

Uses rclpy's default wait-set dispatch (rclpy.spin, not a manual spin_once
poll loop) so the callback fires as soon as the executor's OS-level wait
wakes up -- a busy poll loop would add its own polling jitter directly into
the thing being measured.

Usage: o2_latency_probe.py --uav-id uav0 --duration 90 --output /tmp/o2.json
Exit: 0 = measured with enough samples, 2 = FAILED TO MEASURE (R27-1).
"""
import argparse
import json
import signal
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from uav_interfaces.msg import ControlCommand

# Matches control_authority_manager_node's command_qos (memory S:3, P7):
# Reliable / KeepLast(10) / Volatile. Depth is bumped here on the SUBSCRIBER
# side only -- QoS compatibility only requires reliability/durability to
# match, not depth -- to give this probe slack under load before a queue
# overrun could masquerade as a latency spike that never really happened.
COMMAND_SELECTED_QOS = QoSProfile(
    depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

MIN_INTERVALS_TO_MEASURE = 20   # below this, the run itself is suspect (R27-1)


def percentile(sorted_values, p):
    if not sorted_values:
        return float('nan')
    k = (len(sorted_values) - 1) * p
    f = int(k)
    c = min(f + 1, len(sorted_values) - 1)
    if f == c:
        return sorted_values[f]
    return sorted_values[f] + (sorted_values[c] - sorted_values[f]) * (k - f)


class LatencyProbe(Node):
    def __init__(self, uav_id):
        super().__init__('o2_latency_probe')
        self.topic = '/uav/%s/control/command_selected' % uav_id
        self.arrivals = []
        self.create_subscription(ControlCommand, self.topic, self._on_msg, COMMAND_SELECTED_QOS)
        self.get_logger().info('o2_latency_probe subscribed to %s' % self.topic)

    def _on_msg(self, _msg):
        self.arrivals.append(time.monotonic())


def analyze(arrivals, gap_threshold):
    """Splits raw inter-arrivals into 'active streaming' vs 'known idle gap'.

    command_selected is forwarded EVENT-DRIVEN (control_authority_manager_node
    republishes exactly on onSourceMessage(), no republish timer) -- so any
    interval far above the nominal ~50ms (20Hz) is a real idle window (e.g.
    smoke_flight.py's inter-flight settle where target=None), not jitter.
    Excluding those is required for p99 of ACTIVE streaming to mean anything;
    both are reported so the exclusion is never silent.
    """
    if len(arrivals) < 2:
        return None
    intervals = []
    excluded = []
    for i in range(1, len(arrivals)):
        dt = arrivals[i] - arrivals[i - 1]
        if dt > gap_threshold:
            excluded.append(dt)
        else:
            intervals.append(dt)
    intervals_sorted = sorted(intervals)
    return {
        'n_messages': len(arrivals),
        'n_intervals_used': len(intervals),
        'n_gaps_excluded': len(excluded),
        'excluded_gap_seconds_total': sum(excluded),
        'excluded_gap_seconds_max': max(excluded) if excluded else 0.0,
        'p50': percentile(intervals_sorted, 0.50),
        'p95': percentile(intervals_sorted, 0.95),
        'p99': percentile(intervals_sorted, 0.99),
        'max': max(intervals) if intervals else float('nan'),
        'mean': (sum(intervals) / len(intervals)) if intervals else float('nan'),
        'window_seconds': arrivals[-1] - arrivals[0],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--duration', type=float, default=180.0,
                         help='max wall seconds to listen before auto-stop')
    parser.add_argument('--gap-threshold', type=float, default=1.0,
                         help='inter-arrival gaps above this are excluded as known idle '
                              'windows (between-flight settle / priming), not jitter')
    parser.add_argument('--output', required=True, help='JSON report path')
    args = parser.parse_args()

    # NO: rclpy's DEFAULT SignalHandlerOptions installs its own SIGINT
    # handler that tears the context down mid-wait -- spin_until_future_
    # complete then raises rclpy.executors.ExternalShutdownException
    # instead of returning, which the gate script's `kill -INT` (the
    # documented way to stop this probe early) turned into a crash BEFORE
    # the JSON report was ever written (bug caught live in the first real
    # G-O2 run: every row read p99=NaN n_used=0 despite the flight itself
    # succeeding). Installing our own handler that resolves the SAME
    # stop_future the timer uses makes SIGINT and the timer one code path,
    # no exception involved.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LatencyProbe(args.uav_id)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    stop_future = Future()

    def _request_stop(*_args):
        if not stop_future.done():
            stop_future.set_result(None)

    node.create_timer(args.duration, _request_stop)
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    executor.spin_until_future_complete(stop_future)

    stats = analyze(node.arrivals, args.gap_threshold)
    report = {
        'uav_id': args.uav_id,
        'topic': node.topic,
        'gap_threshold_sec': args.gap_threshold,
        'stats': stats,
    }
    with open(args.output, 'w') as f:
        json.dump(report, f, indent=2)

    # SIGINT may have already made rclpy tear the context down (rclpy.init's
    # own default handler) -- the JSON above is already safely on disk by
    # this point, so a double-shutdown here must never swallow the report.
    try:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    except Exception as exc:  # noqa: BLE001 -- best-effort cleanup only
        print('cleanup warning (report already written): %r' % exc)

    if stats is None or stats['n_intervals_used'] < MIN_INTERVALS_TO_MEASURE:
        got = 0 if stats is None else stats['n_intervals_used']
        print('SUMMARY FAILED_TO_MEASURE n_messages=%d n_intervals_used=%d (need >= %d)'
              % (len(node.arrivals), got, MIN_INTERVALS_TO_MEASURE))
        sys.exit(2)

    print('SUMMARY n=%d used=%d excluded_gaps=%d p50=%.4f p95=%.4f p99=%.4f max=%.4f window=%.1f'
          % (stats['n_messages'], stats['n_intervals_used'], stats['n_gaps_excluded'],
             stats['p50'], stats['p95'], stats['p99'], stats['max'], stats['window_seconds']))
    sys.exit(0)


if __name__ == '__main__':
    main()
