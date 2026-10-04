#!/usr/bin/env python3
"""N6 positive control: prove assert_single_writer() can BITE on a REAL ROS graph.

Why this exists. The N6 fix (P10-gate-debt review round 2) was verified only by
extracting the function with ast+exec() and driving it with a FAKE node/rclpy --
so nothing had ever shown it behaves the same against real discovery. A gate
that has only ever been seen to PASS is not evidence (ops-playbook S:7).

G-N5 supplies the "does not bite falsely" half: it calls assert_single_writer
first thing on a live stack and exits 0. This supplies the other half -- three
cases where it MUST raise, and one where it must not:

  A) discovery probe topic has no publisher at all
        -> Failure (FAILED TO MEASURE), never a silent pass
  B) target topic carries two writers while expected == 1
        -> Failure naming the count
  C) a second writer appears only AFTER the first sample
        -> Failure, because the hold window keeps sampling (the pre-N6 code
           read the count once and would have passed here)
  D) exactly one writer, probe topic alive
        -> returns cleanly

R20 exception: runs on its own domain, it needs no live stack.
R1/R25: no px4_msgs anywhere.

Usage: python3 n6_single_writer_selftest.py
Exit 0 = all four cases behaved; exit 1 otherwise.
"""
import pathlib
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from uav_interfaces.msg import TargetState

GATE = pathlib.Path(__file__).with_name('gn5_followtrack_gate.py')

PROBE_TOPIC = '/n6selftest/probe'
TARGET_TOPIC = '/n6selftest/target'


def load_gate_symbols():
    """Import the gate module by path; it guards its own main()."""
    import importlib.util
    spec = importlib.util.spec_from_file_location('gn5_gate', GATE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.assert_single_writer, module.Failure


def main():
    if not GATE.exists():
        print('FAILED TO MEASURE: %s missing' % GATE)
        return 1
    assert_single_writer, Failure = load_gate_symbols()

    rclpy.init()
    node = Node('n6_selftest')
    results = []

    def record(name, ok, detail):
        results.append((name, ok, detail))
        print('  [%s] %s  %s' % ('PASS' if ok else 'FAIL', name, detail))

    # ---- A) no publisher on the discovery probe topic at all -------------
    t0 = time.time()
    try:
        assert_single_writer(node, TARGET_TOPIC, 0, PROBE_TOPIC, timeout_sec=3.0)
        record('A discovery probe with no publisher must raise', False,
               'returned cleanly -- a count read before discovery converged was trusted')
    except Failure as exc:
        record('A discovery probe with no publisher must raise', True,
               'raised after %.1fs: %s' % (time.time() - t0, str(exc)[:90]))

    # From here the probe topic has a live writer, so discovery can converge.
    probe_writer = node.create_publisher(TargetState, PROBE_TOPIC, 1)

    # ---- B) two writers where one is expected ----------------------------
    first = node.create_publisher(TargetState, TARGET_TOPIC, 1)
    second = node.create_publisher(TargetState, TARGET_TOPIC, 1)
    try:
        assert_single_writer(node, TARGET_TOPIC, 1, PROBE_TOPIC,
                             timeout_sec=10.0, hold_sec=1.5)
        record('B two writers where one is expected must raise', False,
               'returned cleanly with 2 publishers on the topic')
    except Failure as exc:
        record('B two writers where one is expected must raise', True,
               str(exc)[:90])
    node.destroy_publisher(second)

    # ---- C) the second writer appears only after the first sample --------
    late = {}

    def add_late_writer():
        time.sleep(0.8)
        late['pub'] = node.create_publisher(TargetState, TARGET_TOPIC, 1)

    thread = threading.Thread(target=add_late_writer)
    thread.start()
    try:
        assert_single_writer(node, TARGET_TOPIC, 1, PROBE_TOPIC,
                             timeout_sec=10.0, hold_sec=3.0, sample_period_sec=0.25)
        record('C a writer appearing mid-window must raise', False,
               'returned cleanly -- the hold window is not actually sampling')
    except Failure as exc:
        record('C a writer appearing mid-window must raise', True, str(exc)[:90])
    thread.join()
    if 'pub' in late:
        node.destroy_publisher(late['pub'])

    # ---- D) the clean case must still pass -------------------------------
    try:
        assert_single_writer(node, TARGET_TOPIC, 1, PROBE_TOPIC,
                             timeout_sec=10.0, hold_sec=1.5)
        record('D exactly one writer must pass', True, '1 writer held for the window')
    except Failure as exc:
        record('D exactly one writer must pass', False,
               'raised on a clean graph: %s' % str(exc)[:90])

    node.destroy_publisher(first)
    node.destroy_publisher(probe_writer)
    node.destroy_node()
    rclpy.shutdown()

    passed = all(ok for _, ok, _ in results)
    print('RESULT: %s (%d/%d)' % ('PASS' if passed else 'FAIL',
                                  sum(1 for _, ok, _ in results if ok), len(results)))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
