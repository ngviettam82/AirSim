#!/usr/bin/env python3
"""P10.8b G-O3: replay-vs-original comparison + authority liveness watch.

G-O3(a/b): a bag is only evidence if replaying it reproduces what was
actually recorded. `reference` reads a bag DIRECTLY via rosbag2_py (no ROS
graph involved -- ground truth); `live` subscribes on a fresh domain while
the caller runs `ros2 bag play --clock` and counts what actually arrives;
`compare` diffs the two per plan S:6 row P10.8b's 3 metrics (authority
edges, mission/events count, odometry_fused sample count).

G-O3(c): `authority-watch` is the counterpart for the N-c fix itself
(docs/interface-contract-v0.1.md Sec2.20, plan Sec5) -- it watches a REAL,
standalone control_authority_manager_node's own /control/authority and
/diagnostics/control_authority while the caller replays cmd_* traffic
(optionally with `ros2 bag play --loop`, whose wrap point is a genuine
clock regression, not a synthetic gtest one) and reports whether
clock_regressions ever incremented and whether active_source ever fell back
to NONE.

Usage: see --help on each subcommand.
Exit convention (verify_observability.sh's): 0=PASS 1=FAIL 2=FAILED TO MEASURE
"""
import argparse
import json
import signal
import sys

MIN_ODOM_SAMPLES_TO_TRUST = 20  # below this a +-1% tolerance is meaningless (R27-1)
ODOM_TOLERANCE_FRACTION = 0.01  # plan S:6 row P10.8b: "lech <=1%"

SOURCE_NAMES = {0: 'NONE', 1: 'TEST', 2: 'MISSION', 3: 'OPERATOR', 4: 'SAFETY'}


def source_name(value):
    return SOURCE_NAMES.get(value, 'UNKNOWN(%d)' % value)


# P10.8c: the other 7 reliable_tl topics (authority is already tracked above
# as the edge-count metric) -- observability_params.yaml's parallel
# topics/qos_profiles arrays, index-matched by hand (all 8 TL entries carry
# depth=1 there). '/tf_static' has no per-uav prefix (same global exemption
# as /fmu/*, contract Sec2.20/S:4). Existence-only check (count > 0 on BOTH
# reference and live) -- these are NOT edge-count metrics like authority/
# mission_events, just proof the P10.8c offered_qos_profiles fix actually
# lets a late-joining TransientLocal subscriber see them on replay.
TL_TOPIC_SUFFIXES = [
    'state/estimator_source',
    'state/gnss_origin',
    'planning/trajectory',
    'safety/state',
    'mission/status',
    'world/mission_reference',
]


def tlTopicsForUav(uav_id):
    prefix = '/uav/%s' % uav_id
    topics = [prefix + '/' + suffix for suffix in TL_TOPIC_SUFFIXES]
    topics.append('/tf_static')   # global, unprefixed
    return topics


# --------------------------------------------------------------------- reference
def cmd_reference(args):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from uav_interfaces.msg import ControlAuthority

    prefix = '/uav/%s' % args.uav_id
    topic_authority = prefix + '/control/authority'
    topic_events = prefix + '/mission/events'
    topic_odom = prefix + '/state/odometry_fused'
    tl_topics = tlTopicsForUav(args.uav_id)

    storage_options = rosbag2_py.StorageOptions(uri=args.bag_dir, storage_id=args.storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as exc:  # bag not closed cleanly / not reindexed yet
        print('READ_ERROR=%r' % exc, file=sys.stderr)
        sys.exit(2)

    reader.set_filter(
        rosbag2_py.StorageFilter(topics=[topic_authority, topic_events, topic_odom] + tl_topics))

    authority_msgs = 0
    authority_edges = 0
    prev_source = None
    mission_events = 0
    odom_samples = 0
    tl_counts = {t: 0 for t in tl_topics}

    while reader.has_next():
        topic, data, _stamp_ns = reader.read_next()
        if topic == topic_authority:
            msg = deserialize_message(data, ControlAuthority)
            authority_msgs += 1
            if prev_source is not None and msg.active_source != prev_source:
                authority_edges += 1
            prev_source = msg.active_source
        elif topic == topic_events:
            mission_events += 1
        elif topic == topic_odom:
            odom_samples += 1
        elif topic in tl_counts:
            tl_counts[topic] += 1

    result = {
        'bag_dir': args.bag_dir, 'storage_id': args.storage_id, 'uav_id': args.uav_id,
        'authority_msgs': authority_msgs, 'authority_edges': authority_edges,
        'mission_events': mission_events, 'odom_samples': odom_samples,
        'tl_counts': tl_counts,
    }
    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2)
    print('REFERENCE authority_msgs=%d authority_edges=%d mission_events=%d odom_samples=%d'
          % (authority_msgs, authority_edges, mission_events, odom_samples))
    print('REFERENCE tl_counts=%s' % tl_counts)


# -------------------------------------------------------------------------- live
def cmd_live(args):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.task import Future
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from uav_interfaces.msg import (
        ControlAuthority, MissionEvent, MissionStatus, SafetyState, Trajectory3D)

    # Matches rosbag_manager_node's recorded qos_profiles (qosFromProfile()):
    # reliable_tl / reliable / be. Depth bumped on the subscriber side only --
    # QoS compatibility only needs reliability/durability to match (R27-4
    # spirit: measure what actually arrives, don't let our own queue be the
    # bottleneck being mistaken for a replay drop).
    authority_qos = QoSProfile(
        depth=200, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    events_qos = QoSProfile(
        depth=500, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)
    odom_qos = QoSProfile(
        depth=500, reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE)
    # P10.8c: the other 7 reliable_tl topics -- yaml depth=1 for every one of
    # them (observability_params.yaml), subscriber side widened to 500 for
    # the same R27-4 reason as authority_qos above.
    tl_qos = QoSProfile(
        depth=500, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)

    prefix = '/uav/%s' % args.uav_id

    class LiveCounter(Node):
        def __init__(self):
            super().__init__('o3_replay_gate_live')
            self.authority_msgs = 0
            self.authority_edges = 0
            self.prev_source = None
            self.mission_events = 0
            self.odom_samples = 0
            self.tl_counts = {t: 0 for t in tlTopicsForUav(args.uav_id)}
            self.create_subscription(
                ControlAuthority, prefix + '/control/authority', self._on_authority,
                authority_qos)
            self.create_subscription(
                MissionEvent, prefix + '/mission/events', self._on_event, events_qos)
            self.create_subscription(
                Odometry, prefix + '/state/odometry_fused', self._on_odom, odom_qos)
            # P10.8c: existence-only counters for the other 7 TL topics --
            # each closure captures its OWN topic name (default arg, not the
            # loop variable) so every subscription counts independently.
            tl_types = {
                prefix + '/state/estimator_source': String,
                prefix + '/state/gnss_origin': NavSatFix,
                prefix + '/planning/trajectory': Trajectory3D,
                prefix + '/safety/state': SafetyState,
                prefix + '/mission/status': MissionStatus,
                prefix + '/world/mission_reference': PoseStamped,
                '/tf_static': TFMessage,
            }
            for topic, msg_type in tl_types.items():
                self.create_subscription(
                    msg_type, topic,
                    (lambda _msg, t=topic: self.tl_counts.__setitem__(t, self.tl_counts[t] + 1)),
                    tl_qos)
            self.get_logger().info('o3_replay_gate live-counting on domain for %s' % args.uav_id)

        def _on_authority(self, msg):
            self.authority_msgs += 1
            if self.prev_source is not None and msg.active_source != self.prev_source:
                self.authority_edges += 1
            self.prev_source = msg.active_source

        def _on_event(self, _msg):
            self.mission_events += 1

        def _on_odom(self, _msg):
            self.odom_samples += 1

    # Same NO-default-signal-handler pattern as o2_latency_probe.py: rclpy's
    # own SIGINT handler tears the context down mid-wait before the JSON
    # report is written -- resolve the SAME stop_future from our handler
    # instead so SIGINT and the timer are one code path.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LiveCounter()
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

    result = {
        'uav_id': args.uav_id,
        'authority_msgs': node.authority_msgs, 'authority_edges': node.authority_edges,
        'mission_events': node.mission_events, 'odom_samples': node.odom_samples,
        'tl_counts': node.tl_counts,
    }
    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2)

    try:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    except Exception as exc:  # noqa: BLE001 -- best-effort, report already on disk
        print('cleanup warning (report already written): %r' % exc)

    print('LIVE authority_msgs=%d authority_edges=%d mission_events=%d odom_samples=%d'
          % (node.authority_msgs, node.authority_edges, node.mission_events, node.odom_samples))
    print('LIVE tl_counts=%s' % node.tl_counts)


# ----------------------------------------------------------------------- compare
def cmd_compare(args):
    with open(args.reference) as f:
        ref = json.load(f)
    with open(args.live) as f:
        live = json.load(f)

    checks = []

    def check(name, ok, detail):
        checks.append((name, ok, detail))

    check(
        'authority edges match 100%%',
        ref['authority_edges'] == live['authority_edges'],
        'reference=%d live=%d (authority_msgs reference=%d live=%d)'
        % (ref['authority_edges'], live['authority_edges'],
           ref['authority_msgs'], live['authority_msgs']))

    check(
        'mission/events count matches 100%%',
        ref['mission_events'] == live['mission_events'],
        'reference=%d live=%d' % (ref['mission_events'], live['mission_events']))

    ftm = ref['odom_samples'] < MIN_ODOM_SAMPLES_TO_TRUST
    if ftm:
        check(
            'odometry_fused sample count within 1%%', None,
            'FAILED TO MEASURE: reference odom_samples=%d < %d, a %%-tolerance is meaningless'
            % (ref['odom_samples'], MIN_ODOM_SAMPLES_TO_TRUST))
    else:
        diff = abs(ref['odom_samples'] - live['odom_samples'])
        frac = diff / ref['odom_samples']
        check(
            'odometry_fused sample count within 1%%', frac <= ODOM_TOLERANCE_FRACTION,
            'reference=%d live=%d diff=%d (%.3f%%, limit %.1f%%)'
            % (ref['odom_samples'], live['odom_samples'], diff, frac * 100,
               ODOM_TOLERANCE_FRACTION * 100))

    # P10.8c: all 8 reliable_tl topics present in the LIVE replay (existence
    # only, not edge-count) -- authority is covered by the edge-count check
    # above already being > 0 messages; the other 7 are new here. This is
    # the direct proof the offered_qos_profiles fix works: before it, `ros2
    # bag play` offered everything Volatile and a TransientLocal subscriber
    # got zero samples on every one of these.
    ref_tl = ref.get('tl_counts', {})
    live_tl = live.get('tl_counts', {})
    check(
        'authority (TL #1/8) present in replay', live.get('authority_msgs', 0) > 0,
        'reference=%d live=%d' % (ref.get('authority_msgs', 0), live.get('authority_msgs', 0)))
    for topic in sorted(ref_tl.keys()):
        ref_count = ref_tl.get(topic, 0)
        live_count = live_tl.get(topic, 0)
        if ref_count == 0:
            # No publisher ever fired for this topic in THIS bag/config (eg.
            # /tf_static has zero publishers anywhere in the codebase today;
            # gnss_origin never fires on a GPS-denied indoor world) -- that
            # is a data-availability fact about the flight, not a replay/QoS
            # defect, so it is FAILED TO MEASURE, not FAIL (same R30 split
            # as the odometry_fused ftm branch above).
            check(
                '%s present in replay' % topic, None,
                'FAILED TO MEASURE: reference=0 (no publisher for this topic in this run) -- '
                'replay fidelity for it cannot be judged from this bag')
        else:
            check(
                '%s present in replay' % topic, live_count > 0,
                'reference=%d live=%d' % (ref_count, live_count))

    any_ftm = False
    verdict = 0
    for name, ok, detail in checks:
        if ok is None:
            status = 'FTM '
            any_ftm = True
        else:
            status = 'PASS' if ok else 'FAIL'
            if not ok:
                verdict = max(verdict, 1)
        print('[%s] %-40s %s' % (status, name, detail))

    if any_ftm:
        verdict = max(verdict, 2)
    print('G-O3(a/b) verdict: %d (0=PASS 1=FAIL 2=FAILED TO MEASURE)' % verdict)
    sys.exit(verdict)


# ------------------------------------------------------------------ authority-watch
def cmd_authority_watch(args):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.task import Future
    from diagnostic_msgs.msg import DiagnosticArray
    from uav_interfaces.msg import ControlAuthority

    authority_qos = QoSProfile(
        depth=200, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # Matches control_authority_manager_node.cpp's own publisher QoS for this
    # topic: create_publisher<DiagnosticArray>(..., rclcpp::QoS(10)) with no
    # .best_effort()/.transient_local() call, i.e. plain Reliable/Volatile.
    diag_qos = QoSProfile(
        depth=200, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)

    prefix = '/uav/%s' % args.uav_id

    class Watcher(Node):
        def __init__(self):
            super().__init__('o3_authority_watch')
            self.active_source_samples = []   # (t_sec, value)
            self.saw_none_after_nonnone = False
            self.last_nonnone = False
            self.clock_regressions_samples = []  # (t_sec, count)
            self.create_subscription(
                ControlAuthority, prefix + '/control/authority', self._on_authority,
                authority_qos)
            self.create_subscription(
                DiagnosticArray, prefix + '/diagnostics/control_authority', self._on_diag,
                diag_qos)
            self.get_logger().info('o3_authority_watch subscribed for %s' % args.uav_id)

        def _on_authority(self, msg):
            t = self.get_clock().now().nanoseconds / 1e9
            self.active_source_samples.append((t, int(msg.active_source)))
            if msg.active_source == ControlAuthority.SOURCE_NONE and self.last_nonnone:
                self.saw_none_after_nonnone = True
            self.last_nonnone = msg.active_source != ControlAuthority.SOURCE_NONE

        def _on_diag(self, msg):
            for status in msg.status:
                if status.name != 'control_authority: clock regressions':
                    continue
                for kv in status.values:
                    if kv.key == 'clock_regressions':
                        t = self.get_clock().now().nanoseconds / 1e9
                        try:
                            count = int(float(kv.value))
                        except ValueError:
                            continue
                        self.clock_regressions_samples.append((t, count))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Watcher()
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

    final_clock_regressions = node.clock_regressions_samples[-1][1] \
        if node.clock_regressions_samples else None
    final_active_source = node.active_source_samples[-1][1] if node.active_source_samples else None
    ever_none = any(v == ControlAuthority.SOURCE_NONE for _t, v in node.active_source_samples)
    # P10.9a G-O3(c) rerun: positive-assertion requirement -- callers must
    # confirm the arbiter genuinely received cmd_mission (active_source
    # reached MISSION) BEFORE drawing any conclusion from clock_regressions,
    # since a QoS/double-writer bug can silently make the whole watch a
    # no-op that still "passes" a naive clock_regressions==0 check.
    mission_samples = [
        (t, v) for t, v in node.active_source_samples if v == ControlAuthority.SOURCE_MISSION]
    ever_mission = len(mission_samples) > 0

    result = {
        'uav_id': args.uav_id,
        'n_authority_samples': len(node.active_source_samples),
        'n_diag_samples': len(node.clock_regressions_samples),
        'final_clock_regressions': final_clock_regressions,
        'final_active_source': final_active_source,
        'final_active_source_name': (
            source_name(final_active_source) if final_active_source is not None else None),
        'ever_saw_none': ever_none,
        'saw_none_after_nonnone': node.saw_none_after_nonnone,
        'ever_saw_mission': ever_mission,
        'first_mission_at_t': mission_samples[0][0] if mission_samples else None,
        'clock_regressions_series': node.clock_regressions_samples,
        'active_source_series': node.active_source_samples,
    }
    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2)

    try:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    except Exception as exc:  # noqa: BLE001
        print('cleanup warning (report already written): %r' % exc)

    print(
        'AUTHORITY_WATCH n_authority=%d n_diag=%d final_clock_regressions=%s '
        'final_active_source=%s ever_saw_none=%s saw_none_after_nonnone=%s ever_saw_mission=%s '
        'first_mission_at_t=%s'
        % (len(node.active_source_samples), len(node.clock_regressions_samples),
           final_clock_regressions, source_name(final_active_source) if final_active_source
           is not None else 'NEVER_SEEN', ever_none, node.saw_none_after_nonnone, ever_mission,
           mission_samples[0][0] if mission_samples else None))
    if final_clock_regressions is None or final_active_source is None:
        sys.exit(2)
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)

    p = sub.add_parser('reference')
    p.add_argument('--bag-dir', required=True)
    p.add_argument('--storage-id', default='sqlite3')
    p.add_argument('--uav-id', default='uav0')
    p.add_argument('--output', required=True)

    p = sub.add_parser('live')
    p.add_argument('--uav-id', default='uav0')
    p.add_argument('--duration', type=float, default=180.0)
    p.add_argument('--output', required=True)

    p = sub.add_parser('compare')
    p.add_argument('--reference', required=True)
    p.add_argument('--live', required=True)

    p = sub.add_parser('authority-watch')
    p.add_argument('--uav-id', default='uav0')
    p.add_argument('--duration', type=float, default=180.0)
    p.add_argument('--output', required=True)

    args = parser.parse_args()
    {
        'reference': cmd_reference,
        'live': cmd_live,
        'compare': cmd_compare,
        'authority-watch': cmd_authority_watch,
    }[args.mode](args)


if __name__ == '__main__':
    main()
