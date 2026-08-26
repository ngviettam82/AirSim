#!/usr/bin/env python3
"""G2: odometry_fused vs Gazebo ground truth. Gate and injection level: see README."""

import argparse
import math
import statistics
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from uav_interfaces.msg import (
    ControlAuthority,
    ControlCommand,
    LocalizationStatus,
    OffboardStatus,
    VehicleState,
)
from uav_interfaces.srv import Arm, Disarm, SetFlightMode

AUTHORITY_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

CRUISE_ALTITUDE = 2.5           # m above the starting point
RMSE_GATE = 0.5                 # m per axis; set with the injection level below
PAIRING_TOLERANCE = 0.05        # s, max gap when matching the two series
STATIONARY_SECONDS = 3.0        # ground window used to measure the frame offset
RTF_GATE = 0.95                 # below this a flight measurement is not trusted

# Timed legs keep the window at 60 s; see README.
LEGS = [
    (0.0, 0.0, CRUISE_ALTITUDE, 6.0),
    (4.0, 0.0, CRUISE_ALTITUDE, 9.0),
    (4.0, 4.0, CRUISE_ALTITUDE, 9.0),
    (4.0, 4.0, CRUISE_ALTITUDE + 1.5, 8.0),
    (0.0, 4.0, CRUISE_ALTITUDE + 1.5, 9.0),
    (0.0, 4.0, CRUISE_ALTITUDE, 8.0),
    (0.0, 0.0, CRUISE_ALTITUDE, 11.0),
]

SOURCE_NAMES = {0: 'NONE', 1: 'GPS', 2: 'VIO', 3: 'OPTICAL_FLOW', 4: 'FUSED'}
STATE_ACTIVE = OffboardStatus.STATE_ACTIVE
MODE_LAND = VehicleState.FLIGHT_MODE_LAND


class Failure(Exception):
    """Raised when the flight cannot be completed; the run reports no number."""


class FusedAccuracy(Node):
    def __init__(self, uav_id):
        super().__init__(
            'g2_fused_accuracy',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        prefix = '/uav/' + uav_id

        self.uav_id = uav_id
        self.state = None
        self.offboard = None
        self.raw_odom = None
        self.target = None
        self.recording = False
        self.fused = []             # (sim_time, x, y, z)
        self.truth = []             # (sim_time, x, y, z)
        self.sources = {}           # source id -> sample count
        self.selected = {}          # source name the mux actually used -> count
        self.authority = None
        self._authority_stamp = None    # Y16/B2-d3: SIM-time of the last authority msg

        self.command_pub = self.create_publisher(
            ControlCommand, prefix + '/control/cmd_test', 10)
        self.create_subscription(VehicleState, prefix + '/state/vehicle', self._on_state, 10)
        self.create_subscription(
            OffboardStatus, prefix + '/backend/offboard_status', self._on_offboard, 10)
        self.create_subscription(
            ControlAuthority, prefix + '/control/authority', self._on_authority, AUTHORITY_QOS)
        self.create_subscription(
            Odometry, prefix + '/state/odometry_raw', self._on_raw, 10)
        self.create_subscription(
            Odometry, prefix + '/state/odometry_fused', self._on_fused, 50)
        self.create_subscription(
            Odometry, prefix + '/localization/vio_odometry_raw', self._on_truth, 50)
        self.create_subscription(
            LocalizationStatus, prefix + '/state/localization_status', self._on_status, 10)
        # Status says FUSED, not which input won; see README.
        self.create_subscription(
            String, prefix + '/state/estimator_source', self._on_source,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.arm_client = self.create_client(Arm, prefix + '/backend/arm')
        self.disarm_client = self.create_client(Disarm, prefix + '/backend/disarm')
        self.mode_client = self.create_client(SetFlightMode, prefix + '/backend/set_mode')

        self.create_timer(0.05, self._stream)      # 20 Hz


    def _stream(self):
        if self.target is None:
            return
        cmd = ControlCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'odom'
        cmd.uav_id = self.uav_id
        cmd.control_mode = ControlCommand.MODE_POSITION
        cmd.position.x, cmd.position.y, cmd.position.z = self.target
        cmd.yaw = 0.0
        cmd.source = ControlAuthority.SOURCE_TEST
        self._require_well_formed(cmd)
        self.command_pub.publish(cmd)

    def _require_well_formed(self, cmd):
        # The arbiter DROPs these silently (S:3c); catch it here so a bug
        # aborts loudly instead of looking like a hung flight.
        if cmd.header.frame_id != 'odom':
            raise Failure('probe command frame_id %r is not odom' % cmd.header.frame_id)
        if cmd.header.stamp.sec == 0 and cmd.header.stamp.nanosec == 0:
            raise Failure('probe command stamp is unset')
        fields = (cmd.position.x, cmd.position.y, cmd.position.z, cmd.yaw)
        if not all(math.isfinite(v) for v in fields):
            raise Failure('probe command has a non-finite field: %r' % (fields,))

    def _on_state(self, msg):
        self.state = msg

    def _on_authority(self, msg):
        self.authority = msg
        self._authority_stamp = self.get_clock().now()

    def _authority_is_fresh(self, source):
        # Y16: TransientLocal means a STALE cached message can satisfy an
        # active_source value check without a single new message ever having
        # arrived -- freshness (2 Hz heartbeat, so well under 1 s when alive)
        # is what actually tells "granted and confirmed alive" from "last
        # known value, arbiter might be dead".
        #
        # B2-d3: measured on SIM time (this node runs use_sim_time=True, same
        # as control_authority_manager_node), not the wall clock -- under a
        # low RTF a wall-clock "< 1 s" window can elapse before even one
        # sim-time heartbeat period has passed, aborting the flight on a
        # false "is control_authority_manager_node running?".
        if self.authority is None or self._authority_stamp is None:
            return False
        if self.authority.active_source != source:
            return False
        age_sec = (self.get_clock().now() - self._authority_stamp).nanoseconds * 1e-9
        return age_sec < 1.0

    def _on_offboard(self, msg):
        self.offboard = msg

    def _on_raw(self, msg):
        self.raw_odom = msg

    def _sample(self, sink, msg):
        if not self.recording:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        sink.append((stamp, p.x, p.y, p.z))

    def _on_fused(self, msg):
        self._sample(self.fused, msg)

    def _on_truth(self, msg):
        self._sample(self.truth, msg)

    def _on_status(self, msg):
        if self.recording:
            self.sources[msg.active_source] = self.sources.get(msg.active_source, 0) + 1

    def _on_source(self, msg):
        # Ungated: transient_local delivers once, before recording.
        self.selected[msg.data] = self.selected.get(msg.data, 0) + 1


    def spin(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_until(self, predicate, timeout, description):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise Failure('timed out waiting for %s' % description)

    def position(self):
        if self.raw_odom is None:
            return None
        p = self.raw_odom.pose.pose.position
        return (p.x, p.y, p.z)

    def call(self, client, request, timeout, description):
        if not client.wait_for_service(timeout_sec=10.0):
            raise Failure('%s service never appeared' % description)
        future = client.call_async(request)
        end = time.time() + timeout
        while rclpy.ok() and not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise Failure('%s call timed out' % description)
        result = future.result()
        if not result.success:
            raise Failure('%s rejected: %s' % (description, result.message))
        return result


    def fly(self):
        self.wait_until(
            lambda: self.state is not None and self.state.connected, 60.0, 'autopilot')
        self.wait_until(lambda: self.raw_odom is not None, 30.0, 'odometry')

        origin = self.position()
        self.target = origin
        self.offboard = None
        self.spin(1.5)

        self.wait_until(
            lambda: self._authority_is_fresh(ControlAuthority.SOURCE_TEST),
            10.0, 'control authority granted to TEST (fresh within 1 s) -- '
            'is control_authority_manager_node running?')

        # Offboard before arm; see README.
        self.wait_until(
            lambda: self.offboard is not None and self.offboard.state == STATE_ACTIVE,
            30.0, 'offboard engagement')
        self.call(self.arm_client, Arm.Request(), 15.0, 'arm')

        # Recording starts on the ground: those samples set the frame offset.
        self.recording = True
        wall_start = time.time()
        sim_start = self.get_clock().now().nanoseconds * 1e-9
        self.spin(STATIONARY_SECONDS)

        for east, north, up, seconds in LEGS:
            self.target = (origin[0] + east, origin[1] + north, origin[2] + up)
            self.spin(seconds)

        self.recording = False
        sim_elapsed = self.get_clock().now().nanoseconds * 1e-9 - sim_start
        wall_elapsed = time.time() - wall_start

        request = SetFlightMode.Request()
        request.mode = MODE_LAND
        self.call(self.mode_client, request, 15.0, 'set land mode')
        self.spin(1.0)
        self.target = None
        try:
            self.wait_until(lambda: not self.state.armed, 40.0, 'automatic disarm')
        except Failure:
            self.call(self.disarm_client, Disarm.Request(), 15.0, 'disarm')

        return sim_elapsed, wall_elapsed


def pair_series(fused, truth):
    """Match each fused sample to the nearest truth sample in simulated time."""
    if not truth:
        return []
    times = [t for t, _, _, _ in truth]
    pairs = []
    cursor = 0
    for stamp, fx, fy, fz in fused:
        while cursor + 1 < len(times) and abs(times[cursor + 1] - stamp) <= abs(times[cursor] - stamp):
            cursor += 1
        if abs(times[cursor] - stamp) <= PAIRING_TOLERANCE:
            _, tx, ty, tz = truth[cursor]
            pairs.append((stamp, (fx, fy, fz), (tx, ty, tz)))
    return pairs


def analyse(pairs, stationary_end, forced_offset=None):
    """RMSE per axis after removing the constant offset between the two frames.

    Estimating that offset from the ground window is only sound while the
    injectors are off. With them on, whatever error the estimator has already
    accumulated by the time recording starts is a constant too, so estimating
    here quietly subtracts real error and reports a lower bound. Pass the
    physical offset instead, and the gap between the two becomes a reading of
    the error already present at t=0.
    """
    ground = [p for p in pairs if p[0] <= stationary_end]
    if len(ground) < 5:
        raise Failure('only %d stationary samples; cannot fix the frame offset' % len(ground))
    estimated = tuple(
        statistics.median([truth[i] - fused[i] for _, fused, truth in ground])
        for i in range(3)
    )
    offset = forced_offset if forced_offset is not None else estimated

    flight = [p for p in pairs if p[0] > stationary_end]
    if len(flight) < 100:
        raise Failure('only %d in-flight samples' % len(flight))

    errors = [[], [], []]
    for _, fused, truth in flight:
        for i in range(3):
            errors[i].append((truth[i] - offset[i]) - fused[i])

    return {
        'offset': offset,
        'estimated_offset': estimated,
        'error_at_start': tuple(estimated[i] - offset[i] for i in range(3)),
        'ground_samples': len(ground),
        'flight_samples': len(flight),
        'rmse': [math.sqrt(sum(e * e for e in axis) / len(axis)) for axis in errors],
        'max_abs': [max(abs(e) for e in axis) for axis in errors],
        'bias': [statistics.mean(axis) for axis in errors],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--label', default='', help='tag for the printed report')
    parser.add_argument(
        '--frame-offset',
        help='x,y,z of the physical odom-to-Gazebo offset. Required whenever the '
             'injectors are on, or accumulated error gets absorbed as frame offset.')
    args = parser.parse_args()

    forced_offset = None
    if args.frame_offset:
        parts = args.frame_offset.split(',')
        if len(parts) != 3:
            print('--frame-offset needs three comma-separated numbers')
            return 1
        forced_offset = tuple(float(p) for p in parts)

    rclpy.init()
    node = FusedAccuracy(args.uav_id)
    try:
        sim_elapsed, wall_elapsed = node.fly()
        rtf = sim_elapsed / wall_elapsed if wall_elapsed > 0 else 0.0
        stationary_end = node.fused[0][0] + STATIONARY_SECONDS if node.fused else 0.0
        pairs = pair_series(node.fused, node.truth)
        if not pairs:
            raise Failure('no fused/truth pairs within %.0f ms' % (PAIRING_TOLERANCE * 1000))
        result = analyse(pairs, stationary_end, forced_offset)
    except Failure as exc:
        print('G2 %s: FAILED TO MEASURE: %s' % (args.label, exc))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    sources = ', '.join(
        '%s=%d' % (SOURCE_NAMES.get(k, str(k)), v) for k, v in sorted(node.sources.items()))
    print()
    print('=== G2 %s ===' % (args.label or '(no label)'))
    print('  window          %.1f s sim / %.1f s wall  -> RTF %.3f'
          % (sim_elapsed, wall_elapsed, rtf))
    print('  samples         %d fused, %d truth, %d paired (%d on the ground)'
          % (len(node.fused), len(node.truth), len(pairs), result['ground_samples']))
    selected = ', '.join('%s=%d' % (k, v) for k, v in sorted(node.selected.items()))
    print('  mux label       %s' % (sources or 'none reported'))
    print('  source selected %s' % (selected or 'none reported'))
    print('  frame offset    x %+.3f  y %+.3f  z %+.3f  m  (%s)'
          % (result['offset'] + ('given' if forced_offset else 'estimated',)))
    if forced_offset:
        print('  error at t=0    x %+.3f  y %+.3f  z %+.3f  m  (estimated minus given)'
              % result['error_at_start'])
    print('  RMSE            x %.3f  y %.3f  z %.3f  m   (gate %.2f)'
          % (result['rmse'][0], result['rmse'][1], result['rmse'][2], RMSE_GATE))
    print('  bias            x %+.3f  y %+.3f  z %+.3f  m' % tuple(result['bias']))
    print('  max abs error   x %.3f  y %.3f  z %.3f  m' % tuple(result['max_abs']))

    worst = max(result['rmse'])
    passed = worst < RMSE_GATE
    if rtf < RTF_GATE:
        print('  RESULT: NOT TRUSTED - RTF %.3f is below %.2f, rerun on a quiet machine'
              % (rtf, RTF_GATE))
        verdict = 2
    else:
        print('  RESULT: %s (worst axis %.3f m)' % ('PASS' if passed else 'FAIL', worst))
        verdict = 0 if passed else 1

    node.destroy_node()
    rclpy.shutdown()
    return verdict


if __name__ == '__main__':
    sys.exit(main())
