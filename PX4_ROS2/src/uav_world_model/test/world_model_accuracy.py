#!/usr/bin/env python3
"""P5.6 gates S5/S6: landmark in odom vs a marker at a known pose. Run via verify_world_model.sh."""

import argparse
import json
import math
import statistics
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from uav_interfaces.msg import (
    ControlAuthority,
    ControlCommand,
    LocalizationStatus,
    MarkerObservation,
    OffboardStatus,
    SemanticLandmarkArray,
    VehicleState,
)
from uav_interfaces.srv import Arm, Disarm, SetFlightMode

AUTHORITY_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Gazebo reports the model origin; base_link sits this much higher.
MODEL_ORIGIN_TO_BASE_LINK = 0.240

# G2 condition A worst per-axis error 0.106 m, P5.3 marker error 0.036 m at 2.5 m,
# plus 0.02 m for the mount. Asking for better than the pose allows is impossible.
POSITION_TOLERANCE_M = 0.15

# Frame offset beyond this is a frame fault, not noise; 0.240 m has bitten twice.
FRAME_OFFSET_TOLERANCE_M = 0.10

MIN_SAMPLES = 5

# How much wall time a sim-time wait may consume before we call the run unusable.
WALL_CLOCK_SAFETY_FACTOR = 6.0

STATE_ACTIVE = OffboardStatus.STATE_ACTIVE
MODE_LAND = VehicleState.FLIGHT_MODE_LAND

SOURCE_NAMES = {0: 'none', 1: 'gps', 2: 'vio', 3: 'optical_flow', 4: 'fused'}


class Failure(Exception):
    """Raised when the flight cannot be completed; the run reports no number."""


class WorldModelAccuracy(Node):
    def __init__(self, uav_id, marker_id):
        super().__init__(
            'world_model_accuracy',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        prefix = '/uav/' + uav_id

        self.uav_id = uav_id
        self.marker_id = marker_id
        self.state = None
        self.offboard = None
        self.raw_odom = None
        self.fused = None
        self.truth = None
        self.target = None
        self.collecting = False
        self.samples = []           # (x, y, z, uncertainty, age)
        self.marker_seen = 0
        self.sources = set()
        self.frames = set()
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
            Odometry, prefix + '/state/odometry_fused', self._on_fused, 10)
        self.create_subscription(
            Odometry, prefix + '/localization/vio_odometry_raw', self._on_truth, 20)
        self.create_subscription(
            LocalizationStatus, prefix + '/state/localization_status', self._on_status, 10)
        self.create_subscription(
            MarkerObservation, prefix + '/perception/markers', self._on_marker, 20)
        self.create_subscription(
            SemanticLandmarkArray, prefix + '/world/semantic_landmarks', self._on_landmarks, 10)

        self.create_timer(0.05, self._stream)

    def _stream(self):
        if self.target is None:
            return
        command = ControlCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'odom'
        command.uav_id = self.uav_id
        command.control_mode = ControlCommand.MODE_POSITION
        command.position.x, command.position.y, command.position.z = self.target
        command.yaw = 0.0
        command.source = ControlAuthority.SOURCE_TEST
        self._require_well_formed(command)
        self.command_pub.publish(command)

    def _require_well_formed(self, command):
        # The arbiter DROPs these silently (S:3c); catch it here so a bug
        # aborts loudly instead of looking like a hung flight.
        if command.header.frame_id != 'odom':
            raise Failure('probe command frame_id %r is not odom' % command.header.frame_id)
        if command.header.stamp.sec == 0 and command.header.stamp.nanosec == 0:
            raise Failure('probe command stamp is unset')
        fields = (command.position.x, command.position.y, command.position.z, command.yaw)
        if not all(math.isfinite(v) for v in fields):
            raise Failure('probe command has a non-finite field: %r' % (fields,))

    def _on_state(self, message):
        self.state = message

    def _on_authority(self, message):
        self.authority = message
        self._authority_stamp = self.get_clock().now()

    def _authority_is_fresh(self, source):
        # Y16: TransientLocal means a STALE cached message can satisfy an
        # active_source value check without a single new message ever having
        # arrived -- freshness (2 Hz heartbeat, so well under 1 s when alive)
        # is what actually tells "granted and confirmed alive" from "last
        # known value, arbiter might be dead".
        #
        # B2-d3: measured on SIM time (this node runs use_sim_time=True, same
        # as control_authority_manager_node), not the wall clock -- this
        # probe flies a camera model (uav0_full-class RTF), where a wall-clock
        # "< 1 s" window can elapse before even one sim-time heartbeat period
        # has passed, aborting the flight on a false "is
        # control_authority_manager_node running?".
        if self.authority is None or self._authority_stamp is None:
            return False
        if self.authority.active_source != source:
            return False
        age_sec = (self.get_clock().now() - self._authority_stamp).nanoseconds * 1e-9
        return age_sec < 1.0

    def _on_offboard(self, message):
        self.offboard = message

    def _on_raw(self, message):
        self.raw_odom = message

    def _on_fused(self, message):
        self.fused = message

    def _on_truth(self, message):
        self.truth = message

    def _on_status(self, message):
        self.sources.add(SOURCE_NAMES.get(message.active_source, str(message.active_source)))

    def _on_marker(self, message):
        if message.marker_id == self.marker_id:
            self.marker_seen += 1

    def _on_landmarks(self, message):
        self.frames.add(message.header.frame_id)
        if not self.collecting:
            return
        for landmark in message.landmarks:
            if landmark.marker_id != self.marker_id:
                continue
            self.samples.append((
                landmark.pose.position.x,
                landmark.pose.position.y,
                landmark.pose.position.z,
                landmark.position_uncertainty,
                landmark.time_since_seen_sec,
            ))

    def sim_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin(self, seconds):
        """Measured on /clock: a wall-clock wait shrinks with the real-time factor."""
        start_sim = self.sim_seconds()
        wall_deadline = time.time() + seconds * WALL_CLOCK_SAFETY_FACTOR
        while rclpy.ok() and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.sim_seconds() - start_sim >= seconds:
                return
        raise Failure('sim clock advanced %.1f s of %.1f s before the wall clock ran out'
                      % (self.sim_seconds() - start_sim, seconds))

    def wait_until(self, predicate, timeout, description):
        wall_deadline = time.time() + timeout * WALL_CLOCK_SAFETY_FACTOR
        while rclpy.ok() and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise Failure('timed out waiting for %s' % description)

    def position(self):
        if self.raw_odom is None:
            return None
        point = self.raw_odom.pose.pose.position
        return (point.x, point.y, point.z)

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

    def frame_offset(self):
        """Fused pose minus ground truth lifted to base_link; a frame fault shows up here."""
        if self.fused is None or self.truth is None:
            return None
        fused = self.fused.pose.pose.position
        truth = self.truth.pose.pose.position
        return (fused.x - truth.x,
                fused.y - truth.y,
                fused.z - (truth.z + MODEL_ORIGIN_TO_BASE_LINK))

    def fly(self, hover, seconds):
        arm_client = self.create_client(Arm, '/uav/%s/backend/arm' % self.uav_id)
        disarm_client = self.create_client(Disarm, '/uav/%s/backend/disarm' % self.uav_id)
        mode_client = self.create_client(SetFlightMode, '/uav/%s/backend/set_mode' % self.uav_id)

        self.wait_until(
            lambda: self.state is not None and self.state.connected, 60.0, 'autopilot')
        self.wait_until(lambda: self.raw_odom is not None, 30.0, 'odometry')
        self.wait_until(lambda: self.fused is not None, 30.0, 'fused odometry')

        origin = self.position()
        self.target = origin
        self.offboard = None
        self.spin(1.5)
        self.wait_until(
            lambda: self._authority_is_fresh(ControlAuthority.SOURCE_TEST),
            10.0, 'control authority granted to TEST (fresh within 1 s) -- '
            'is control_authority_manager_node running?')
        self.wait_until(
            lambda: self.offboard is not None and self.offboard.state == STATE_ACTIVE,
            30.0, 'offboard engagement')
        self.call(arm_client, Arm.Request(), 15.0, 'arm')

        self.target = (origin[0], origin[1], origin[2] + hover[2])
        self.spin(10.0)
        self.target = (hover[0], hover[1], origin[2] + hover[2])
        self.spin(18.0)                 # fly across and settle before believing anything

        offset = self.frame_offset()
        self.samples = []
        self.collecting = True
        self.spin(seconds)
        self.collecting = False

        request = SetFlightMode.Request()
        request.mode = MODE_LAND
        self.call(mode_client, request, 15.0, 'set land mode')
        self.spin(1.0)
        self.target = None
        try:
            self.wait_until(lambda: not self.state.armed, 40.0, 'automatic disarm')
        except Failure:
            self.call(disarm_client, Disarm.Request(), 15.0, 'disarm')

        return offset


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--marker-id', type=int, default=7)
    parser.add_argument('--marker-x', type=float, required=True)
    parser.add_argument('--marker-y', type=float, required=True)
    parser.add_argument('--marker-z', type=float, default=0.01)
    # Deliberately off to one side: an axis swap or sign flip then shows up big.
    parser.add_argument('--hover-dx', type=float, default=-0.5)
    parser.add_argument('--hover-dy', type=float, default=-0.3)
    parser.add_argument('--height', type=float, default=2.5)
    parser.add_argument('--seconds', type=float, default=15.0)
    parser.add_argument('--label', default='A')
    parser.add_argument('--out', default='')
    parser.add_argument('--check-position', action='store_true')
    args = parser.parse_args()

    truth = (args.marker_x, args.marker_y, args.marker_z)
    hover = (args.marker_x + args.hover_dx, args.marker_y + args.hover_dy, args.height)

    rclpy.init()
    node = WorldModelAccuracy(args.uav_id, args.marker_id)
    try:
        offset = node.fly(hover, args.seconds)
        if len(node.samples) < MIN_SAMPLES:
            raise Failure('only %d landmark samples (marker seen %d times)'
                          % (len(node.samples), node.marker_seen))
    except Failure as exc:
        print('P5.6 %s FAILED TO MEASURE: %s' % (args.label, exc))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    measured = tuple(statistics.median([sample[axis] for sample in node.samples])
                     for axis in range(3))
    uncertainty = statistics.median([sample[3] for sample in node.samples])
    age = statistics.median([sample[4] for sample in node.samples])
    error = tuple(measured[axis] - truth[axis] for axis in range(3))
    distance = math.sqrt(sum(component ** 2 for component in error))

    print()
    print('=== P5.6 world model, condition %s, ArUco id %d ===' % (args.label, args.marker_id))
    print('  samples          %d over %.0f s (marker detected %d times)'
          % (len(node.samples), args.seconds, node.marker_seen))
    print('  localization     source(s) %s' % (', '.join(sorted(node.sources)) or 'unknown'))
    print('  landmark frame   %s' % (', '.join(sorted(node.frames)) or 'unknown'))
    if offset is not None:
        print('  frame offset     fused - truth = (%+.3f %+.3f %+.3f) m' % offset)
    print('  marker truth     (%.3f %.3f %.3f) m in world' % truth)
    print('  landmark in odom (%.3f %.3f %.3f) m' % measured)
    print('  error            (%+.3f %+.3f %+.3f) m, distance %.3f m (tolerance %.2f)'
          % (error + (distance, POSITION_TOLERANCE_M)))
    print('  uncertainty      %.4f m   age at publish %.3f s' % (uncertainty, age))

    failures = []
    if uncertainty <= 0.0:
        failures.append('uncertainty %.4f is not positive' % uncertainty)
    if offset is None:
        failures.append('no ground truth, so the frame offset went unchecked')
    elif args.check_position:
        worst_offset = max(abs(component) for component in offset)
        if worst_offset > FRAME_OFFSET_TOLERANCE_M:
            failures.append('frame offset %.3f m: the odom frame itself is wrong'
                            % worst_offset)
    if node.frames != {'odom'}:
        failures.append('landmarks arrived in %s, not odom' % sorted(node.frames))
    if args.check_position and distance > POSITION_TOLERANCE_M:
        failures.append('landmark off by %.3f m' % distance)

    if args.out:
        with open(args.out, 'w') as handle:
            json.dump({
                'label': args.label,
                'samples': len(node.samples),
                'frame_offset': offset,
                'measured': measured,
                'error': error,
                'distance': distance,
                'uncertainty': uncertainty,
                'age': age,
                'sources': sorted(node.sources),
            }, handle)

    node.destroy_node()
    rclpy.shutdown()

    if failures:
        print('  RESULT: FAIL (%s)' % '; '.join(failures))
        return 1
    print('  RESULT: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
