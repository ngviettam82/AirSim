#!/usr/bin/env python3
"""P5.3 gate: marker ID and distance at two heights. Run via verify_marker_detector.sh."""

import argparse
import math
import os
import statistics
import sys
import time

FRAME_DIR = os.path.expanduser('~/marker_debug')

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from uav_interfaces.msg import (
    ControlAuthority,
    ControlCommand,
    MarkerObservation,
    OffboardStatus,
    VehicleState,
)
from uav_interfaces.srv import Arm, Disarm, SetFlightMode

AUTHORITY_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

CAMERA_BELOW_BASE_LINK = 0.065  # sensor_camera_down mount on uav0_nav

# Gazebo odometry reports the model origin, not base_link (see README).
MODEL_ORIGIN_TO_BASE_LINK = 0.240

MARKER_GROUND_Z = 0.01          # clear of the ground, avoids z-fighting

DISTANCE_TOLERANCE_M = 0.15
TRACKING_TOLERANCE_M = 0.10
MIN_CONFIDENCE = 0.5

STATE_ACTIVE = OffboardStatus.STATE_ACTIVE
MODE_LAND = VehicleState.FLIGHT_MODE_LAND


class Failure(Exception):
    """Raised when the flight cannot be completed; the run reports no number."""


class MarkerAccuracy(Node):
    def __init__(self, uav_id, marker_id):
        super().__init__(
            'marker_accuracy',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        prefix = '/uav/' + uav_id

        self.uav_id = uav_id
        self.marker_id = marker_id
        self.state = None
        self.offboard = None
        self.raw_odom = None
        self.truth = None
        self.target = None
        self.collecting = False
        self.observations = []      # (distance_m, confidence, truth_z)
        self.wrong_ids = set()
        self.last_frame = None
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
            Odometry, prefix + '/localization/vio_odometry_raw', self._on_truth, 20)
        self.create_subscription(
            MarkerObservation, prefix + '/perception/markers', self._on_marker, 20)
        # Lets a blind hover show the scene; see README trap 2.
        self.create_subscription(
            Image, prefix + '/perception/down/image_raw', self._on_image,
            QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.arm_client = self.create_client(Arm, prefix + '/backend/arm')
        self.disarm_client = self.create_client(Disarm, prefix + '/backend/disarm')
        self.mode_client = self.create_client(SetFlightMode, prefix + '/backend/set_mode')

        self.create_timer(0.05, self._stream)


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

    def _on_offboard(self, msg):
        self.offboard = msg

    def _on_raw(self, msg):
        self.raw_odom = msg

    def _on_truth(self, msg):
        self.truth = msg

    def _on_image(self, msg):
        self.last_frame = msg

    def save_frame(self, path):
        message = self.last_frame
        if message is None or message.encoding not in ('rgb8', 'bgr8', 'mono8'):
            return None
        channels = 1 if message.encoding == 'mono8' else 3
        array = np.frombuffer(bytes(message.data), dtype=np.uint8)
        array = array.reshape(message.height, message.width, channels)
        if message.encoding == 'bgr8':
            array = array[:, :, ::-1]
        PilImage.fromarray(array.squeeze()).save(path)
        grey = array.mean(axis=2) if channels == 3 else array[:, :, 0]
        return '%s (very dark %.1f%%)' % (path, 100 * float((grey < 60).mean()))

    def _on_marker(self, msg):
        if msg.marker_id != self.marker_id:
            self.wrong_ids.add(msg.marker_id)
            return
        if not self.collecting or self.truth is None:
            return
        distance = math.sqrt(
            msg.pose.position.x ** 2 + msg.pose.position.y ** 2 + msg.pose.position.z ** 2)
        self.observations.append(
            (distance, msg.confidence, self.truth.pose.pose.position.z))


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

    def hover_and_collect(self, origin, height, seconds):
        self.target = (origin[0], origin[1], origin[2] + height)
        self.spin(12.0)                 # climb and settle before believing anything
        self.observations = []
        self.collecting = True
        self.spin(seconds)
        self.collecting = False
        if not self.observations:
            # Not /tmp: frames vanish there (ops-playbook S2).
            os.makedirs(FRAME_DIR, exist_ok=True)
            described = self.save_frame(
                os.path.join(FRAME_DIR, 'marker_hover_%.1fm.png' % height))
            print('  no detections at %.1f m; saved what the camera saw: %s'
                  % (height, described or 'no frame available'))
        return list(self.observations)


    def fly(self, heights, seconds_each):
        self.wait_until(
            lambda: self.state is not None and self.state.connected, 60.0, 'autopilot')
        self.wait_until(lambda: self.raw_odom is not None, 30.0, 'odometry')
        self.wait_until(lambda: self.truth is not None, 30.0, 'ground truth')

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
        self.call(self.arm_client, Arm.Request(), 15.0, 'arm')

        collected = []
        for height in heights:
            collected.append(self.hover_and_collect(origin, height, seconds_each))

        request = SetFlightMode.Request()
        request.mode = MODE_LAND
        self.call(self.mode_client, request, 15.0, 'set land mode')
        self.spin(1.0)
        self.target = None
        try:
            self.wait_until(lambda: not self.state.armed, 40.0, 'automatic disarm')
        except Failure:
            self.call(self.disarm_client, Disarm.Request(), 15.0, 'disarm')

        return collected


def summarise(label, observations):
    if len(observations) < 5:
        raise Failure('%s: only %d observations' % (label, len(observations)))
    distances = [d for d, _, _ in observations]
    confidences = [c for _, c, _ in observations]
    truth_z = [z for _, _, z in observations]
    return {
        'label': label,
        'count': len(observations),
        'distance': statistics.median(distances),
        'distance_spread': max(distances) - min(distances),
        'confidence': statistics.median(confidences),
        'truth_z': statistics.median(truth_z),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--marker-id', type=int, default=7)
    parser.add_argument('--heights', type=float, nargs='+', default=[2.5, 3.5])
    parser.add_argument('--seconds', type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = MarkerAccuracy(args.uav_id, args.marker_id)
    try:
        collected = node.fly(args.heights, args.seconds)
        results = [summarise('h=%.1f' % h, obs)
                   for h, obs in zip(args.heights, collected)]
    except Failure as exc:
        print('P5.3 FAILED TO MEASURE: %s' % exc)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print()
    print('=== P5.3 marker detector, ArUco id %d ===' % args.marker_id)
    failures = []

    for result in results:
        # Model origin -> base_link -> lens -> board.
        expected = (result['truth_z'] + MODEL_ORIGIN_TO_BASE_LINK
                    - CAMERA_BELOW_BASE_LINK - MARKER_GROUND_Z)
        error = result['distance'] - expected
        print('  %-8s n=%3d  distance %.3f m  expected %.3f m  error %+.3f m  '
              'spread %.3f  confidence %.2f'
              % (result['label'], result['count'], result['distance'], expected,
                 error, result['distance_spread'], result['confidence']))
        if abs(error) > DISTANCE_TOLERANCE_M:
            failures.append('%s distance error %.3f m' % (result['label'], error))
        if result['confidence'] < MIN_CONFIDENCE:
            failures.append('%s confidence %.2f' % (result['label'], result['confidence']))

    if len(results) >= 2:
        measured_step = results[-1]['distance'] - results[0]['distance']
        true_step = results[-1]['truth_z'] - results[0]['truth_z']
        print('  tracking: height changed %.3f m, measured distance changed %.3f m'
              % (true_step, measured_step))
        if abs(measured_step - true_step) > TRACKING_TOLERANCE_M:
            failures.append('tracking error %.3f m' % (measured_step - true_step))

    if node.wrong_ids:
        print('  ⚠ also reported ids %s' % sorted(node.wrong_ids))
        failures.append('reported unexpected ids %s' % sorted(node.wrong_ids))

    node.destroy_node()
    rclpy.shutdown()

    if failures:
        print('  RESULT: FAIL (%s)' % '; '.join(failures))
        return 1
    print('  RESULT: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
