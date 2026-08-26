#!/usr/bin/env python3
"""G-M0: P9.5 sim assets - moving target_box and marker-at-hover via the
navigator. Run via scripts/gate_gm0_mission_assets.sh (spawns the box/marker
first). NOT RUN by the author - sim access is reserved for the verifier-runner."""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from uav_interfaces.action import Land, Takeoff
from uav_interfaces.msg import MarkerObservation

# ---------------------------------------------------------------- phase: target-box
# Pure Gazebo measurement, no ROS: displacement over SIM time, read from the
# world's own pose stream (README pitfall 10 documents why the box needs a
# ground clearance + gravity off, and why polling `gz model -p` in a shell
# loop is NOT trustworthy for this - subprocess overhead alone inflated a
# measured 0.2 m/s command to ~0.27 m/s in that shell-loop prototype).
SPEED_TOLERANCE_FRACTION = 0.15
STEP_JUMP_TOLERANCE_FRACTION = 0.5   # vs the run's own mean step speed
MIN_SAMPLES = 50


class Failure(Exception):
    """Raised when the measurement itself cannot be trusted; no verdict follows."""


def run_target_box(args):
    import gz.transport13 as gz_transport
    from gz.msgs10.pose_v_pb2 import Pose_V

    samples = []

    def callback(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for pose in msg.pose:
            if pose.name == args.name:
                samples.append((stamp, pose.position.x, pose.position.y))
                return

    node = gz_transport.Node()
    topic = '/world/%s/dynamic_pose/info' % args.world
    if not node.subscribe(Pose_V, topic, callback):
        raise Failure('could not subscribe to %s' % topic)

    # R27-1: prove the measurement is alive before judging anything.
    deadline = time.time() + 15.0
    while not samples and time.time() < deadline:
        time.sleep(0.2)
    if not samples:
        raise Failure(
            '%s never appeared on %s within 15s - is it spawned?' % (args.name, topic))

    deadline = time.time() + args.duration
    while time.time() < deadline:
        time.sleep(0.2)

    if len(samples) < MIN_SAMPLES:
        raise Failure('only %d pose samples collected (need >= %d)'
                       % (len(samples), MIN_SAMPLES))

    step_speeds = []
    path_length = 0.0
    for (ta, xa, ya), (tb, xb, yb) in zip(samples, samples[1:]):
        dt = tb - ta
        if dt <= 0:
            raise Failure('non-increasing sim timestamps in pose samples')
        step_dist = math.hypot(xb - xa, yb - ya)
        path_length += step_dist
        step_speeds.append(step_dist / dt)

    mean_step_speed = sum(step_speeds) / len(step_speeds)
    worst_deviation = max(abs(s - mean_step_speed) for s in step_speeds)

    checks = []
    checks.append((
        'a1. path length >= 8 m',
        path_length >= 8.0,
        'path length %.3f m over %d samples, %.2f sim-s'
        % (path_length, len(samples), samples[-1][0] - samples[0][0])))

    speed_error = abs(mean_step_speed - args.speed) / args.speed
    checks.append((
        'a2. mean speed within 15%% of commanded %.3f m/s' % args.speed,
        speed_error <= SPEED_TOLERANCE_FRACTION,
        'mean step speed %.4f m/s (%.1f%% error)' % (mean_step_speed, 100 * speed_error)))

    checks.append((
        'a3. continuous motion (no step/teleport)',
        worst_deviation <= mean_step_speed * STEP_JUMP_TOLERANCE_FRACTION,
        'worst single-step deviation from mean %.4f m/s (cap %.4f m/s = 50%% of mean)'
        % (worst_deviation, mean_step_speed * STEP_JUMP_TOLERANCE_FRACTION)))

    return checks


# ---------------------------------------------------------------- phase: marker
# Reuses the exact distance formula proven in uav_perception/test/marker_accuracy.py
# (P5.3, DISTANCE_TOLERANCE_M=0.15) - the constants are physical mount offsets,
# not logic worth re-deriving. Flies via the navigator's Takeoff/Land actions
# (not marker_accuracy.py's raw ControlCommand test-source stream) because that
# is the mechanism P9 missions actually use.
CAMERA_BELOW_BASE_LINK = 0.065   # sensor_camera_down mount on uav0_nav
MODEL_ORIGIN_TO_BASE_LINK = 0.240
MARKER_GROUND_Z = 0.01
DISTANCE_TOLERANCE_M = 0.15
MIN_OBSERVATIONS = 5


class MarkerHoverGate(Node):
    def __init__(self, uav_id, marker_id):
        super().__init__(
            'gm0_assets_gate',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        prefix = '/uav/' + uav_id
        self.marker_id = marker_id
        self.truth = None
        self.collecting = False
        self.observations = []      # (distance_m, truth_z)
        self.wrong_ids = set()

        self.create_subscription(
            Odometry, prefix + '/localization/vio_odometry_raw', self._on_truth, 20)
        self.create_subscription(
            MarkerObservation, prefix + '/perception/markers', self._on_marker, 20)

        self.takeoff_client = ActionClient(self, Takeoff, prefix + '/planning/takeoff')
        self.land_client = ActionClient(self, Land, prefix + '/planning/land')

    def _on_truth(self, msg):
        self.truth = msg

    def _on_marker(self, msg):
        if msg.marker_id != self.marker_id:
            self.wrong_ids.add(msg.marker_id)
            return
        if not self.collecting or self.truth is None:
            return
        distance = math.sqrt(
            msg.pose.position.x ** 2 + msg.pose.position.y ** 2 + msg.pose.position.z ** 2)
        self.observations.append((distance, self.truth.pose.pose.position.z))

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

    def await_server(self, client, timeout, description):
        """Spin while waiting: a blocking wait_for_server never refreshes the graph."""
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            if client.server_is_ready():
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise Failure('%s action server never appeared' % description)

    def send_goal(self, client, goal, description):
        self.await_server(client, 30.0, description)
        future = client.send_goal_async(goal)
        deadline = time.time() + 15.0
        while rclpy.ok() and not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise Failure('%s goal response timed out' % description)
        handle = future.result()
        if not handle.accepted:
            raise Failure('%s goal was rejected' % description)
        return handle

    def wait_result(self, handle, timeout, description):
        future = handle.get_result_async()
        deadline = time.time() + timeout
        while rclpy.ok() and not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise Failure('%s result timed out' % description)
        return future.result()


def run_marker(args):
    node = MarkerHoverGate(args.uav_id, args.marker_id)
    try:
        node.wait_until(lambda: node.truth is not None, 30.0, 'ground truth odometry')

        takeoff = Takeoff.Goal()
        takeoff.target_altitude = float(args.height)
        handle = node.send_goal(node.takeoff_client, takeoff, 'takeoff')
        result = node.wait_result(handle, 120.0, 'takeoff')
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise Failure('takeoff did not succeed: %s' % result.result.message)

        node.spin(8.0)          # settle before believing anything
        node.observations = []
        node.collecting = True
        node.spin(args.seconds)
        node.collecting = False

        checks = []
        if len(node.observations) < MIN_OBSERVATIONS:
            raise Failure('only %d marker observations (need >= %d)'
                           % (len(node.observations), MIN_OBSERVATIONS))

        distances = sorted(d for d, _ in node.observations)
        truth_zs = sorted(z for _, z in node.observations)
        median_distance = distances[len(distances) // 2]
        median_truth_z = truth_zs[len(truth_zs) // 2]
        expected = median_truth_z + MODEL_ORIGIN_TO_BASE_LINK - CAMERA_BELOW_BASE_LINK \
            - MARKER_GROUND_Z
        error = median_distance - expected

        checks.append((
            'b1. marker id %d detected' % args.marker_id,
            len(node.observations) >= MIN_OBSERVATIONS,
            '%d observations' % len(node.observations)))
        checks.append((
            'b2. pose error <= 0.15 m at hover height',
            abs(error) <= DISTANCE_TOLERANCE_M,
            'distance %.3f m, expected %.3f m, error %+.3f m (n=%d)'
            % (median_distance, expected, error, len(node.observations))))
        checks.append((
            'b3. no other marker id reported',
            not node.wrong_ids,
            'unexpected ids: %s' % sorted(node.wrong_ids)))

        land = Land.Goal()
        land.use_precision_landing = False
        handle = node.send_goal(node.land_client, land, 'land')
        result = node.wait_result(handle, 120.0, 'land')
        checks.append((
            'b4. land succeeded',
            result.status == GoalStatus.STATUS_SUCCEEDED,
            'status=%d' % result.status))

        return checks
    finally:
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='phase', required=True)

    box = sub.add_parser('target-box')
    box.add_argument('--world', default='uav_arena')
    box.add_argument('--name', default='gm0_target_box')
    box.add_argument('--speed', type=float, default=0.2)
    box.add_argument('--duration', type=float, default=45.0)

    marker = sub.add_parser('marker')
    marker.add_argument('--uav-id', default='uav0')
    marker.add_argument('--marker-id', type=int, default=7)
    marker.add_argument('--height', type=float, default=2.5)
    marker.add_argument('--seconds', type=float, default=15.0)

    args = parser.parse_args()

    checks = None
    failure = None
    if args.phase == 'target-box':
        try:
            checks = run_target_box(args)
        except Failure as exc:
            failure = str(exc)
    else:
        rclpy.init()
        try:
            checks = run_marker(args)
        except Failure as exc:
            failure = str(exc)
        finally:
            rclpy.shutdown()

    print()
    print('=== G-M0 %s ===' % args.phase)
    if failure:
        print('  FAILURE (measurement incomplete): %s' % failure)
        print('  RESULT: FAILED TO MEASURE')
        sys.stdout.flush()
        return 2

    for name, ok, detail in checks:
        print('  [%s] %-50s %s' % ('PASS' if ok else 'FAIL', name, detail))
    passed = all(entry[1] for entry in checks)
    print('  RESULT: %s' % ('PASS' if passed else 'FAIL'))
    sys.stdout.flush()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
