#!/usr/bin/env python3
"""G-N1: M5 flown through the navigator actions. Run via scripts/verify_navigator.sh.
NOT RUN by the author - sim access is reserved for the verifier."""

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
from uav_interfaces.action import GotoPose, Land, Takeoff
from uav_interfaces.msg import VehicleState

TAKEOFF_ALTITUDE = 2.5          # m above the takeoff point
ALTITUDE_TOLERANCE = 0.3        # M5 threshold
HORIZONTAL_TOLERANCE = 0.5      # M5 threshold
ACCEPTANCE_RADIUS = 0.3

GOTO_TARGET = (1.5, 1.0, 2.5)   # odom, absolute
CANCEL_TARGET = (-2.0, -1.5, 2.5)
CANCEL_SPEED = 0.5              # slow enough that the cancel lands mid-flight
FLY_BEFORE_CANCEL = 4.0         # s

SETTLE_AFTER_CANCEL = 2.0       # s of braking that is not drift
HOVER_WATCH_SECONDS = 5.0
HOVER_DRIFT_LIMIT = 0.3         # m

LAND_DISARM_LIMIT = 40.0        # s

# How much wall time a sim-time wait may consume before the run is unusable.
WALL_CLOCK_SAFETY_FACTOR = 6.0

# Floor for the anti-hang valve; 6x a 0.2 s wait is one stall away from failing.
MIN_WALL_ALLOWANCE_SEC = 15.0


class Failure(Exception):
    """Raised when the flight cannot continue; the run reports no verdict for later checks."""


class NavigatorGate(Node):
    def __init__(self, uav_id):
        super().__init__(
            'navigator_gate',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        prefix = '/uav/' + uav_id

        self.uav_id = uav_id
        self.fused = None
        self.raw = None
        self.state = None
        self.feedback_distances = []

        self.create_subscription(
            Odometry, prefix + '/state/odometry_fused', self._on_fused, 10)
        self.create_subscription(
            Odometry, prefix + '/state/odometry_raw', self._on_raw, 10)
        self.create_subscription(
            VehicleState, prefix + '/state/vehicle', self._on_state, 10)

        self.takeoff_client = ActionClient(self, Takeoff, prefix + '/planning/takeoff')
        self.goto_client = ActionClient(self, GotoPose, prefix + '/planning/goto_pose')
        self.land_client = ActionClient(self, Land, prefix + '/planning/land')

    def _on_fused(self, message):
        self.fused = message

    def _on_raw(self, message):
        self.raw = message

    def _on_state(self, message):
        self.state = message

    # ------------------------------------------------------------- sim-paced waiting

    def sim_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wall_allowance(self, seconds):
        """A single sim stall eats a short wait entirely, so keep a floor."""
        return max(seconds * WALL_CLOCK_SAFETY_FACTOR, MIN_WALL_ALLOWANCE_SEC)

    def spin(self, seconds):
        """Measured on /clock: a wall-clock wait shrinks with the real-time factor."""
        start_sim = self.sim_seconds()
        wall_deadline = time.time() + self.wall_allowance(seconds)
        while rclpy.ok() and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.sim_seconds() - start_sim >= seconds:
                return
        # Re-check: the loop can exit on the wall clock in the same tick the sim
        # reached the target, which used to report "advanced 0.2 s of 0.2 s".
        if self.sim_seconds() - start_sim >= seconds:
            return
        raise Failure('sim clock advanced %.3f s of %.3f s before the wall clock ran out'
                      % (self.sim_seconds() - start_sim, seconds))

    def wait_until(self, predicate, timeout, description):
        wall_deadline = time.time() + self.wall_allowance(timeout)
        while rclpy.ok() and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        if predicate():
            return
        raise Failure('timed out waiting for %s' % description)

    def wait_future(self, future, timeout, description):
        wall_deadline = time.time() + self.wall_allowance(timeout)
        while rclpy.ok() and not future.done() and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise Failure('timed out waiting for %s' % description)
        return future.result()

    # ---------------------------------------------------------------- action plumbing

    def await_server(self, client, timeout, description):
        """Spin while waiting: the graph cache only refreshes on a spun node,
        so the blocking wait_for_server() never sees the server appear."""
        wall_deadline = time.time() + timeout
        while rclpy.ok() and time.time() < wall_deadline:
            if client.server_is_ready():
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise Failure('%s action server never appeared' % description)

    def send_goal(self, client, goal, description, feedback_callback=None):
        self.await_server(client, 30.0, description)
        future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        return self.wait_future(future, 15.0, '%s goal response' % description)

    def wait_result(self, handle, timeout, description):
        future = handle.get_result_async()
        return self.wait_future(future, timeout, '%s result' % description)

    # ------------------------------------------------------------------- measurement

    def position(self):
        if self.fused is None:
            return None
        point = self.fused.pose.pose.position
        return (point.x, point.y, point.z)

    def source_offset(self):
        if self.fused is None or self.raw is None:
            return None
        fused = self.fused.pose.pose.position
        raw = self.raw.pose.pose.position
        return (fused.x - raw.x, fused.y - raw.y, fused.z - raw.z)


def goto_goal(target, speed=0.0, frame_id='odom'):
    goal = GotoPose.Goal()
    goal.target_pose.position.x = float(target[0])
    goal.target_pose.position.y = float(target[1])
    goal.target_pose.position.z = float(target[2])
    goal.target_pose.orientation.w = 1.0
    goal.frame_id = frame_id
    goal.acceptance_radius = float(ACCEPTANCE_RADIUS)
    goal.max_speed = float(speed)
    return goal


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    args = parser.parse_args()

    rclpy.init()
    node = NavigatorGate(args.uav_id)
    checks = []
    failure = None

    def record(name, passed, detail):
        checks.append((name, bool(passed), detail))

    try:
        node.wait_until(
            lambda: node.state is not None and node.state.connected, 60.0, 'autopilot connection')
        node.wait_until(lambda: node.fused is not None, 60.0, 'odometry_fused')
        origin = node.position()
        print('origin (fused) %.3f %.3f %.3f' % origin)
        offset = node.source_offset()
        if offset is not None:
            print('fused - raw    %+.3f %+.3f %+.3f m' % offset)

        # (a) a goto before takeoff has no aircraft to fly.
        handle = node.send_goal(node.goto_client, goto_goal(GOTO_TARGET), 'goto (grounded)')
        record('a. goto rejected before takeoff', not handle.accepted,
               'accepted=%s' % handle.accepted)

        # (c) takeoff.
        takeoff = Takeoff.Goal()
        takeoff.target_altitude = float(TAKEOFF_ALTITUDE)
        handle = node.send_goal(node.takeoff_client, takeoff, 'takeoff')
        if not handle.accepted:
            raise Failure('takeoff goal was rejected')
        result = node.wait_result(handle, 90.0, 'takeoff')
        cruise_z = origin[2] + TAKEOFF_ALTITUDE
        altitude_error = abs(node.position()[2] - cruise_z)
        record('c. takeoff reached %.2f m' % TAKEOFF_ALTITUDE,
               result.status == GoalStatus.STATUS_SUCCEEDED
               and altitude_error <= ALTITUDE_TOLERANCE,
               'status=%d altitude_error=%.3f m (limit %.2f)'
               % (result.status, altitude_error, ALTITUDE_TOLERANCE))

        # (b) the same goal in a frame that does not exist yet.
        handle = node.send_goal(
            node.goto_client, goto_goal(GOTO_TARGET, frame_id='map'), 'goto (map frame)')
        record('b. goto rejected in frame "map"', not handle.accepted,
               'accepted=%s' % handle.accepted)

        # (d) goto with feedback.
        node.feedback_distances = []

        def on_feedback(message):
            node.feedback_distances.append(message.feedback.distance_remaining)

        handle = node.send_goal(
            node.goto_client, goto_goal(GOTO_TARGET), 'goto', feedback_callback=on_feedback)
        if not handle.accepted:
            raise Failure('goto goal was rejected while hovering')
        result = node.wait_result(handle, 120.0, 'goto')
        reached = node.position()
        horizontal_error = math.dist(reached[:2], GOTO_TARGET[:2])
        distance_error = math.dist(reached, GOTO_TARGET)
        record('d. goto reached the waypoint',
               result.status == GoalStatus.STATUS_SUCCEEDED
               and distance_error <= ACCEPTANCE_RADIUS + 0.1
               and horizontal_error <= HORIZONTAL_TOLERANCE,
               'status=%d distance=%.3f m horizontal=%.3f m'
               % (result.status, distance_error, horizontal_error))

        samples = node.feedback_distances
        record('d. feedback distance shrank',
               len(samples) >= 3 and samples[-1] < samples[0],
               '%d samples, first %.2f m, last %.2f m'
               % (len(samples), samples[0] if samples else float('nan'),
                  samples[-1] if samples else float('nan')))

        # (e) and (f) share one flight: reject a second goal, then cancel the first.
        flying = node.send_goal(
            node.goto_client, goto_goal(CANCEL_TARGET, speed=CANCEL_SPEED), 'goto (long)')
        if not flying.accepted:
            raise Failure('the second goto was rejected while hovering')
        node.spin(1.0)
        intruder = node.send_goal(node.goto_client, goto_goal(GOTO_TARGET), 'goto (second)')
        record('e. second goal rejected while flying', not intruder.accepted,
               'accepted=%s' % intruder.accepted)

        node.spin(FLY_BEFORE_CANCEL)
        cancel_future = flying.cancel_goal_async()
        node.wait_future(cancel_future, 15.0, 'cancel response')
        result = node.wait_result(flying, 30.0, 'canceled goto')

        node.spin(SETTLE_AFTER_CANCEL)
        anchor = node.position()
        drift = 0.0
        watch_start = node.sim_seconds()
        while node.sim_seconds() - watch_start < HOVER_WATCH_SECONDS:
            node.spin(0.25)
            drift = max(drift, math.dist(node.position(), anchor))
        record('f. hover holds after cancel',
               result.status == GoalStatus.STATUS_CANCELED and drift <= HOVER_DRIFT_LIMIT,
               'status=%d drift=%.3f m over %.0f s (limit %.2f)'
               % (result.status, drift, HOVER_WATCH_SECONDS, HOVER_DRIFT_LIMIT))

        # (g) land.
        land = Land.Goal()
        land.use_precision_landing = False
        handle = node.send_goal(node.land_client, land, 'land')
        if not handle.accepted:
            raise Failure('land goal was rejected')
        land_start = node.sim_seconds()
        result = node.wait_result(handle, 90.0, 'land')
        node.wait_until(lambda: node.state is not None and not node.state.armed,
                        LAND_DISARM_LIMIT, 'disarm')
        land_seconds = node.sim_seconds() - land_start
        record('g. land disarmed the aircraft',
               result.status == GoalStatus.STATUS_SUCCEEDED
               and land_seconds <= LAND_DISARM_LIMIT,
               'status=%d disarmed after %.1f s (limit %.0f)'
               % (result.status, land_seconds, LAND_DISARM_LIMIT))

    except Failure as exc:
        failure = str(exc)
    except KeyboardInterrupt:
        failure = 'interrupted'

    print()
    print('=== G-N1 navigator action gate ===')
    for name, passed, detail in checks:
        print('  [%s] %-38s %s' % ('PASS' if passed else 'FAIL', name, detail))
    if failure:
        print('  FAILURE: %s' % failure)

    expected_checks = 8
    passed = (failure is None and len(checks) == expected_checks
              and all(entry[1] for entry in checks))
    print('  RESULT: %s' % ('PASS' if passed else 'FAIL'))
    sys.stdout.flush()

    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
