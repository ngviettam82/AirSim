#!/usr/bin/env python3
"""G-N2: three Goto legs flown on a trajectory. Run via scripts/verify_trajectory.sh.
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from uav_interfaces.action import GotoPose, Land, Takeoff
from uav_interfaces.msg import ControlCommand, Trajectory3D, VehicleState

TAKEOFF_ALTITUDE = 2.5          # m above the takeoff point
# Measured 2026-08-18: PX4 tracks a moving position setpoint with a steady lag of
# about v / 0.95, so anything above ~0.76 m/s rides the 0.8 m leash and every
# emitted acceleration then describes the clamp, not the plan. The legs therefore
# fly at a speed the aircraft can actually hold; the ceiling itself is a debt.
LEG_SPEED = 0.5                 # m/s
# The control has the opposite job: a big carrot step that the metric must catch.
CONTROL_LEG_SPEED = 1.0         # m/s
ACCEPTANCE_RADIUS = 0.3

# Two 90 degree corners, and the last leg also changes altitude.
LEGS = [(2.0, 0.0, 2.5), (2.0, 2.0, 2.5), (0.0, 2.0, 2.0)]

COMMAND_SPEED_LIMIT = 1.10      # x LEG_SPEED
VEHICLE_SPEED_LIMIT = 1.25      # x LEG_SPEED, the aircraft may overshoot the setpoint
# max_yaw_rate 0.5 rad/s at 20 Hz is 1.43 deg per setpoint; 3x leaves room for
# jitter and still fails long before the rate limit is breached.
YAW_STEP_LIMIT = 4.5            # deg between two setpoints
# Two separate questions, two limits, both derived from the emitted positions and
# never from the acceleration field, which is blind to a step at a segment join:
#   - is this the trajectory or the carrot? the carrot measures ~50 m/s2 here.
#   - is the configured max_acceleration 1.0 m/s2 actually respected?
CARROT_ACCELERATION_LIMIT = 8.0
COMMAND_ACCELERATION_LIMIT = 3.0
# G-N1 landed a goto within 0.055 m, so a metre-scale path deserves this much.
CROSS_TRACK_LIMIT = 0.3
# 20 Hz stream: fewer than this over a multi-second leg means the probe missed it.
MIN_LEG_SAMPLES = 40
# Under the 0.8 m budget: at the budget the stream is clamped, not planned.
LEASH_CLEAR_LIMIT = 0.7
STREAM_HZ_RANGE = (12.0, 26.0)
PLAN_ENDPOINT_LIMIT = 0.05      # m between the last plan point and the goal
VEHICLE_SPEED_WINDOW = 0.25     # s, long enough to outlast localization noise
TRUSTWORTHY_RTF = 0.95

WALL_CLOCK_SAFETY_FACTOR = 6.0
MIN_WALL_ALLOWANCE_SEC = 15.0


class Failure(Exception):
    """Raised when the flight cannot continue; later checks get no verdict."""


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def distance(left, right):
    return math.dist(left, right)


def point_to_segment(point, start, end):
    span = [end[i] - start[i] for i in range(3)]
    length_squared = sum(value * value for value in span)
    if length_squared < 1e-12:
        return distance(point, start)
    fraction = sum((point[i] - start[i]) * span[i] for i in range(3)) / length_squared
    fraction = max(0.0, min(1.0, fraction))
    closest = [start[i] + fraction * span[i] for i in range(3)]
    return distance(point, closest)


def cross_track(point, path):
    return min(point_to_segment(point, path[i], path[i + 1]) for i in range(len(path) - 1))


def worst(values, label):
    """max() returns the number when it meets a NaN, which is how an unmeasured
    leg turns into a passing gate. A missing measurement is a failure here."""
    if not values or any(value != value for value in values):
        raise Failure('%s has an unmeasured leg: %s' % (label, values))
    return max(values)


def percentile(values, fraction):
    if not values:
        return float('nan')
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))
    return ordered[index]


def derive_velocities(samples):
    """Velocity between consecutive emitted setpoints, paired with their mid-time.
    Vector, not speed: a turn at constant speed is acceleration too."""
    velocities = []
    for index in range(1, len(samples)):
        gap = samples[index][0] - samples[index - 1][0]
        if gap <= 1e-3:
            continue
        step = [samples[index][1][axis] - samples[index - 1][1][axis] for axis in range(3)]
        velocities.append((
            0.5 * (samples[index][0] + samples[index - 1][0]),
            [value / gap for value in step]))
    return velocities


def peak_speed(samples):
    return max((math.dist(velocity, (0.0, 0.0, 0.0))
                for _, velocity in derive_velocities(samples)), default=0.0)


def peak_acceleration(samples, max_gap):
    """A starved tick leaves a hole in the stream that reads as acceleration, so
    gaps longer than a tick and a half are skipped rather than blamed on the plan."""
    velocities = derive_velocities(samples)
    peak = 0.0
    for index in range(1, len(velocities)):
        gap = velocities[index][0] - velocities[index - 1][0]
        if gap <= 1e-3 or gap > max_gap:
            continue
        change = [velocities[index][1][axis] - velocities[index - 1][1][axis]
                  for axis in range(3)]
        peak = max(peak, math.dist(change, (0.0, 0.0, 0.0)) / gap)
    return peak


def peak_lead(commands, odometry):
    """How far the setpoint ran ahead of the aircraft: at the leash budget the
    stream is clamped, and every acceleration number then describes that regime."""
    if not commands or not odometry:
        return float('nan')
    peak = 0.0
    index = 0
    for stamp, position in odometry:
        while index + 1 < len(commands) and commands[index + 1][0] <= stamp:
            index += 1
        setpoint = commands[index][1]
        peak = max(peak, math.hypot(setpoint[0] - position[0], setpoint[1] - position[1]))
    return peak


def windowed_speed(samples, window):
    peak = 0.0
    start = 0
    for index in range(len(samples)):
        while samples[index][0] - samples[start][0] > window:
            start += 1
        span = samples[index][0] - samples[start][0]
        if span >= window * 0.5:
            peak = max(peak, distance(samples[index][1], samples[start][1]) / span)
    return peak


class TrajectoryGate(Node):
    def __init__(self, uav_id):
        super().__init__(
            'trajectory_gate',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        prefix = '/uav/' + uav_id

        self.state = None
        self.recording = False
        self.commands = []      # (sim seconds, (x, y, z), yaw)
        self.odometry = []      # (sim seconds, (x, y, z))
        self.position_now = None
        self.plans = []

        self.create_subscription(Odometry, prefix + '/state/odometry_fused', self._on_odom, 20)
        self.create_subscription(
            ControlCommand, prefix + '/control/command_selected', self._on_command, 50)
        self.create_subscription(VehicleState, prefix + '/state/vehicle', self._on_state, 10)
        # Matches the navigator: the plan in force stays available to a late reader.
        self.create_subscription(
            Trajectory3D, prefix + '/planning/trajectory',
            self._on_plan,
            QoSProfile(
                depth=5, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.takeoff_client = ActionClient(self, Takeoff, prefix + '/planning/takeoff')
        self.goto_client = ActionClient(self, GotoPose, prefix + '/planning/goto_pose')
        self.land_client = ActionClient(self, Land, prefix + '/planning/land')

    # ------------------------------------------------------------------- intake

    def _on_odom(self, message):
        point = message.pose.pose.position
        self.position_now = (point.x, point.y, point.z)
        if self.recording:
            self.odometry.append((stamp_seconds(message.header.stamp), self.position_now))

    def _on_command(self, message):
        if self.recording:
            self.commands.append((
                stamp_seconds(message.header.stamp),
                (message.position.x, message.position.y, message.position.z),
                message.yaw))

    def _on_state(self, message):
        self.state = message

    def _on_plan(self, message):
        self.plans.append(message)

    # ------------------------------------------------------------ sim-paced wait

    def sim_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wall_allowance(self, seconds):
        return max(seconds * WALL_CLOCK_SAFETY_FACTOR, MIN_WALL_ALLOWANCE_SEC)

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

    def await_server(self, client, timeout, description):
        """Spin while waiting: a blocking wait_for_server never refreshes the graph."""
        wall_deadline = time.time() + timeout
        while rclpy.ok() and time.time() < wall_deadline:
            if client.server_is_ready():
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise Failure('%s action server never appeared' % description)

    def send_goal(self, client, goal, description):
        self.await_server(client, 30.0, description)
        future = client.send_goal_async(goal)
        handle = self.wait_future(future, 15.0, '%s goal response' % description)
        if not handle.accepted:
            raise Failure('%s goal was rejected' % description)
        return handle

    def wait_result(self, handle, timeout, description):
        return self.wait_future(handle.get_result_async(), timeout, '%s result' % description)

    # ------------------------------------------------------------------- flying

    def fly_leg(self, target, description, speed=LEG_SPEED):
        goal = GotoPose.Goal()
        goal.target_pose.position.x = float(target[0])
        goal.target_pose.position.y = float(target[1])
        goal.target_pose.position.z = float(target[2])
        goal.target_pose.orientation.w = 1.0
        goal.frame_id = 'odom'
        goal.acceptance_radius = float(ACCEPTANCE_RADIUS)
        goal.max_speed = float(speed)

        plans_before = len(self.plans)
        self.commands = []
        self.odometry = []
        self.recording = True
        handle = self.send_goal(self.goto_client, goal, description)
        result = self.wait_result(handle, 180.0, description)
        self.recording = False

        return {
            'target': target,
            'status': result.status,
            'message': result.result.message,
            'commands': list(self.commands),
            'odometry': list(self.odometry),
            'plans': list(self.plans[plans_before:]),
            'reached': self.position_now,
        }


def measure(leg, max_gap):
    commands = [(entry[0], entry[1]) for entry in leg['commands']]
    yaws = [entry[2] for entry in leg['commands']]
    span = commands[-1][0] - commands[0][0] if len(commands) > 1 else 0.0

    report = {
        'command_speed': peak_speed(commands),
        'command_acceleration': peak_acceleration(commands, max_gap),
        'vehicle_speed': windowed_speed(leg['odometry'], VEHICLE_SPEED_WINDOW),
        'lead': peak_lead(commands, leg['odometry']),
        'yaw_step_deg': 0.0,
        'cross_track_p95': float('nan'),
        'cross_track_max': float('nan'),
        'samples': len(commands),
        'odometry_samples': len(leg['odometry']),
        'stream_hz': (len(commands) - 1) / span if span > 0.5 else float('nan'),
    }
    for index in range(1, len(yaws)):
        step = abs(math.atan2(math.sin(yaws[index] - yaws[index - 1]),
                              math.cos(yaws[index] - yaws[index - 1])))
        report['yaw_step_deg'] = max(report['yaw_step_deg'], math.degrees(step))

    valid = [plan for plan in leg['plans'] if plan.is_valid and len(plan.points) >= 2]
    if valid:
        path = [(point.position.x, point.position.y, point.position.z)
                for point in valid[0].points]
        errors = [cross_track(sample[1], path) for sample in leg['odometry']]
        report['cross_track_p95'] = percentile(errors, 0.95)
        report['cross_track_max'] = max(errors, default=float('nan'))
        report['plan_points'] = len(path)
        report['plan_end_error'] = distance(path[-1], leg['target'])
        report['plan_ordered'] = all(
            stamp_seconds(valid[0].points[i].time_from_start) >
            stamp_seconds(valid[0].points[i - 1].time_from_start)
            for i in range(1, len(valid[0].points)))
    report['valid_plans'] = len(valid)
    return report


def run_gate(node, record):
    node.wait_until(lambda: node.state is not None and node.state.connected, 60.0, 'autopilot')
    node.wait_until(lambda: node.position_now is not None, 60.0, 'odometry_fused')
    origin = node.position_now
    print('origin (fused) %.3f %.3f %.3f' % origin)

    takeoff = Takeoff.Goal()
    takeoff.target_altitude = float(TAKEOFF_ALTITUDE)
    handle = node.send_goal(node.takeoff_client, takeoff, 'takeoff')
    result = node.wait_result(handle, 120.0, 'takeoff')
    if result.status != GoalStatus.STATUS_SUCCEEDED:
        raise Failure('takeoff did not succeed: %s' % result.result.message)

    sim_start = node.sim_seconds()
    wall_start = time.time()
    max_gap = 1.5 / STREAM_HZ_RANGE[0]
    reports = []
    for index, target in enumerate(LEGS):
        leg = node.fly_leg(target, 'leg %d' % (index + 1))
        report = measure(leg, max_gap)
        reports.append(report)
        print('\nleg %d -> %.1f %.1f %.1f  (%d setpoints at %.1f Hz, %d odom)'
              % (index + 1, target[0], target[1], target[2], report['samples'],
                 report['stream_hz'], report['odometry_samples']))
        print('  command speed %.2f m/s | vehicle speed %.2f m/s | command accel %.2f m/s2'
              % (report['command_speed'], report['vehicle_speed'],
                 report['command_acceleration']))
        print('  cross-track p95 %.3f m max %.3f m | yaw step %.2f deg | lead %.2f m | plans %d'
              % (report['cross_track_p95'], report['cross_track_max'],
                 report['yaw_step_deg'], report['lead'], report['valid_plans']))
        if leg['status'] != GoalStatus.STATUS_SUCCEEDED:
            raise Failure('leg %d did not succeed: %s' % (index + 1, leg['message']))
        error = distance(leg['reached'], target)
        record('leg %d reached its waypoint' % (index + 1), error <= ACCEPTANCE_RADIUS + 0.1,
               'error %.3f m (limit %.2f)' % (error, ACCEPTANCE_RADIUS + 0.1))

    # Nothing below means anything until the probe proves it saw the flight.
    record('0. every leg was actually sampled',
           all(report['samples'] >= MIN_LEG_SAMPLES for report in reports)
           and all(report['odometry_samples'] >= 10 for report in reports)
           and all(STREAM_HZ_RANGE[0] <= report['stream_hz'] <= STREAM_HZ_RANGE[1]
                   for report in reports),
           'setpoints %s at %s Hz, odom %s'
           % ([report['samples'] for report in reports],
              ['%.1f' % report['stream_hz'] for report in reports],
              [report['odometry_samples'] for report in reports]))

    rtf = (node.sim_seconds() - sim_start) / max(1e-6, time.time() - wall_start)
    print('\npeak setpoint lead: %.3f m (leash budget 0.80 m)'
          % worst([report['lead'] for report in reports], 'setpoint lead'))
    print('RTF while flying the legs: %.3f%s'
          % (rtf, '' if rtf >= TRUSTWORTHY_RTF else '  <-- BELOW 0.95, numbers are labelled'))

    worst_command = worst([report['command_speed'] for report in reports], 'command speed')
    worst_vehicle = worst([report['vehicle_speed'] for report in reports], 'vehicle speed')
    worst_accel = worst([report['command_acceleration'] for report in reports], 'acceleration')
    worst_yaw = worst([report['yaw_step_deg'] for report in reports], 'yaw step')
    worst_p95 = worst([report['cross_track_p95'] for report in reports], 'cross-track p95')
    worst_track = worst([report['cross_track_max'] for report in reports], 'cross-track max')
    worst_lead = worst([report['lead'] for report in reports], 'setpoint lead')

    record('a. the setpoint stream stayed under the goal speed',
           worst_command <= COMMAND_SPEED_LIMIT * LEG_SPEED,
           '%.3f m/s (limit %.2f)' % (worst_command, COMMAND_SPEED_LIMIT * LEG_SPEED))
    record('b. the aircraft stayed under the goal speed',
           worst_vehicle <= VEHICLE_SPEED_LIMIT * LEG_SPEED,
           '%.3f m/s (limit %.2f)' % (worst_vehicle, VEHICLE_SPEED_LIMIT * LEG_SPEED))
    # The leash bounds position, not acceleration: clamping steps the stream by
    # design (measured at 48.6 m/s2 in the unit rig), so every acceleration number
    # below only means something once the flight is shown to have stayed off it.
    record('c. the setpoint never hit the leash', worst_lead <= LEASH_CLEAR_LIMIT,
           'peak lead %.3f m (budget 0.80, limit %.2f)' % (worst_lead, LEASH_CLEAR_LIMIT))
    record('c2. the emitted stream is a trajectory, not a carrot step',
           worst_accel <= CARROT_ACCELERATION_LIMIT,
           '%.2f m/s2 (limit %.1f, the carrot measures ~50)'
           % (worst_accel, CARROT_ACCELERATION_LIMIT))
    record('c3. the emitted stream respects the acceleration envelope',
           worst_accel <= COMMAND_ACCELERATION_LIMIT,
           '%.2f m/s2 (limit %.1f for max_acceleration 1.0)'
           % (worst_accel, COMMAND_ACCELERATION_LIMIT))
    record('d. yaw slewed instead of jumping', worst_yaw <= YAW_STEP_LIMIT,
           '%.2f deg per setpoint (limit %.0f)' % (worst_yaw, YAW_STEP_LIMIT))
    record('e. the aircraft stayed on the published path',
           worst_p95 <= CROSS_TRACK_LIMIT and worst_track <= CROSS_TRACK_LIMIT * 2.0,
           'p95 %.3f m, max %.3f m (limits %.2f / %.2f)'
           % (worst_p95, worst_track, CROSS_TRACK_LIMIT, CROSS_TRACK_LIMIT * 2.0))
    record('f. every leg published exactly one plan',
           all(report['valid_plans'] == 1 for report in reports),
           'plans per leg %s' % [report['valid_plans'] for report in reports])
    record('g. plans are ordered and end on the goal',
           all(report.get('plan_ordered') for report in reports)
           and all(report.get('plan_end_error', 9.9) <= PLAN_ENDPOINT_LIMIT
                   for report in reports),
           'endpoint error %s m' % ['%.3f' % report.get('plan_end_error', float('nan'))
                                    for report in reports])
    # Attributed globally and waited for: the retire is published before the goal
    # result, but DDS can deliver it after, and takeoff retires a plan too.
    node.wait_until(lambda: bool(node.plans) and not node.plans[-1].is_valid, 15.0,
                    'the last plan to be retired')
    published = [plan.is_valid for plan in node.plans]
    # Counted in ERAS, not in messages (changed 2026-08-20): one task may publish
    # many plans - a carrot leg republishes every carrot_plan_period_sec, and an
    # escape rebuilds mid-leg - but each era must still end in a retire. Comparing
    # plan count against retire count stopped measuring anything the day a task was
    # allowed to publish more than one plan.
    eras = sum(1 for index, live in enumerate(published)
               if live and (index == 0 or not published[index - 1]))
    record('h. every plan era is closed by a retire',
           published.count(False) >= eras and not published[-1],
           '%d plans in %d eras, %d retires'
           % (published.count(True), eras, published.count(False)))

    land = Land.Goal()
    land.use_precision_landing = False
    handle = node.send_goal(node.land_client, land, 'land')
    result = node.wait_result(handle, 120.0, 'land')
    node.wait_until(lambda: node.state is not None and not node.state.armed, 60.0, 'disarm')
    record('i. land disarmed the aircraft', result.status == GoalStatus.STATUS_SUCCEEDED,
           'status=%d' % result.status)
    return worst_accel


def run_control(node):
    """The same measurement on the carrot path, to prove the metric can fail."""
    node.wait_until(lambda: node.state is not None and node.state.connected, 60.0, 'autopilot')
    node.wait_until(lambda: node.position_now is not None, 60.0, 'odometry_fused')

    takeoff = Takeoff.Goal()
    takeoff.target_altitude = float(TAKEOFF_ALTITUDE)
    handle = node.send_goal(node.takeoff_client, takeoff, 'takeoff')
    if node.wait_result(handle, 120.0, 'takeoff').status != GoalStatus.STATUS_SUCCEEDED:
        raise Failure('takeoff did not succeed')

    leg = node.fly_leg(LEGS[0], 'control leg', CONTROL_LEG_SPEED)
    report = measure(leg, 1.5 / STREAM_HZ_RANGE[0])
    print('control leg (use_trajectory:=false): command accel %.2f m/s2, speed %.2f m/s, '
          '%d setpoints, valid plans %d'
          % (report['command_acceleration'], report['command_speed'], report['samples'],
             report['valid_plans']))
    if report['samples'] < MIN_LEG_SAMPLES:
        raise Failure('the control leg was not sampled: %d setpoints' % report['samples'])

    land = Land.Goal()
    land.use_precision_landing = False
    handle = node.send_goal(node.land_client, land, 'land')
    node.wait_result(handle, 120.0, 'land')
    return report['command_acceleration']


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--mode', choices=['gate', 'control'], default='gate')
    args = parser.parse_args()

    rclpy.init()
    node = TrajectoryGate(args.uav_id)
    checks = []
    failure = None
    measured = float('nan')

    def record(name, passed, detail):
        checks.append((name, bool(passed), detail))

    try:
        if args.mode == 'control':
            measured = run_control(node)
        else:
            measured = run_gate(node, record)
    except Failure as exc:
        failure = str(exc)
    except KeyboardInterrupt:
        failure = 'interrupted'

    print()
    if args.mode == 'control':
        print('=== G-N2 positive control ===')
        print('  carrot command acceleration: %.2f m/s2' % measured)
        passed = failure is None and measured > CARROT_ACCELERATION_LIMIT
        if not passed and failure is None:
            print('  the control did NOT exceed %.1f m/s2, so check c2 proves nothing'
                  % CARROT_ACCELERATION_LIMIT)
    else:
        print('=== G-N2 trajectory gate ===')
        for name, ok, detail in checks:
            print('  [%s] %-44s %s' % ('PASS' if ok else 'FAIL', name, detail))
        expected_checks = 3 + 12
        passed = (failure is None and len(checks) == expected_checks
                  and all(entry[1] for entry in checks))
    if failure:
        print('  FAILURE: %s' % failure)
    print('  RESULT: %s' % ('PASS' if passed else 'FAIL'))
    sys.stdout.flush()

    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
