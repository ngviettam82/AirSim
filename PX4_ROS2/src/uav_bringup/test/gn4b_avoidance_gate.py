#!/usr/bin/env python3
"""G-N4b: uav0_track flies a Goto leg; the runner spawns a real box mid-flight
and this probe feeds the matching ObstacleArray. Run via scripts/run_gn4b.sh.
NOT RUN by the author - sim access is reserved for the verifier-runner."""

import argparse
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from uav_interfaces.action import GotoPose, Land, Takeoff
from uav_interfaces.msg import (
    AvoidanceAdvice, ControlCommand, Obstacle, ObstacleArray, Trajectory3D, VehicleState)

TAKEOFF_ALTITUDE = 2.5          # m above the takeoff point, matches the other flight gates
LEG_LENGTH = 8.0                # m, comfortably above the "at least 6 m" requirement
LEG_SPEED = 0.5                 # m/s, per the D2 brief
ACCEPTANCE_RADIUS = 0.3         # m, the navigator's shipped default (navigation_params.yaml)

# Full extent (contract 2.7), centred on the cruise altitude so it sits inside
# the +-flight_band_m (1.0 m, navigation_params.yaml) slice the costmap actually
# checks; wide enough (2.0 m) to force a real detour, not a knife-edge miss.
OBSTACLE_SIZE = (2.0, 2.0, 3.0)
OBSTACLE_ID = 1
SENSING_RANGE = 10.0            # m, plausible synthetic sensor range
# Contract 2.12: -1 is banned on /world/* (must always carry at least the
# localization contribution). 0.05 m matches the TargetState precedent value.
OBSTACLE_UNCERTAINTY = 0.05
OBSTACLE_PUBLISH_HZ = 5.0

# max_acceleration is 1.0 m/s^2 (navigation_params.yaml); x3 mirrors trajectory_gate's c3.
ACCEL_ENVELOPE_LIMIT = 3.0
NO_DESCENT_TOLERANCE = 0.2      # m, per the D2 brief

STREAM_HZ_RANGE = (12.0, 26.0)  # same tolerance band as trajectory_gate.py
MIN_LEG_SAMPLES = 40
TRUSTWORTHY_RTF = 0.95

SPAWN_CONFIRM_TIMEOUT = 45.0    # s wall clock, waiting on the runner's gz service call
TRAJECTORY_ESTABLISHED_TIMEOUT = 20.0
CONSUMPTION_EVIDENCE_TIMEOUT = 25.0   # advice_timeout_sec 1.0 @ advise_hz 10 -> generous margin
REPLAN_WINDOW_SEC = 1.5         # half-window around a sequence change for the accel check

GT_ODOM_SUFFIX = '/localization/vio_odometry_raw'   # ground truth on *_track/*_nav models

WALL_CLOCK_SAFETY_FACTOR = 6.0
MIN_WALL_ALLOWANCE_SEC = 15.0
EXPECTED_CHECKS = 7             # a silently vanished check must not read as PASS


class Failure(Exception):
    """Raised when the measurement itself cannot be trusted; no verdict follows."""


def assert_single_writer(node, topic, expected, discovery_probe_topic, timeout_sec=15.0,
                          hold_sec=2.5, sample_period_sec=0.5):
    """rclpy twin of scripts/gate_common.sh's assert_single_writer -- a
    DETECTION net, not prevention: this node's own create_publisher() call on
    `topic` already counts as one writer, so `expected` is 1 here, not 0.
    A count above that means something else (perception:=true left on, most
    likely) is already publishing ObstacleArray on our output topic; fixing
    that at the source (the launch command) is still mandatory, this only
    catches the symptom before we add a confusing second stream to it.

    N6 fix (P10-gate-debt review round 2, 2026-08-24): reading
    count_publishers(topic) ONCE right after node construction was a false
    PASS -- OUR OWN publisher (created in __init__) already satisfies
    `expected` before ROS graph discovery in THIS process has had any
    chance to see a REMOTE double-writer, which typically takes ~0.5s+ to
    appear (mirrors gate_common.sh's own N5 fix). Two changes: (1)
    `discovery_probe_topic` -- a topic KNOWN to already have >=1 remote
    publisher right now (state/vehicle from px4_state_adapter_node, always
    up regardless of perception/mission flags) -- must itself be SEEN
    before ANY reading of `topic` is trusted; discovery not proven to
    converge within timeout_sec raises Failure (FAILED TO MEASURE), never
    PASS. (2) `topic`'s own count must then STAY == expected for a full
    hold_sec window (sampled every sample_period_sec), not just match on a
    single early read."""
    probe_deadline = time.time() + timeout_sec
    probe_count = node.count_publishers(discovery_probe_topic)
    while probe_count < 1 and time.time() < probe_deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        probe_count = node.count_publishers(discovery_probe_topic)
    if probe_count < 1:
        raise Failure(
            'discovery probe on %s never saw a publisher within %.1fs -- cannot trust '
            'any publisher count read on %s before graph discovery is proven to have '
            'converged' % (discovery_probe_topic, timeout_sec, topic))

    hold_deadline = time.time() + hold_sec
    while time.time() < hold_deadline:
        rclpy.spin_once(node, timeout_sec=sample_period_sec)
        count = node.count_publishers(topic)
        if count != expected:
            raise Failure(
                '%s has %d publisher(s), expected %d (this probe counts as %d) -- '
                'refusing to become an uncoordinated second writer on our own '
                'output topic; check perception:=false on the launch command'
                % (topic, count, expected, 1))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def worst(values, label):
    """max() silently passes a NaN through; a missing sample must fail loudly."""
    if not values or any(value != value for value in values):
        raise Failure('%s has an unmeasured sample: %s' % (label, values))
    return max(values)


def safe_min(values, label):
    return -worst([-value for value in values], label)


def point_to_aabb_distance(point, box_min, box_max):
    """0 when point is inside the box (i.e. a collision); positive clearance otherwise."""
    gaps = [max(box_min[i] - point[i], 0.0, point[i] - box_max[i]) for i in range(3)]
    return math.sqrt(sum(g * g for g in gaps))


def derive_velocities(samples):
    """Same technique as uav_navigation/test/trajectory_gate.py: velocity between
    consecutive emitted setpoints, paired with their mid-time."""
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


def peak_acceleration(samples, max_gap):
    """Gaps longer than a tick and a half are skipped: a starved tick reads as a
    huge acceleration that belongs to the stream, not the plan."""
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


class AvoidanceGate(Node):
    def __init__(self, uav_id, request_file, spawned_flag):
        super().__init__(
            'gn4b_avoidance_gate',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        prefix = '/uav/' + uav_id
        self.uav_id = uav_id
        self.request_file = request_file
        self.spawned_flag = spawned_flag

        self.state = None
        self.position_now = None       # odometry_fused, m (ENU)
        self.gt_position_now = None    # ground truth, m (ENU)
        self.latest_trajectory = None
        self.latest_advice = None

        self.recording = False
        self.commands = []             # (sim seconds, (x, y, z))
        self.gt_samples = []           # (sim seconds, (x, y, z))
        self.trajectory_log = []       # (sim seconds, sequence, is_valid)
        self.advice_log = []           # (sim seconds, AvoidanceAdvice)

        self.obstacle_topic = prefix + '/world/obstacle_map_local'
        self.obstacle_publisher = self.create_publisher(
            ObstacleArray, self.obstacle_topic, qos_profile_sensor_data)

        self.create_subscription(Odometry, prefix + '/state/odometry_fused', self._on_odom, 20)
        self.create_subscription(Odometry, prefix + GT_ODOM_SUFFIX, self._on_gt, 50)
        self.create_subscription(
            ControlCommand, prefix + '/control/command_selected', self._on_command, 50)
        self.create_subscription(VehicleState, prefix + '/state/vehicle', self._on_state, 10)
        self.create_subscription(
            Trajectory3D, prefix + '/planning/trajectory', self._on_trajectory,
            QoSProfile(
                depth=5, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            AvoidanceAdvice, prefix + '/planning/avoidance', self._on_advice, 10)

        self.takeoff_client = ActionClient(self, Takeoff, prefix + '/planning/takeoff')
        self.goto_client = ActionClient(self, GotoPose, prefix + '/planning/goto_pose')
        self.land_client = ActionClient(self, Land, prefix + '/planning/land')

        self._obstacle_spec = None     # (center xyz, size xyz), set once confirmed
        self.create_timer(1.0 / OBSTACLE_PUBLISH_HZ, self._publish_obstacle)

    # ------------------------------------------------------------------- intake

    def _on_odom(self, message):
        point = message.pose.pose.position
        self.position_now = (point.x, point.y, point.z)

    def _on_gt(self, message):
        point = message.pose.pose.position
        self.gt_position_now = (point.x, point.y, point.z)
        if self.recording:
            self.gt_samples.append((stamp_seconds(message.header.stamp), self.gt_position_now))

    def _on_command(self, message):
        if self.recording:
            self.commands.append((
                stamp_seconds(message.header.stamp),
                (message.position.x, message.position.y, message.position.z)))

    def _on_state(self, message):
        self.state = message

    def _on_trajectory(self, message):
        self.latest_trajectory = message
        if self.recording:
            self.trajectory_log.append((
                stamp_seconds(message.header.stamp), message.sequence, message.is_valid,
                len(message.points)))

    def _on_advice(self, message):
        self.latest_advice = message
        if self.recording:
            self.advice_log.append((self.sim_seconds(), message))

    def _publish_obstacle(self):
        if self._obstacle_spec is None:
            return
        center, size = self._obstacle_spec
        array = ObstacleArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = 'odom'          # contract 2.13: odom on /world/*
        array.uav_id = self.uav_id
        array.sensing_range = SENSING_RANGE
        obstacle = Obstacle()
        obstacle.obstacle_id = OBSTACLE_ID
        obstacle.shape = Obstacle.SHAPE_BOX
        obstacle.center.x, obstacle.center.y, obstacle.center.z = center
        obstacle.size.x, obstacle.size.y, obstacle.size.z = size
        obstacle.confidence = 1.0
        obstacle.position_uncertainty = OBSTACLE_UNCERTAINTY
        if self.gt_position_now is not None:
            box_min = [center[i] - size[i] / 2.0 for i in range(3)]
            box_max = [center[i] + size[i] / 2.0 for i in range(3)]
            obstacle.distance = point_to_aabb_distance(self.gt_position_now, box_min, box_max)
        array.obstacles = [obstacle]
        self.obstacle_publisher.publish(array)

    def begin_publishing_obstacle(self, center, size):
        self._obstacle_spec = (center, size)

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

    # ---------------------------------------------------------------- handshake

    def wait_for_spawn_confirmation(self):
        wall_deadline = time.time() + SPAWN_CONFIRM_TIMEOUT
        while rclpy.ok() and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if os.path.isfile(self.spawned_flag):
                return
        raise Failure(
            'the runner never confirmed the gz service spawn (%s missing) - '
            'the box may not actually exist in Gazebo' % self.spawned_flag)


def analyse_replans(trajectory_log, commands, message_text, since):
    """Two independent counts of the same event, reported side by side (integrity
    rule 4): the message-string chain that ties back to R-N1, and the raw
    sequence-number transitions on /planning/trajectory as corroboration.
    Only splines (points > 2) after `since` count: carrot flight publishes
    2-point segments with fresh sequences too, and those are not rebuilds."""
    message_count = message_text.count('avoidance escape:')

    # First-seen sim time of every distinct valid spline sequence number.
    seen = set()
    transitions = []
    for stamp, sequence, is_valid, points in trajectory_log:
        if stamp >= since and is_valid and points > 2 and sequence not in seen:
            seen.add(sequence)
            transitions.append((stamp, sequence))
    transitions.sort()
    sequence_replan_count = max(0, len(transitions) - 1)
    replan_times = [stamp for stamp, _ in transitions[1:]]

    max_gap = 1.5 / STREAM_HZ_RANGE[0]
    per_event_peaks = []
    for event_time in replan_times:
        window = [(t, p) for t, p in commands if abs(t - event_time) <= REPLAN_WINDOW_SEC]
        per_event_peaks.append(peak_acceleration(window, max_gap))

    return {
        'message_count': message_count,
        'sequence_replan_count': sequence_replan_count,
        'replan_times': replan_times,
        'per_event_peak_accel': per_event_peaks,
    }


def run_gate(node, record):
    # R27-1, rclpy side: this probe's create_publisher() ran in __init__, so
    # it is already advertised -- confirm it is still the ONLY writer on its
    # own output topic before anything downstream can consume from it.
    assert_single_writer(
        node, node.obstacle_topic, 1, '/uav/%s/state/vehicle' % node.uav_id)

    node.wait_until(lambda: node.state is not None and node.state.connected, 60.0, 'autopilot')
    node.wait_until(lambda: node.position_now is not None, 60.0, 'odometry_fused')
    node.wait_until(
        lambda: node.gt_position_now is not None, 30.0,
        'ground truth on %s (uav0_track odometry bridge)' % GT_ODOM_SUFFIX)

    origin = node.position_now
    print('origin (fused) %.3f %.3f %.3f' % origin)
    print('origin (ground truth) %.3f %.3f %.3f' % node.gt_position_now)

    node.recording = True

    takeoff = Takeoff.Goal()
    takeoff.target_altitude = float(TAKEOFF_ALTITUDE)
    handle = node.send_goal(node.takeoff_client, takeoff, 'takeoff')
    result = node.wait_result(handle, 120.0, 'takeoff')
    if result.status != GoalStatus.STATUS_SUCCEEDED:
        raise Failure('takeoff did not succeed: %s' % result.result.message)

    cruise_z = node.position_now[2]
    goal_xyz = (origin[0] + LEG_LENGTH, origin[1], cruise_z)

    baseline_sequence = node.latest_trajectory.sequence if node.latest_trajectory else -1

    sim_start = node.sim_seconds()
    wall_start = time.time()

    goal = GotoPose.Goal()
    goal.target_pose.position.x = float(goal_xyz[0])
    goal.target_pose.position.y = float(goal_xyz[1])
    goal.target_pose.position.z = float(goal_xyz[2])
    goal.target_pose.orientation.w = 1.0
    goal.frame_id = 'odom'
    goal.acceptance_radius = float(ACCEPTANCE_RADIUS)
    goal.max_speed = float(LEG_SPEED)
    goto_handle = node.send_goal(node.goto_client, goal, 'goto')

    # Claim 8 / precondition: the reactive lane only fires when the obstacle
    # appears AFTER a trajectory already exists for this goal. points > 2 keeps
    # a 2-point carrot segment from a hold/takeoff era passing as the plan.
    node.wait_until(
        lambda: (node.latest_trajectory is not None and node.latest_trajectory.is_valid
                 and len(node.latest_trajectory.points) > 2
                 and node.latest_trajectory.sequence != baseline_sequence),
        TRAJECTORY_ESTABLISHED_TIMEOUT, 'a fresh spline trajectory for the goto goal')
    print('trajectory established: sequence %d, %d points (baseline was %d)'
          % (node.latest_trajectory.sequence, len(node.latest_trajectory.points),
             baseline_sequence))

    current = node.position_now
    center = (
        0.5 * (current[0] + goal_xyz[0]),
        0.5 * (current[1] + goal_xyz[1]),
        cruise_z)
    size = OBSTACLE_SIZE
    print('obstacle requested at (%.3f %.3f %.3f) size (%.2f %.2f %.2f)'
          % (center + size))

    with open(node.request_file, 'w') as handle_file:
        handle_file.write('%.4f %.4f %.4f %.3f %.3f %.3f\n' % (center + size))

    node.wait_for_spawn_confirmation()
    print('runner confirmed the gz service spawn')

    z_before_avoidance = node.gt_position_now[2]
    spawn_confirmed_time = node.sim_seconds()
    node.begin_publishing_obstacle(center, size)

    # Precondition (rule 1): prove the feed is actually reaching the advisor
    # before any behavioural claim about the flight is scored.
    node.wait_until(
        lambda: node.latest_advice is not None and node.latest_advice.map_fresh
        and node.latest_advice.checked_horizon_m > 0.0,
        CONSUMPTION_EVIDENCE_TIMEOUT, 'the advisor to report a fresh, checked map')
    print('advisor map_fresh=True, checked_horizon_m=%.2f'
          % node.latest_advice.checked_horizon_m)

    node.wait_until(
        lambda: any(msg.advice == AvoidanceAdvice.ADVICE_ESCAPE
                    for _, msg in node.advice_log),
        CONSUMPTION_EVIDENCE_TIMEOUT, 'at least one ADVICE_ESCAPE (the obstacle recognised)')
    print('advisor issued ADVICE_ESCAPE at least once - the feed is being consumed')

    result = node.wait_result(goto_handle, 260.0, 'goto with an obstacle in the way')
    goal_end_time = node.sim_seconds()
    node.recording = False

    rtf = (node.sim_seconds() - sim_start) / max(1e-6, time.time() - wall_start)
    print('RTF while flying the avoidance leg: %.3f%s'
          % (rtf, '' if rtf >= TRUSTWORTHY_RTF else '  <-- BELOW 0.95, numbers are labelled'))

    record('0. the leg was actually sampled',
           len(node.commands) >= MIN_LEG_SAMPLES and len(node.gt_samples) >= 10,
           '%d setpoints, %d ground-truth samples' % (len(node.commands), len(node.gt_samples)))

    record('1. the goal succeeded before goto_timeout_sec (convergence)',
           result.status == GoalStatus.STATUS_SUCCEEDED,
           'status=%d message=%s' % (result.status, result.result.message))

    replans = analyse_replans(node.trajectory_log, node.commands, result.result.message, sim_start)
    print('replans seen in result.message ("avoidance escape:"): %d'
          % replans['message_count'])
    print('replans seen as trajectory sequence transitions: %d (times %s)'
          % (replans['sequence_replan_count'],
             ['%.2f' % t for t in replans['replan_times']]))
    if replans['message_count'] != replans['sequence_replan_count']:
        print('  NOTE: the two counts disagree - see the report for what this means')

    record('2. the reactive lane actually fired (message evidence, rule 4)',
           replans['message_count'] >= 1,
           '%d occurrences of "avoidance escape:" in result.message'
           % replans['message_count'])

    box_min = [center[i] - size[i] / 2.0 for i in range(3)]
    box_max = [center[i] + size[i] / 2.0 for i in range(3)]
    flight_window = [p for t, p in node.gt_samples if spawn_confirmed_time <= t <= goal_end_time]
    # Rule 1: zero ground-truth coverage of the avoidance window means the
    # collision/descent claims below have nothing to stand on - that is an
    # unmeasured claim, not a passed or failed one.
    if not flight_window:
        raise Failure(
            'no ground-truth samples (%s) fell inside the avoidance window '
            '(%.2f - %.2f s sim time); collision/descent cannot be judged'
            % (GT_ODOM_SUFFIX, spawn_confirmed_time, goal_end_time))

    clearances = [point_to_aabb_distance(p, box_min, box_max) for p in flight_window]
    min_clearance = safe_min(clearances, 'ground-truth clearance to the box')
    print('ground-truth clearance to the obstacle AABB: min %.3f m over %d samples'
          % (min_clearance, len(flight_window)))
    record('3. no collision (ground-truth clearance to the AABB > 0)',
           min_clearance > 0.0,
           'min clearance %.3f m over %d ground-truth samples' % (min_clearance, len(flight_window)))

    z_values = [p[2] for p in flight_window]
    min_z = safe_min(z_values, 'altitude during avoidance')
    record('4. no descending avoidance (z stayed within 0.2 m of the pre-avoidance z)',
           min_z >= z_before_avoidance - NO_DESCENT_TOLERANCE,
           'min z %.3f m, pre-avoidance z %.3f m (floor %.3f m)'
           % (min_z, z_before_avoidance, z_before_avoidance - NO_DESCENT_TOLERANCE))

    if replans['per_event_peak_accel']:
        worst_replan_accel = worst(
            replans['per_event_peak_accel'], 'acceleration through a trajectory rebuild')
    else:
        worst_replan_accel = float('nan')
    record('5. acceleration through each trajectory rebuild stays in envelope',
           replans['per_event_peak_accel'] and worst_replan_accel <= ACCEL_ENVELOPE_LIMIT,
           '%.2f m/s2 across %d rebuild(s) (limit %.1f, from max_acceleration 1.0 x 3)'
           % (worst_replan_accel, len(replans['per_event_peak_accel']), ACCEL_ENVELOPE_LIMIT))

    global_peak_accel = peak_acceleration(node.commands, 1.5 / STREAM_HZ_RANGE[0])
    print('whole-flight peak acceleration (context only, not gated): %.2f m/s2' % global_peak_accel)

    node.begin_publishing_obstacle(center, size)   # keep publishing through landing, harmless
    land = Land.Goal()
    land.use_precision_landing = False
    handle = node.send_goal(node.land_client, land, 'land')
    result = node.wait_result(handle, 120.0, 'land')
    node.wait_until(lambda: node.state is not None and not node.state.armed, 60.0, 'disarm')
    record('6. land disarmed the aircraft', result.status == GoalStatus.STATUS_SUCCEEDED,
           'status=%d' % result.status)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument(
        '--request-file', default=os.path.expanduser('~/gate_logs/gn4b_obstacle_request.txt'))
    parser.add_argument(
        '--spawned-flag', default=os.path.expanduser('~/gate_logs/gn4b_obstacle_spawned.flag'))
    args = parser.parse_args()

    rclpy.init()
    node = AvoidanceGate(args.uav_id, args.request_file, args.spawned_flag)
    checks = []
    failure = None

    def record(name, passed, detail):
        checks.append((name, bool(passed), detail))

    try:
        run_gate(node, record)
    except Failure as exc:
        failure = str(exc)
    except KeyboardInterrupt:
        failure = 'interrupted'

    print()
    print('=== G-N4b avoidance gate (uav0_track) ===')
    for name, ok, detail in checks:
        print('  [%s] %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))

    node.destroy_node()
    rclpy.shutdown()

    if failure:
        print('  FAILURE (measurement incomplete): %s' % failure)
        print('  RESULT: FAILED TO MEASURE')
        sys.stdout.flush()
        return 2

    if len(checks) != EXPECTED_CHECKS:
        print('  CHECK COUNT MISMATCH: %d recorded, %d expected - a check vanished'
              % (len(checks), EXPECTED_CHECKS))
    passed = len(checks) == EXPECTED_CHECKS and all(entry[1] for entry in checks)
    print('  RESULT: %s' % ('PASS' if passed else 'FAIL'))
    sys.stdout.flush()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
