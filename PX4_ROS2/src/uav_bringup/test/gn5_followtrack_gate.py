#!/usr/bin/env python3
"""G-N5: FollowPath over 4 waypoints, then a TrackTarget slice (orbit, recede,
lost). Run via scripts/run_gn5.sh. Model: uav0_nav.
NOT RUN by the author - sim access is reserved for the verifier-runner."""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from uav_interfaces.action import FollowPath, Land, Takeoff, TrackTarget
from uav_interfaces.msg import ControlCommand, Path3D, ResultCode, TargetState, TargetTrack, \
    VehicleState

TAKEOFF_ALTITUDE = 2.5          # m, matches the other flight gates

# ---------------------------------------------------------------- Part A: FollowPath
# 4 waypoints, relative to the hover point reached after Takeoff. The FIRST leg
# (hover point -> waypoints[0]) is a free transit: percentAlongPath() only sums
# distances *within* the waypoints array (navigator_action_server_node.cpp),
# so it never counts against completion%. The three INTERNAL legs are chosen
# deliberately uneven (1.0 / 3.0 / 1.5 m) so a bug that reports index-based
# completion (25/50/75/100%) is trivially distinguishable from arc-length based
# completion (~18/73/100%, see PROGRESS_CHECKPOINT below).
LEG_OFFSETS = [(1.5, 0.0), (1.5, 1.0), (4.5, 1.0), (4.5, -0.5)]
FOLLOWPATH_SPEED = 0.5          # m/s


def corner_cut(prev_xy, mid_xy, next_xy):
    """|W0 - 2*W1 + W2| / 6: how far the B-spline cuts inside an interior
    waypoint (uav_navigation/README.md S6c). Computed, not copied, per the brief."""
    vx = prev_xy[0] - 2.0 * mid_xy[0] + next_xy[0]
    vy = prev_xy[1] - 2.0 * mid_xy[1] + next_xy[1]
    return math.hypot(vx, vy) / 6.0


def _compute_corner_cuts():
    # Interior points are waypoints[0..2] (waypoints[3] is the exact target);
    # each is cut using its immediate neighbours in the full chain that starts
    # at the (relative) hover point (0, 0).
    chain = [(0.0, 0.0)] + LEG_OFFSETS
    return [corner_cut(chain[i - 1], chain[i], chain[i + 1]) for i in range(1, 4)]


_CORNER_CUTS = _compute_corner_cuts()
_MAX_CORNER_CUT = max(_CORNER_CUTS)
# 1.3x margin over the worst corner cut, same spirit as G-N2's ACCEPTANCE_RADIUS
# note: an acceptance radius at or below the cut would never be "reached" even
# on a perfectly flown path.
WAYPOINT_ACCEPTANCE_RADIUS = round(_MAX_CORNER_CUT * 1.3, 2)

_TOTAL_INTERNAL_LENGTH = sum(
    math.dist(LEG_OFFSETS[i], LEG_OFFSETS[i + 1]) for i in range(len(LEG_OFFSETS) - 1))
# Percent expected right after the FIRST internal leg (waypoints[0]->[1], 1.0 m)
# is flown: milestone[1] / total, exactly the quantity percentAlongPath()
# computes. +-5 points covers corner-cutting shifting *when* "ahead" advances
# and one feedback_hz (2.0 Hz) tick's worth of extra progress into the next
# (3.0 m) leg - both real dynamics, not slack for a wrong formula. An
# index-based bug would read 25% or 33%, well outside this band either way.
_FIRST_LEG_LENGTH = math.dist(LEG_OFFSETS[0], LEG_OFFSETS[1])
PROGRESS_CHECKPOINT_PERCENT = 100.0 * _FIRST_LEG_LENGTH / _TOTAL_INTERNAL_LENGTH
PROGRESS_CHECKPOINT_BAND = 5.0

# ---------------------------------------------------------------- Part B: TrackTarget
# Decision 4 (navigation_params.yaml): effective standoff = max(min_standoff_m,
# requested) + target_velocity_error_mps * target_reaction_sec = requested + 0.71
# when requested >= min_standoff_m (1.0). Both shipped defaults, cited here.
MIN_STANDOFF_M = 1.0
STANDOFF_REACTION_BONUS_M = 0.71
ORBIT_RADIUS_M = 4.0
# Picked so max(min_standoff_m, requested) + 0.71 == ORBIT_RADIUS_M exactly: the
# standoff point then sits AT the orbit centre every tick (see run_track_target),
# which is what makes phase 1 "hover and yaw" instead of "chase a moving ring".
REQUESTED_STANDOFF_M = ORBIT_RADIUS_M - STANDOFF_REACTION_BONUS_M
assert REQUESTED_STANDOFF_M >= MIN_STANDOFF_M, 'requested standoff fell below the shipped floor'
EFFECTIVE_STANDOFF_M = max(MIN_STANDOFF_M, REQUESTED_STANDOFF_M) + STANDOFF_REACTION_BONUS_M

ORBIT_ANGULAR_RATE_RAD_S = 0.15    # slow: well under max_yaw_rate_rad_s (0.5)
PHASE1_DURATION_SEC = 40.0
PHASE2_RADIAL_RATE_MPS = 0.08      # "recede slowly"
PHASE2_DURATION_SEC = 25.0
PHASE2_SETTLE_WINDOW_SEC = 8.0     # tail of phase 2 used for the convergence read
# Acceptance_radius (0.3 m, navigation_params.yaml) is the shipped notion of
# "arrived" for a STATIC goal; chasing a moving standoff point adds pursuit lag,
# so this is that value with a working margin, not a copied number.
STANDOFF_TOLERANCE_M = 0.45
DRONE_Z_HOLD_TOLERANCE_M = 0.5     # the aircraft must not chase the target's altitude

TARGET_ID = 42
TARGET_PUBLISH_HZ = 20.0
# Contract 2.12 forbids -1 on /world/*; must always carry at least the
# localization contribution. Matches the D2 obstacle-uncertainty precedent.
TARGET_UNCERTAINTY = 0.05
TARGET_LOST_TIMEOUT_SEC = 3.0      # short on purpose, keeps phase 3 quick

STREAM_HZ_RANGE = (12.0, 26.0)
TRUSTWORTHY_RTF = 0.95
WALL_CLOCK_SAFETY_FACTOR = 6.0
MIN_WALL_ALLOWANCE_SEC = 15.0
EXPECTED_CHECKS = 12            # a silently vanished check must not read as PASS


class Failure(Exception):
    """Raised when the measurement itself cannot be trusted; no verdict follows."""


def assert_single_writer(node, topic, expected, discovery_probe_topic, timeout_sec=15.0,
                          hold_sec=2.5, sample_period_sec=0.5):
    """rclpy twin of scripts/gate_common.sh's assert_single_writer -- a
    DETECTION net, not prevention: this node's own create_publisher() call on
    `topic` already counts as one writer, so `expected` is 1 here, not 0.
    A count above that means something else (perception:=true left on, most
    likely) is already publishing TargetState on our output topic; fixing
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


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def worst(values, label):
    """max() silently passes a NaN through; a missing sample must fail loudly."""
    if not values or any(value != value for value in values):
        raise Failure('%s has an unmeasured sample: %s' % (label, values))
    return max(values)


class TargetMotion:
    """Analytic target trajectory: orbit, then recede radially. The probe both
    publishes from this and grades against it, so there is never a correlation
    problem between "what we sent" and "what we check the drone against"."""

    def __init__(self, center_xy, target_z, orbit_radius, angular_rate,
                 phase1_duration, radial_rate):
        self.center_xy = center_xy
        self.target_z = target_z
        self.orbit_radius = orbit_radius
        self.angular_rate = angular_rate
        self.phase1_duration = phase1_duration
        self.radial_rate = radial_rate

    def _phase_and_angle(self, t):
        if t <= self.phase1_duration:
            return 1, self.angular_rate * t, self.orbit_radius
        angle = self.angular_rate * self.phase1_duration
        radius = self.orbit_radius + self.radial_rate * (t - self.phase1_duration)
        return 2, angle, radius

    def position(self, t):
        _, angle, radius = self._phase_and_angle(t)
        return (
            self.center_xy[0] + radius * math.cos(angle),
            self.center_xy[1] + radius * math.sin(angle),
            self.target_z)

    def velocity(self, t):
        phase, angle, radius = self._phase_and_angle(t)
        if phase == 1:
            return (
                -radius * self.angular_rate * math.sin(angle),
                radius * self.angular_rate * math.cos(angle))
        return (self.radial_rate * math.cos(angle), self.radial_rate * math.sin(angle))


class FollowTrackGate(Node):
    def __init__(self, uav_id):
        super().__init__(
            'gn5_followtrack_gate',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        prefix = '/uav/' + uav_id
        self.uav_id = uav_id

        self.state = None
        self.position_now = None
        self.recording = False
        self.commands = []          # (sim seconds, (x, y, z), yaw) - matches trajectory_gate.py
        self.odometry = []          # (sim seconds, (x, y, z))

        self.target_topic = prefix + '/world/target_state'
        self.target_publisher = self.create_publisher(TargetState, self.target_topic, 10)
        self._motion = None
        self._target_phase_start = None
        self._target_active = False
        self.published_targets = []     # (sim seconds, (x, y, z))
        self.create_timer(1.0 / TARGET_PUBLISH_HZ, self._publish_target)

        self.create_subscription(Odometry, prefix + '/state/odometry_fused', self._on_odom, 20)
        self.create_subscription(
            ControlCommand, prefix + '/control/command_selected', self._on_command, 50)
        self.create_subscription(VehicleState, prefix + '/state/vehicle', self._on_state, 10)

        self.takeoff_client = ActionClient(self, Takeoff, prefix + '/planning/takeoff')
        self.follow_client = ActionClient(self, FollowPath, prefix + '/planning/follow_path')
        self.track_client = ActionClient(self, TrackTarget, prefix + '/planning/track_target')
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

    def _publish_target(self):
        if not self._target_active or self._motion is None:
            return
        now = self.sim_seconds()
        t = now - self._target_phase_start
        x, y, z = self._motion.position(t)
        vx, vy = self._motion.velocity(t)
        msg = TargetState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.uav_id = self.uav_id
        msg.track_id = TARGET_ID
        msg.status = TargetTrack.STATUS_TRACKING
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.w = 1.0
        msg.velocity.x, msg.velocity.y, msg.velocity.z = vx, vy, 0.0
        msg.position_uncertainty = TARGET_UNCERTAINTY
        msg.time_since_seen_sec = 0.0
        self.target_publisher.publish(msg)
        if self.recording:
            self.published_targets.append((now, (x, y, z)))

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

    def send_goal(self, client, goal, description, feedback_callback=None):
        self.await_server(client, 30.0, description)
        future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        handle = self.wait_future(future, 15.0, '%s goal response' % description)
        if not handle.accepted:
            raise Failure('%s goal was rejected' % description)
        return handle

    def wait_result(self, handle, timeout, description):
        return self.wait_future(handle.get_result_async(), timeout, '%s result' % description)


# ------------------------------------------------------------------- Part A

def run_follow_path(node, record):
    origin = node.position_now
    cruise_z = origin[2]
    waypoints = [(origin[0] + dx, origin[1] + dy, cruise_z) for dx, dy in LEG_OFFSETS]
    print('waypoint acceptance radius %.3f m (worst corner cut %.3f m, cuts=%s)'
          % (WAYPOINT_ACCEPTANCE_RADIUS, _MAX_CORNER_CUT,
             ['%.3f' % c for c in _CORNER_CUTS]))
    print('internal path length %.2f m, checkpoint after the 1 m leg: %.1f%% +-%.0f'
          % (_TOTAL_INTERNAL_LENGTH, PROGRESS_CHECKPOINT_PERCENT, PROGRESS_CHECKPOINT_BAND))

    path = Path3D()
    path.header.frame_id = 'odom'
    path.header.stamp = node.get_clock().now().to_msg()
    path.uav_id = node.uav_id
    path.is_valid = True
    path.plan_state = Path3D.PLAN_STATE_VALID
    path.planner_name = 'gn5_probe'
    for x, y, z in waypoints:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w = 1.0
        path.waypoints.append(pose)

    goal = FollowPath.Goal()
    goal.path = path
    goal.max_speed = float(FOLLOWPATH_SPEED)
    goal.waypoint_acceptance_radius = float(WAYPOINT_ACCEPTANCE_RADIUS)

    feedback_log = []

    def on_feedback(feedback_msg):
        feedback_log.append((node.sim_seconds(), feedback_msg.feedback))

    node.commands = []
    node.odometry = []
    node.recording = True
    sim_start = node.sim_seconds()
    wall_start = time.time()
    handle = node.send_goal(node.follow_client, goal, 'follow path', on_feedback)
    result = node.wait_result(handle, 260.0, 'follow path')
    node.recording = False
    rtf = (node.sim_seconds() - sim_start) / max(1e-6, time.time() - wall_start)

    print('FollowPath: %d setpoints, %d feedback samples, RTF %.3f%s'
          % (len(node.commands), len(feedback_log), rtf,
             '' if rtf >= TRUSTWORTHY_RTF else '  <-- BELOW 0.95, numbers are labelled'))

    if result.status != GoalStatus.STATUS_SUCCEEDED:
        raise Failure('follow path did not succeed: %s' % result.result.message)
    if not feedback_log:
        raise Failure('follow path produced no feedback - nothing to grade')

    indices = [fb.current_waypoint_index for _, fb in feedback_log]
    percents = [fb.path_completion_percent for _, fb in feedback_log]

    record('a. waypoints reached == 4', result.result.waypoints_reached == 4,
           '%d of 4 (message: %s)' % (result.result.waypoints_reached, result.result.message))
    record('b. first-seen waypoint index is non-decreasing',
           all(b >= a for a, b in zip(indices, indices[1:])),
           'indices %s' % indices)
    record('c. the index actually progressed (not stuck)',
           len(set(indices)) >= 3, 'distinct indices seen: %s' % sorted(set(indices)))
    record('d. completion% is monotonic (peak-held)',
           all(b >= a - 0.05 for a, b in zip(percents, percents[1:])),
           'min step %.3f' % min((b - a for a, b in zip(percents, percents[1:])), default=0.0))
    record('e. completion% ends at 100 (within feedback_hz sampling)',
           bool(percents) and max(percents) >= 99.0,
           'max observed %.2f%%' % (max(percents) if percents else float('nan')))

    checkpoint_samples = [
        percent for index, percent in zip(indices, percents) if index >= 2]
    checkpoint_value = checkpoint_samples[0] if checkpoint_samples else float('nan')
    record('f. completion% after the 1 m leg matches ARC LENGTH, not waypoint count',
           bool(checkpoint_samples)
           and abs(checkpoint_value - PROGRESS_CHECKPOINT_PERCENT) <= PROGRESS_CHECKPOINT_BAND,
           '%.1f%% at the first index>=2 sample (expected %.1f%% +-%.0f; an index-based '
           'bug would read 25%% or 33%%)'
           % (checkpoint_value, PROGRESS_CHECKPOINT_PERCENT, PROGRESS_CHECKPOINT_BAND))

    return waypoints[-1]


# ------------------------------------------------------------------- Part B

def yaw_lag_series(commands, phase_start, motion):
    lags = []
    for t, (x, y, _z), yaw in commands:
        tx, ty, _tz = motion.position(t - phase_start)
        bearing = math.atan2(ty - y, tx - x)
        lags.append(abs(wrap_pi(yaw - bearing)))
    return lags


def run_track_target(node, record):
    node.wait_until(lambda: node.position_now is not None, 30.0, 'odometry_fused after follow path')
    center = node.position_now[:2]
    cruise_z = node.position_now[2]
    print('track target: orbit centre (%.3f, %.3f), effective standoff %.2f m '
          '(requested %.2f + %.2f reaction bonus)'
          % (center[0], center[1], EFFECTIVE_STANDOFF_M,
             REQUESTED_STANDOFF_M, STANDOFF_REACTION_BONUS_M))

    motion = TargetMotion(
        center, target_z=node.position_now[2] - TAKEOFF_ALTITUDE,
        orbit_radius=ORBIT_RADIUS_M, angular_rate=ORBIT_ANGULAR_RATE_RAD_S,
        phase1_duration=PHASE1_DURATION_SEC, radial_rate=PHASE2_RADIAL_RATE_MPS)
    node._motion = motion
    node._target_phase_start = node.sim_seconds()
    node._target_active = True
    node.published_targets = []
    node.commands = []
    node.odometry = []
    node.recording = True

    goal = TrackTarget.Goal()
    goal.target_id = TARGET_ID
    goal.standoff_distance = float(REQUESTED_STANDOFF_M)
    goal.duration_seconds = 0.0             # track until we end it
    goal.target_lost_timeout = float(TARGET_LOST_TIMEOUT_SEC)

    feedback_log = []

    def on_feedback(feedback_msg):
        feedback_log.append((node.sim_seconds(), feedback_msg.feedback))

    handle = node.send_goal(node.track_client, goal, 'track target', on_feedback)

    # ---- Phase 1: orbit. Nose lag is MEASURED and printed, never thresholded
    # (debt 13, memory.md S7 item 13: measure first, set a threshold later). ----
    phase_start = node._target_phase_start
    node.wait_until(
        lambda: node.sim_seconds() - phase_start >= PHASE1_DURATION_SEC,
        PHASE1_DURATION_SEC + 20.0, 'phase 1 (orbit) to finish')
    phase1_end = node.sim_seconds()

    phase1_commands = [c for c in node.commands if phase_start <= c[0] <= phase1_end]
    phase1_feedback = [fb for t, fb in feedback_log if t <= phase1_end]
    if not phase1_commands:
        raise Failure('no ControlCommand samples during phase 1 - nose lag cannot be measured')
    lags = yaw_lag_series(phase1_commands, phase_start, motion)
    peak_lag_deg = math.degrees(worst(lags, 'phase 1 nose lag'))
    print('phase 1: peak |commanded yaw - bearing to target| = %.1f deg '
          '(MEASURED ONLY, no threshold - debt 13) over %d samples'
          % (peak_lag_deg, len(phase1_commands)))

    record('g. target stayed visible through the orbit (phase 1)',
           bool(phase1_feedback) and all(fb.target_visible for fb in phase1_feedback),
           '%d/%d feedback samples report visible'
           % (sum(1 for fb in phase1_feedback if fb.target_visible), len(phase1_feedback)))

    phase1_z = [p[1][2] for p in node.odometry if phase_start <= p[0] <= phase1_end]
    # A silently skipped check reads as PASS; an empty window is unmeasured.
    if not phase1_z:
        raise Failure('no odometry in phase 1 - the altitude-hold claim has no samples')
    drift = max(abs(z - cruise_z) for z in phase1_z)
    record('h. the aircraft did not chase the target altitude (phase 1)',
           drift <= DRONE_Z_HOLD_TOLERANCE_M,
           'max |z - cruise| %.3f m (limit %.2f), target z %.2f m'
           % (drift, DRONE_Z_HOLD_TOLERANCE_M, motion.target_z))

    # ---- Phase 2: target recedes; standoff must converge on EFFECTIVE_STANDOFF_M. ----
    node.wait_until(
        lambda: node.sim_seconds() - phase_start >= PHASE1_DURATION_SEC + PHASE2_DURATION_SEC,
        PHASE2_DURATION_SEC + 20.0, 'phase 2 (recede) to finish')
    phase2_end = node.sim_seconds()
    settle_start = phase2_end - PHASE2_SETTLE_WINDOW_SEC

    tail_odometry = [(t, p) for t, p in node.odometry if settle_start <= t <= phase2_end]
    if not tail_odometry:
        raise Failure('no odometry samples in the phase 2 settle window - standoff not measured')
    horizontal_gaps = []
    for t, (x, y, _z) in tail_odometry:
        tx, ty, _tz = motion.position(t - phase_start)
        horizontal_gaps.append(math.hypot(tx - x, ty - y))
    mean_gap = sum(horizontal_gaps) / len(horizontal_gaps)
    worst_gap_error = worst(
        [abs(g - EFFECTIVE_STANDOFF_M) for g in horizontal_gaps], 'standoff convergence error')
    record('i. standoff converges on requested + 0.71 m (decision 4)',
           worst_gap_error <= STANDOFF_TOLERANCE_M,
           'mean horizontal gap %.3f m, worst error %.3f m from target %.2f m (tolerance %.2f)'
           % (mean_gap, worst_gap_error, EFFECTIVE_STANDOFF_M, STANDOFF_TOLERANCE_M))

    # ---- Phase 3: stop publishing target_state; must abort ABORTED_LOST_TARGET,
    # not a generic timeout. This is the fault-injection positive control for
    # Part B (integrity rule 3): we manufacture the one condition designed to
    # make the gate fail a specific, distinguishable way. ----
    node._target_active = False
    print('phase 3: target_state publishing stopped, waiting for ABORTED_LOST_TARGET')
    result = node.wait_result(
        handle, TARGET_LOST_TIMEOUT_SEC + 30.0, 'track target after losing the target')
    node.recording = False

    lost_feedback = [fb for t, fb in feedback_log if t > phase2_end]
    record('j. feedback reported the target no longer visible before the abort',
           any(not fb.target_visible for fb in lost_feedback),
           '%d feedback samples after target_state stopped, %d report invisible'
           % (len(lost_feedback), sum(1 for fb in lost_feedback if not fb.target_visible)))
    record('k. the goal aborted with ABORTED_LOST_TARGET, not a timeout',
           result.status == GoalStatus.STATUS_ABORTED
           and result.result.result_code == ResultCode.ABORTED_LOST_TARGET,
           'status=%d result_code=%d message=%s'
           % (result.status, result.result.result_code, result.result.message))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    args = parser.parse_args()

    rclpy.init()
    node = FollowTrackGate(args.uav_id)
    checks = []
    failure = None

    def record(name, passed, detail):
        checks.append((name, bool(passed), detail))

    try:
        # R27-1, rclpy side: this probe's create_publisher() ran in __init__,
        # so it is already advertised -- confirm it is still the ONLY writer
        # on its own output topic before anything downstream can consume it.
        assert_single_writer(
            node, node.target_topic, 1, '/uav/%s/state/vehicle' % node.uav_id)

        node.wait_until(lambda: node.state is not None and node.state.connected, 60.0, 'autopilot')
        node.wait_until(lambda: node.position_now is not None, 60.0, 'odometry_fused')
        # Position source note (per the brief): odometry_fused, same as
        # uav_navigation/test/trajectory_gate.py on uav0_nav. It is the
        # estimator's fused output, not literal Gazebo ground truth - but
        # memory.md 2026-08-16/18 (G-N1) measured fused-raw at the mm level on
        # this model after the EKF2 double-vision fix, so it stands in for it.
        origin = node.position_now
        print('origin (fused) %.3f %.3f %.3f' % origin)

        takeoff = Takeoff.Goal()
        takeoff.target_altitude = float(TAKEOFF_ALTITUDE)
        handle = node.send_goal(node.takeoff_client, takeoff, 'takeoff')
        result = node.wait_result(handle, 120.0, 'takeoff')
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise Failure('takeoff did not succeed: %s' % result.result.message)

        run_follow_path(node, record)
        run_track_target(node, record)

        land = Land.Goal()
        land.use_precision_landing = False
        handle = node.send_goal(node.land_client, land, 'land')
        result = node.wait_result(handle, 120.0, 'land')
        node.wait_until(lambda: node.state is not None and not node.state.armed, 60.0, 'disarm')
        record('l. land disarmed the aircraft', result.status == GoalStatus.STATUS_SUCCEEDED,
               'status=%d' % result.status)
    except Failure as exc:
        failure = str(exc)
    except KeyboardInterrupt:
        failure = 'interrupted'

    print()
    print('=== G-N5 FollowPath + TrackTarget gate (uav0_nav) ===')
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
