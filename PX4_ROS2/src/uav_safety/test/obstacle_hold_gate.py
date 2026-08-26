#!/usr/bin/env python3
"""G-S3-HOLD (P8-safety.md S:8 P8.5, verify_obstacle_hold.sh): fly a Goto,
inject an OBSTACLE_TOO_CLOSE fault by publishing ObstacleArray directly on
/world/obstacle_map_local (world_model_node must be OFF -- perception:=false,
sim.launch.py's own default -- so this probe is the ONLY publisher; the
wrapper script proves that with `ros2 topic info` BEFORE this process starts,
R27 dieu 1), and record the timestamped chain:
  obstacle published -> command_selected source -> SOURCE_SAFETY
  -> cmd_safety stream (frozen pose, restamped every tick)
  -> /safety/state recommended_action == "hold" + OBSTACLE_TOO_CLOSE violation
  -> obstacle cleared (empty ObstacleArray, R27-2: +inf is a MEASURED clear,
     not "cannot measure") -> ClearFault accepted after clear_stability_sec
  -> a fresh GotoPose proves the door is open again (command_selected flows
     with source != SAFETY).
R1 R20/R25: only uav_interfaces types, never px4_msgs -- not isolated to a
domain on purpose (must see the real flight, real arbiter, real gateway).

This probe does NOT judge whether navigator's ORIGINAL GotoPose goal survives
losing authority mid-flight (P8-safety.md S:8: "ghi nhan hanh vi that, khong
ep PASS/FAIL len phan navigator, ngoai pham vi cong nay") -- it records the
goal's terminal status and moves on; PASS/FAIL is judged only on the safety
chain itself (a-e in the P8.5 gate spec).

Usage: python3 obstacle_hold_gate.py --uav-id uav0
Prints a report; exit 0 means the recorded chain matched every acceptance
criterion below. Exit 1 with a printed reason otherwise -- never silently
downgraded to PASS (R0: FAILED-TO-MEASURE never becomes PASS).
"""
import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Header
from uav_interfaces.action import GotoPose, Takeoff
from uav_interfaces.msg import (
    ControlAuthority,
    ControlCommand,
    Obstacle,
    ObstacleArray,
    SafetyState,
    VehicleState,
)
from uav_interfaces.srv import ClearFault

TAKEOFF_ALTITUDE = 2.5       # m above the starting point
GOTO_EAST = 8.0               # m -- long enough leg to hold mid-flight
GOTO_SPEED = 0.35             # m/s, slow: wide window to observe HOLD in

# --- safety_params.yaml mirrors (S:9d discipline: named copies, not magic numbers) ---
MIN_OBSTACLE_DISTANCE_M = 0.35
OBSTACLE_GRACE_SEC = 0.5
OBSTACLE_MAP_TIMEOUT_SEC = 0.3
CLEAR_STABILITY_SEC = 3.0
HOLD_STREAM_HZ = 20.0
HOLD_STREAM_MIN_HZ = 15.0      # validate() V3 floor: hold_stream_hz >= 3/0.20
FROZEN_SETPOINT_EPSILON_M = 0.05

INJECTED_DISTANCE_M = 0.18     # well under the 0.35 m threshold
OBSTACLE_PUBLISH_HZ = 10.0     # matches world_model_node's real cadence
HOLD_INJECT_SEC = 6.0          # >> obstacle_grace_sec, gives a clean plateau
MAX_ONSET_SEC = 2.5            # budget: grace 0.5 + tick/latch/gateway slack
# clear_stability_sec is measured ACROSS CALLS to the clear_fault SERVICE
# (failsafe_policy.cpp's own comment), not wall-clock waiting -- must POLL,
# not sleep-then-call-once. Budget: map goes fresh+empty (map timeout) +
# >= clear_stability_sec of polling at CLEAR_POLL_PERIOD_SEC + slack for the
# first couple of rejected (stable_for still climbing) polls.
CLEAR_POLL_PERIOD_SEC = 0.5
CLEAR_WAIT_SEC = OBSTACLE_MAP_TIMEOUT_SEC + CLEAR_STABILITY_SEC + 4.0
POST_CLEAR_SETTLE_SEC = 2.0    # window to confirm cmd_safety really stopped
RESUME_TIMEOUT_SEC = 15.0      # window to see command_selected flow again

SOURCE_SAFETY = ControlAuthority.SOURCE_SAFETY
SOURCE_MISSION = ControlAuthority.SOURCE_MISSION
SOURCE_TEST = ControlAuthority.SOURCE_TEST


class Failure(Exception):
    pass


class _LastClearAttempt:
    """Fallback report object when the ClearFault poll loop times out --
    carries only what main()'s report printing/PASS logic reads (.success,
    .remaining_faults). Never fabricates a success."""

    def __init__(self, success, remaining_faults):
        self.success = success
        self.remaining_faults = remaining_faults


class ObstacleHoldGate(Node):
    def __init__(self, uav_id, use_sim_time):
        from rclpy.parameter import Parameter
        super().__init__(
            'obstacle_hold_gate',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])
        prefix = '/uav/' + uav_id
        self.uav_id = uav_id

        self.state = None
        self.odom = None

        self.commands = []          # (wall, source, x, y, z)
        self.cmd_safety = []        # (wall, x, y, z)
        self.cmd_mission = []       # (wall, x, y, z)
        self.safety_states = []     # (wall, level, recommended_action, obstacle_clear)
        self.violations = []        # (wall, code, message)
        self.altitude_samples = []  # (wall, z)

        self.create_subscription(Odometry, prefix + '/state/odometry_fused', self._on_odom, 10)
        self.create_subscription(
            ControlCommand, prefix + '/control/command_selected', self._on_command, 50)
        self.create_subscription(
            ControlCommand, prefix + '/control/cmd_safety', self._on_cmd_safety, 50)
        self.create_subscription(
            ControlCommand, prefix + '/control/cmd_mission', self._on_cmd_mission, 50)
        self.create_subscription(VehicleState, prefix + '/state/vehicle', self._on_state, 10)
        self.create_subscription(SafetyState, prefix + '/safety/state', self._on_safety_state, 10)
        self.create_subscription(
            DiagnosticArray, prefix + '/safety/violations', self._on_violations, 20)

        self.obstacle_publisher = self.create_publisher(
            ObstacleArray, prefix + '/world/obstacle_map_local', 10)

        self.takeoff_client = ActionClient(self, Takeoff, prefix + '/planning/takeoff')
        self.goto_client = ActionClient(self, GotoPose, prefix + '/planning/goto_pose')
        self.clear_fault_client = self.create_client(ClearFault, prefix + '/safety/clear_fault')

    # ------------------------------------------------------------ callbacks

    def _on_odom(self, msg):
        self.odom = msg
        p = msg.pose.pose.position
        self.altitude_samples.append((time.time(), p.z))

    def _on_command(self, msg):
        p = msg.position
        self.commands.append((time.time(), msg.source, p.x, p.y, p.z))

    def _on_cmd_safety(self, msg):
        p = msg.position
        self.cmd_safety.append((time.time(), p.x, p.y, p.z))
        self._log('cmd_safety x=%.4f y=%.4f z=%.4f' % (p.x, p.y, p.z))

    def _on_cmd_mission(self, msg):
        p = msg.position
        self.cmd_mission.append((time.time(), p.x, p.y, p.z))

    def _on_state(self, msg):
        self.state = msg

    def _on_safety_state(self, msg):
        self.safety_states.append(
            (time.time(), msg.level, msg.recommended_action, msg.obstacle_clear))
        self._log(
            'safety/state level=%d action=%s obstacle_clear=%s'
            % (msg.level, msg.recommended_action, msg.obstacle_clear))

    def _on_violations(self, msg):
        for status in msg.status:
            if status.name.startswith('safety: '):
                code = status.name[len('safety: '):]
                self.violations.append((time.time(), code, status.message))
                self._log('violation edge: %s -- %s' % (code, status.message))

    def _log(self, text):
        print('  [t=%.3f] %s' % (time.time(), text))
        sys.stdout.flush()

    # -------------------------------------------------------------- helpers

    def spin(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_and_publish(self, seconds, obstacle_present):
        """Publish ObstacleArray at OBSTACLE_PUBLISH_HZ while spinning. When
        obstacle_present is False the array is EMPTY (R27-2: +inf, a real
        measured 'clear', not a missing measurement) -- keeps the map FRESH
        (< obstacle_map_timeout_copy_sec) the whole time so clearFault's
        clear_stability_sec window is judged against a genuine live reading,
        not a channel that merely went stale."""
        end = time.time() + seconds
        period = 1.0 / OBSTACLE_PUBLISH_HZ
        next_publish = time.time()
        while rclpy.ok() and time.time() < end:
            now = time.time()
            if now >= next_publish:
                self._publish_obstacle(obstacle_present)
                next_publish = now + period
            rclpy.spin_once(self, timeout_sec=0.02)

    def _publish_obstacle(self, obstacle_present):
        msg = ObstacleArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.uav_id = self.uav_id
        msg.sensing_range = 5.0
        if obstacle_present:
            obstacle = Obstacle()
            obstacle.obstacle_id = 1
            obstacle.shape = Obstacle.SHAPE_SPHERE
            if self.odom is not None:
                p = self.odom.pose.pose.position
                obstacle.center = Point(x=p.x + 1.0, y=p.y, z=p.z)
            else:
                obstacle.center = Point(x=0.0, y=0.0, z=0.0)
            obstacle.size = Vector3(x=0.3, y=0.3, z=0.3)
            obstacle.distance = float(INJECTED_DISTANCE_M)
            obstacle.confidence = 1.0
            obstacle.position_uncertainty = 0.05
            msg.obstacles = [obstacle]
        else:
            msg.obstacles = []
        self.obstacle_publisher.publish(msg)

    def wait_until(self, predicate, timeout, description):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise Failure('timed out waiting for %s' % description)

    def await_server(self, client, timeout, description):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            if client.server_is_ready():
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise Failure('%s action server never appeared' % description)

    def send_goal(self, client, goal, description):
        self.await_server(client, 30.0, description)
        future = client.send_goal_async(goal)
        end = time.time() + 15.0
        while rclpy.ok() and not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise Failure('%s goal response timed out' % description)
        handle = future.result()
        if not handle.accepted:
            raise Failure('%s goal rejected' % description)
        return handle

    def wait_result(self, handle, timeout, description):
        future = handle.get_result_async()
        end = time.time() + timeout
        while rclpy.ok() and not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            return None, 'TIMEOUT'
        result = future.result()
        return result, result.status

    def position(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        return (p.x, p.y, p.z)

    def call_clear_fault(self, fault_code=''):
        if not self.clear_fault_client.wait_for_service(timeout_sec=10.0):
            raise Failure('clear_fault service never appeared')
        request = ClearFault.Request()
        request.fault_code = fault_code
        future = self.clear_fault_client.call_async(request)
        end = time.time() + 10.0
        while rclpy.ok() and not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise Failure('clear_fault call timed out')
        return future.result()


def run(node, watch_hold_after_clear_sec):
    node.wait_until(lambda: node.state is not None and node.state.connected, 60.0, 'autopilot')
    node.wait_until(lambda: node.position() is not None, 30.0, 'odometry_fused')
    origin = node.position()
    print('origin (fused) %.3f %.3f %.3f' % origin)

    takeoff = Takeoff.Goal()
    takeoff.target_altitude = float(TAKEOFF_ALTITUDE)
    takeoff_handle = node.send_goal(node.takeoff_client, takeoff, 'takeoff')
    result, status = node.wait_result(takeoff_handle, 60.0, 'takeoff')
    if status != GoalStatus.STATUS_SUCCEEDED:
        raise Failure('takeoff finished with status=%s' % status)
    print('takeoff complete, altitude %.2f' % node.position()[2])

    goal = GotoPose.Goal()
    goal.target_pose.position.x = origin[0] + GOTO_EAST
    goal.target_pose.position.y = origin[1]
    goal.target_pose.position.z = origin[2] + TAKEOFF_ALTITUDE
    goal.target_pose.orientation.w = 1.0
    goal.frame_id = 'odom'
    goal.acceptance_radius = 0.5
    goal.max_speed = float(GOTO_SPEED)
    goto_handle = node.send_goal(node.goto_client, goal, 'goto')
    print('goto sent (max_speed=%.2f m/s)' % GOTO_SPEED)

    # --- e (pre): positive control -- mission stream really flowing before injection.
    node.wait_until(
        lambda: any(source == SOURCE_MISSION or source == SOURCE_TEST for _w, source, *_ in node.commands),
        20.0, 'command_selected to show a MISSION/TEST source before injection')
    node.spin(2.0)  # a couple more seconds of clean baseline before we touch anything
    pre_inject_command_count = len(node.commands)
    print('pre-injection: %d command_selected samples, last source=%s'
          % (pre_inject_command_count, node.commands[-1][1] if node.commands else None))

    # --- inject: OBSTACLE_TOO_CLOSE, continuously, for HOLD_INJECT_SEC.
    injected_at = time.time()
    print('=== INJECT obstacle distance=%.2f m (< %.2f threshold) at t=%.3f ==='
          % (INJECTED_DISTANCE_M, MIN_OBSTACLE_DISTANCE_M, injected_at))
    node.spin_and_publish(HOLD_INJECT_SEC, obstacle_present=True)

    # Two independent witnesses of the same onset -- command_selected's source
    # field flipping to SAFETY, and the HOLD stream itself starting to arrive
    # on cmd_safety -- take whichever fires first (they can be one arbiter
    # tick apart depending on which topic the daemon delivers first).
    onset_from_command = None
    for wall, source, *_ in node.commands:
        if wall >= injected_at and source == SOURCE_SAFETY:
            onset_from_command = wall
            break
    onset_from_cmd_safety = None
    for wall, *_ in node.cmd_safety:
        if wall >= injected_at:
            onset_from_cmd_safety = wall
            break
    candidates = [w for w in (onset_from_command, onset_from_cmd_safety) if w is not None]
    onset_wall = min(candidates) if candidates else None

    # --- clear: empty map (R27-2: +inf, a measured clear) + POLL ClearFault.
    # failsafe_policy.cpp's clear_stability_sec is measured ACROSS CALLS to
    # clearFault() (its own comment: "a caller wanting timely confirmation
    # polls it, same pattern as any ROS2 service-based retry loop") -- a
    # single call after a wall-clock sleep would always see stable_for=0 on
    # its first (only) sample and get rejected. Poll at CLEAR_POLL_PERIOD_SEC
    # while keeping the map continuously fresh+empty.
    print('=== CLEAR: publishing empty ObstacleArray + polling ClearFault at t=%.3f ===' % time.time())
    clear_start = time.time()
    next_obstacle_publish = clear_start
    next_clear_poll = clear_start + OBSTACLE_MAP_TIMEOUT_SEC  # let the map go empty+fresh first
    clear_attempts = []
    clear_result = None
    deadline = clear_start + CLEAR_WAIT_SEC
    while rclpy.ok() and time.time() < deadline:
        now = time.time()
        if now >= next_obstacle_publish:
            node._publish_obstacle(False)
            next_obstacle_publish = now + (1.0 / OBSTACLE_PUBLISH_HZ)
        if now >= next_clear_poll:
            attempt = node.call_clear_fault('')
            clear_attempts.append((time.time(), attempt.success, list(attempt.remaining_faults)))
            print('  ClearFault poll @ t=%.3f: success=%s remaining=%s'
                  % (time.time(), attempt.success, list(attempt.remaining_faults)))
            if attempt.success:
                clear_result = attempt
                break
            next_clear_poll = time.time() + CLEAR_POLL_PERIOD_SEC
        rclpy.spin_once(node, timeout_sec=0.02)
    if clear_result is None:
        # Timed out polling -- report the LAST attempt (FAILED TO MEASURE if
        # there was none at all), never invent a success.
        if clear_attempts:
            _wall, success, remaining = clear_attempts[-1]
            clear_result = _LastClearAttempt(success, remaining)
        else:
            clear_result = _LastClearAttempt(False, ['NEVER POLLED -- CLEAR_WAIT_SEC too short'])
    print('ClearFault final: success=%s remaining_faults=%s (polled %d time(s) over %.1fs)'
          % (clear_result.success, clear_result.remaining_faults, len(clear_attempts),
             time.time() - clear_start))

    cmd_safety_before_settle = len(node.cmd_safety)
    settle_start = time.time()
    node.spin_and_publish(POST_CLEAR_SETTLE_SEC, obstacle_present=False)
    cmd_safety_after_settle = len(node.cmd_safety)
    new_cmd_safety_after_clear = [
        s for s in node.cmd_safety if s[0] >= settle_start
    ]

    # --- resume: prove the door is open again with a fresh, deliberate goal.
    print('=== RESUME: sending a fresh GotoPose after ClearFault ===')
    resume_pos = node.position() or origin
    resume_goal = GotoPose.Goal()
    resume_goal.target_pose.position.x = resume_pos[0]
    resume_goal.target_pose.position.y = resume_pos[1]
    resume_goal.target_pose.position.z = resume_pos[2]
    resume_goal.target_pose.orientation.w = 1.0
    resume_goal.frame_id = 'odom'
    resume_goal.acceptance_radius = 0.5
    resume_goal.max_speed = float(GOTO_SPEED)
    resume_sent_at = time.time()
    resume_accepted = True
    try:
        node.send_goal(node.goto_client, resume_goal, 'resume goto')
    except Failure as exc:
        resume_accepted = False
        print('  resume goto NOT accepted: %s' % exc)

    resume_seen_at = None
    end = time.time() + RESUME_TIMEOUT_SEC
    while rclpy.ok() and time.time() < end and resume_seen_at is None:
        rclpy.spin_once(node, timeout_sec=0.02)
        for wall, source, *_ in node.commands:
            if wall >= resume_sent_at and source != SOURCE_SAFETY:
                resume_seen_at = wall
                break

    # Original goto goal's terminal status -- recorded, not judged (S:8 caveat).
    orig_goal_status = 'unknown'
    try:
        _r, orig_goal_status = node.wait_result(goto_handle, 0.5, 'original goto (non-blocking check)')
    except Exception:
        pass

    return {
        'injected_at': injected_at,
        'onset_wall': onset_wall,
        'clear_result': clear_result,
        'cmd_safety_before_settle': cmd_safety_before_settle,
        'cmd_safety_after_settle': cmd_safety_after_settle,
        'new_cmd_safety_after_clear': new_cmd_safety_after_clear,
        'resume_accepted': resume_accepted,
        'resume_sent_at': resume_sent_at,
        'resume_seen_at': resume_seen_at,
        'orig_goal_status': orig_goal_status,
        'pre_inject_command_count': pre_inject_command_count,
    }


def analyze(node, report):
    out = dict(report)
    injected_at = report['injected_at']
    onset_wall = report['onset_wall']
    out['onset_after_injection_sec'] = (onset_wall - injected_at) if onset_wall else float('nan')

    # --- HOLD stream during [onset_wall, onset_wall + HOLD_INJECT_SEC]: rate + freeze.
    hold_samples = [s for s in node.cmd_safety if onset_wall and s[0] >= onset_wall]
    out['hold_sample_count'] = len(hold_samples)
    if len(hold_samples) >= 2:
        span = hold_samples[-1][0] - hold_samples[0][0]
        out['hold_stream_hz'] = (len(hold_samples) - 1) / span if span > 0 else float('nan')
        max_drift = 0.0
        for i in range(1, len(hold_samples)):
            _w0, x0, y0, z0 = hold_samples[i - 1]
            _w1, x1, y1, z1 = hold_samples[i]
            drift = math.dist((x0, y0, z0), (x1, y1, z1))
            max_drift = max(max_drift, drift)
        out['max_consecutive_drift_m'] = max_drift
    else:
        out['hold_stream_hz'] = float('nan')
        out['max_consecutive_drift_m'] = float('nan')

    # --- altitude stability during hold.
    if onset_wall:
        onset_alt = None
        alts_during_hold = []
        for wall, z in node.altitude_samples:
            if onset_alt is None and wall >= onset_wall:
                onset_alt = z
            if onset_alt is not None and wall <= onset_wall + HOLD_INJECT_SEC:
                alts_during_hold.append(z)
        if onset_alt is not None and alts_during_hold:
            out['onset_altitude_m'] = onset_alt
            out['max_altitude_drift_m'] = max(abs(z - onset_alt) for z in alts_during_hold)
        else:
            out['onset_altitude_m'] = float('nan')
            out['max_altitude_drift_m'] = float('nan')
    else:
        out['onset_altitude_m'] = float('nan')
        out['max_altitude_drift_m'] = float('nan')

    # --- safety/state + violations evidence of HOLDING.
    out['saw_hold_action'] = any(
        action == 'hold' and not obstacle_clear
        for wall, _level, action, obstacle_clear in node.safety_states
        if onset_wall and wall >= injected_at
    )
    out['saw_obstacle_violation'] = any(
        code == 'OBSTACLE_TOO_CLOSE' for _wall, code, _msg in node.violations
    )

    out['cmd_safety_stopped_after_clear'] = len(report['new_cmd_safety_after_clear']) == 0

    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--no-sim-time', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = ObstacleHoldGate(args.uav_id, use_sim_time=not args.no_sim_time)

    failure = None
    report = {}
    analysis = {}
    try:
        report = run(node, POST_CLEAR_SETTLE_SEC)
        analysis = analyze(node, report)
    except Failure as exc:
        failure = str(exc)
    except KeyboardInterrupt:
        failure = 'interrupted'

    print('')
    print('=== G-S3-HOLD OBSTACLE GATE REPORT (%s) ===' % args.uav_id)
    print('pre_inject_command_count      : %d' % report.get('pre_inject_command_count', -1))
    print('onset_after_injection_sec     : %.3f (budget %.2f = grace %.2f + slack)'
          % (analysis.get('onset_after_injection_sec', float('nan')), MAX_ONSET_SEC, OBSTACLE_GRACE_SEC))
    print('hold_sample_count             : %d' % analysis.get('hold_sample_count', 0))
    print('hold_stream_hz                : %.2f (floor %.1f, nominal %.1f)'
          % (analysis.get('hold_stream_hz', float('nan')), HOLD_STREAM_MIN_HZ, HOLD_STREAM_HZ))
    print('max_consecutive_drift_m       : %.6f (epsilon %.3f)'
          % (analysis.get('max_consecutive_drift_m', float('nan')), FROZEN_SETPOINT_EPSILON_M))
    print('onset_altitude_m              : %.3f' % analysis.get('onset_altitude_m', float('nan')))
    print('max_altitude_drift_m          : %.3f' % analysis.get('max_altitude_drift_m', float('nan')))
    print('saw_hold_action (safety/state): %s' % analysis.get('saw_hold_action'))
    print('saw_obstacle_violation        : %s' % analysis.get('saw_obstacle_violation'))
    clear_result = report.get('clear_result')
    if clear_result is not None:
        print('ClearFault success             : %s' % clear_result.success)
        print('ClearFault remaining_faults    : %s' % list(clear_result.remaining_faults))
    else:
        print('ClearFault success             : NEVER CALLED')
    print('cmd_safety_stopped_after_clear : %s (0 new cmd_safety in %.1fs after ClearFault)'
          % (analysis.get('cmd_safety_stopped_after_clear'), POST_CLEAR_SETTLE_SEC))
    print('resume_accepted                : %s' % report.get('resume_accepted'))
    print('resume_seen_at is not None     : %s (command_selected flowed again, source != SAFETY)'
          % (report.get('resume_seen_at') is not None))
    print('original goto terminal status  : %s (recorded only, S:8 caveat -- not judged)'
          % report.get('orig_goal_status'))
    if failure:
        print('FAILURE                        : %s' % failure)

    onset_ok = (
        not math.isnan(analysis.get('onset_after_injection_sec', float('nan'))) and
        analysis.get('onset_after_injection_sec', 999.0) <= MAX_ONSET_SEC)
    stream_ok = (
        analysis.get('hold_sample_count', 0) >= 3 and
        not math.isnan(analysis.get('hold_stream_hz', float('nan'))) and
        analysis.get('hold_stream_hz', 0.0) >= HOLD_STREAM_MIN_HZ)
    frozen_ok = (
        not math.isnan(analysis.get('max_consecutive_drift_m', float('nan'))) and
        analysis.get('max_consecutive_drift_m', 999.0) < 0.001)
    altitude_ok = (
        not math.isnan(analysis.get('max_altitude_drift_m', float('nan'))) and
        analysis.get('max_altitude_drift_m', 999.0) < 0.20)
    state_ok = analysis.get('saw_hold_action') is True and analysis.get('saw_obstacle_violation') is True
    clear_ok = clear_result is not None and clear_result.success and not list(clear_result.remaining_faults)
    latch_released_ok = analysis.get('cmd_safety_stopped_after_clear') is True
    resume_ok = report.get('resume_seen_at') is not None

    passed = (
        failure is None and onset_ok and stream_ok and frozen_ok and altitude_ok and
        state_ok and clear_ok and latch_released_ok and resume_ok)

    print('')
    print('sub-verdicts: onset=%s stream=%s frozen=%s altitude=%s state=%s clear=%s '
          'latch_released=%s resume=%s'
          % (onset_ok, stream_ok, frozen_ok, altitude_ok, state_ok, clear_ok,
             latch_released_ok, resume_ok))
    print('RESULT                         : %s' % ('PASS' if passed else 'FAIL'))
    sys.stdout.flush()

    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
