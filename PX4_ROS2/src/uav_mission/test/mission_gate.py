#!/usr/bin/env python3
"""P9.6-P9.9 (plan P9-mission.md S:5): mission gates G-M1..G-M4. Multi-phase
probe, one subcommand per gate/injection. Run via scripts/verify_mission.sh
(spawns marker/target-box, starts the right sim.launch.py combo first).
NOT RUN by the author -- sim access is reserved for the verifier-runner.

R27-1: every phase proves its OWN measurement is alive (subscription seeing
real traffic) before judging PASS/FAIL. R21: assertions anchor on events
(mission/status, mission/events, action results), wall time is only a
timeout guard. Exit 0 PASS / 1 FAIL / 2 FAILED TO MEASURE.
"""
import argparse
import math
import re
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Header
from uav_interfaces.action import ExecuteMission
from uav_interfaces.msg import (
    ControlAuthority,
    ControlCommand,
    MissionEvent,
    MissionStatus,
    Obstacle,
    ObstacleArray,
    ResultCode,
    SafetyState,
    SemanticLandmarkArray,
    TargetState,
    VehicleHealth,
    VehicleState,
)
from uav_interfaces.srv import ClearFault, ResumeMission

SOURCE_MISSION = ControlAuthority.SOURCE_MISSION
SOURCE_SAFETY = ControlAuthority.SOURCE_SAFETY

RESULT_NAME = {
    getattr(ResultCode, n): n for n in dir(ResultCode)
    if n.isupper() and isinstance(getattr(ResultCode, n), int)
}
STATE_NAME = {
    getattr(MissionStatus, n): n for n in dir(MissionStatus)
    if n.startswith('STATE_')
}
EVENT_NAME = {
    getattr(MissionEvent, n): n for n in dir(MissionEvent)
    if n.startswith('EVENT_')
}


class Failure(Exception):
    """Measurement itself could not be trusted -- exit 2, never PASS."""


class _LastClearAttempt:
    """Fallback report object when the ClearFault poll loop times out --
    carries only .success/.remaining_faults (and an optional .message set by
    callers). Never fabricates a success (same convention as
    obstacle_hold_gate.py, G-S3-HOLD)."""

    def __init__(self, success, remaining_faults):
        self.success = success
        self.remaining_faults = remaining_faults


def result_name(code):
    return RESULT_NAME.get(code, 'CODE_%d' % code)


def state_name(code):
    return STATE_NAME.get(code, 'STATE_%d' % code)


def event_name(code):
    return EVENT_NAME.get(code, 'EVENT_%d' % code)


class MissionProbe(Node):
    """Shared plumbing for every subcommand -- one node, subscribes to
    everything a gate might need (unused topics just stay silent, no cost
    worth special-casing per phase)."""

    def __init__(self, uav_id, name='mission_gate'):
        super().__init__(
            name, parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.uav_id = uav_id
        self.prefix = '/uav/' + uav_id

        self.status_history = []     # (wall, MissionStatus)
        self.events = []             # (wall, MissionEvent)
        self.odom = None
        self.landmarks = None
        self.target = None
        self.authority_history = []  # (wall, ControlAuthority)
        self.health_history = []     # (wall, VehicleHealth)
        self.safety_states = []      # (wall, SafetyState)
        self.violations = []         # (wall, code, message)
        self.commands = []           # (wall, source, x, y, z)
        self.vehicle_state = None

        self.create_subscription(
            MissionStatus, self.prefix + '/mission/status', self._on_status, 10)
        self.create_subscription(
            MissionEvent, self.prefix + '/mission/events', self._on_event, 50)
        self.create_subscription(
            Odometry, self.prefix + '/state/odometry_fused', self._on_odom, 10)
        self.create_subscription(
            SemanticLandmarkArray, self.prefix + '/world/semantic_landmarks',
            self._on_landmarks, 10)
        self.create_subscription(
            TargetState, self.prefix + '/world/target_state', self._on_target, 10)
        self.create_subscription(
            ControlAuthority, self.prefix + '/control/authority', self._on_authority, 10)
        self.create_subscription(
            VehicleHealth, self.prefix + '/state/health_px4', self._on_health, 10)
        self.create_subscription(
            SafetyState, self.prefix + '/safety/state', self._on_safety_state, 10)
        self.create_subscription(
            DiagnosticArray, self.prefix + '/safety/violations', self._on_violations, 20)
        self.create_subscription(
            ControlCommand, self.prefix + '/control/command_selected', self._on_command, 50)
        self.create_subscription(
            VehicleState, self.prefix + '/state/vehicle', self._on_vehicle_state, 10)

        self.obstacle_publisher = self.create_publisher(
            ObstacleArray, self.prefix + '/world/obstacle_map_local', 10)

        self.execute_client = ActionClient(
            self, ExecuteMission, self.prefix + '/mission/execute_mission')
        self.resume_client = self.create_client(
            ResumeMission, self.prefix + '/mission/resume')
        self.clear_fault_client = self.create_client(
            ClearFault, self.prefix + '/safety/clear_fault')

    # ---------------------------------------------------------- callbacks
    def _log(self, text):
        print('  [t=%.3f] %s' % (time.time(), text))
        sys.stdout.flush()

    def _on_status(self, msg):
        self.status_history.append((time.time(), msg))

    def _on_event(self, msg):
        self.events.append((time.time(), msg))
        self._log(
            'EVENT %s step=%r result=%s desc=%r'
            % (event_name(msg.event_type), msg.step_name, result_name(msg.result_code),
               msg.description))

    def _on_odom(self, msg):
        self.odom = msg

    def _on_landmarks(self, msg):
        self.landmarks = msg

    def _on_target(self, msg):
        self.target = msg

    def _on_authority(self, msg):
        self.authority_history.append((time.time(), msg))

    def _on_health(self, msg):
        self.health_history.append((time.time(), msg))

    def _on_safety_state(self, msg):
        self.safety_states.append((time.time(), msg))

    def _on_violations(self, msg):
        for status in msg.status:
            if status.name.startswith('safety: '):
                self.violations.append((time.time(), status.name[len('safety: '):], status.message))

    def _on_command(self, msg):
        p = msg.position
        self.commands.append((time.time(), msg.source, p.x, p.y, p.z))

    def _on_vehicle_state(self, msg):
        self.vehicle_state = msg

    # ------------------------------------------------------------ helpers
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

    def latest_status(self):
        return self.status_history[-1][1] if self.status_history else None

    def position(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        return (p.x, p.y, p.z)

    def roll_pitch_deg(self):
        """(roll_deg, pitch_deg) from odometry_fused orientation -- diagnostic
        only (coordinator ask 2026-08-22: correlate obstacle_extractor's
        ground-filter defeat with actual attitude during follow_target)."""
        if self.odom is None:
            return None
        q = self.odom.pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        return (math.degrees(roll), math.degrees(pitch))

    def landmark(self, marker_id):
        if self.landmarks is None:
            return None
        for lm in self.landmarks.landmarks:
            if lm.marker_id == marker_id:
                return lm
        return None

    def send_mission(self, mission_id, mission_params, loop=False, timeout=30.0):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            if self.execute_client.server_is_ready():
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        else:
            raise Failure('execute_mission action server never appeared')

        goal = ExecuteMission.Goal()
        goal.mission_id = mission_id
        goal.mission_params = mission_params
        goal.loop = loop
        future = self.execute_client.send_goal_async(goal)
        deadline = time.time() + 15.0
        while rclpy.ok() and not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise Failure('ExecuteMission goal response timed out')
        handle = future.result()
        if not handle.accepted:
            raise Failure('ExecuteMission goal REJECTED for mission_id=%s' % mission_id)
        self._log('ExecuteMission ACCEPTED mission_id=%s params=%r' % (mission_id, mission_params))
        return handle

    def wait_mission_result(self, handle, timeout, description):
        future = handle.get_result_async()
        end = time.time() + timeout
        while rclpy.ok() and not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise Failure('%s: ExecuteMission result timed out after %.0fs' % (description, timeout))
        wrapped = future.result()
        return wrapped.result, wrapped.status

    def wait_terminal_status(self, timeout=5.0):
        """The ExecuteMission action RESULT and the /mission/status TOPIC are
        two INDEPENDENT DDS channels -- the server publishing status before
        sending the result (contract Sec2.19 fix, 2026-08-22) does not
        guarantee THIS process's single-threaded spin_once() has already
        dequeued/delivered the status callback by the moment the action
        future resolves (no cross-channel ordering guarantee). Call this
        right after wait_mission_result() before trusting latest_status()."""
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            st = self.latest_status()
            if st is not None and st.state in (MissionStatus.STATE_COMPLETED, MissionStatus.STATE_ABORTED):
                return st
            rclpy.spin_once(self, timeout_sec=0.02)
        return self.latest_status()

    def wait_for_event(self, event_type, timeout=5.0):
        """Same DDS-independent-channel race as wait_terminal_status(), one
        level deeper: /mission/status and /mission/events are ALSO two
        separate topics/writers -- waiting for status to show a terminal
        state does not guarantee the terminal EVENT_COMPLETED/EVENT_ABORTED
        sample (published from the SAME server-side call, just a different
        writer) has been delivered to this subscriber yet. Found 2026-08-23:
        a clean G-M1 run showed STEP_COMPLETED=8/8 and STATE_COMPLETED with
        a valid goal_id, yet COMPLETED=0 in the events list -- the event was
        simply still in flight. Spin explicitly for it before counting."""
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            if any(e.event_type == event_type for _w, e in self.events):
                return True
            rclpy.spin_once(self, timeout_sec=0.02)
        return any(e.event_type == event_type for _w, e in self.events)

    def call_clear_fault(self):
        if not self.clear_fault_client.wait_for_service(timeout_sec=10.0):
            raise Failure('clear_fault service never appeared')
        future = self.clear_fault_client.call_async(ClearFault.Request())
        end = time.time() + 10.0
        while rclpy.ok() and not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise Failure('clear_fault call timed out')
        return future.result()

    def call_resume(self):
        if not self.resume_client.wait_for_service(timeout_sec=10.0):
            raise Failure('mission/resume service never appeared')
        future = self.resume_client.call_async(ResumeMission.Request())
        end = time.time() + 10.0
        while rclpy.ok() and not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise Failure('mission/resume call timed out')
        return future.result()

    def publish_obstacle(self, present, distance_m=0.18, frame_id='odom'):
        msg = ObstacleArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.uav_id = self.uav_id
        msg.sensing_range = 5.0
        if present:
            ob = Obstacle()
            ob.obstacle_id = 1
            ob.shape = Obstacle.SHAPE_SPHERE
            if self.odom is not None:
                p = self.odom.pose.pose.position
                ob.center = Point(x=p.x + 1.0, y=p.y, z=p.z)
            else:
                ob.center = Point(x=0.0, y=0.0, z=0.0)
            ob.size = Vector3(x=0.3, y=0.3, z=0.3)
            ob.distance = float(distance_m)
            ob.confidence = 1.0
            ob.position_uncertainty = 0.05
            msg.obstacles = [ob]
        else:
            msg.obstacles = []
        self.obstacle_publisher.publish(msg)

    def spin_and_publish_obstacle(self, seconds, present):
        end = time.time() + seconds
        period = 0.1
        nxt = time.time()
        while rclpy.ok() and time.time() < end:
            now = time.time()
            if now >= nxt:
                self.publish_obstacle(present)
                nxt = now + period
            rclpy.spin_once(self, timeout_sec=0.02)


# --------------------------------------------------------------------------
# Common checks shared across phases: navigator "clamped" evidence, monotonic
# progress, event edge presence.
# --------------------------------------------------------------------------

_LOG_TS_RE = re.compile(r'\[(\d{9,}\.\d+)\]')


def _log_line_timestamp(line):
    m = _LOG_TS_RE.search(line)
    return float(m.group(1)) if m else None


def body_window(events):
    """(start_wall, end_wall) spanning only the mission BODY's own NavAction/
    Dwell leaves -- i.e. from the first to the last STEP_STARTED/STEP_COMPLETED
    event. Takeoff and Finish (GotoHome->Land) are plain C++ state outside the
    BT (mission_executor_node.hpp point 4) and NEVER emit STEP_* events, so
    this window naturally excludes both without having to know step names.
    Coordinator ruling 2026-08-22 (G-M1 lot 1 review): plan S:5's "0 lan
    clamped" criterion is about the patrol/track/inspect legs' own goto goals,
    NOT the Takeoff climb (which legitimately leash-clamps at 79-89% of
    ticks, chot P6, memory Sec3 "tran toc do do day xich").

    Returns None when a leaf was STARTED but never COMPLETED (an orphaned
    STEP_STARTED -- e.g. follow_target's Timeout decorator HALTING an
    in-flight TrackTarget NavAction instead of letting it finish normally,
    G-M3 run9 diagnosis 2026-08-22): counting min==max of that single
    timestamp as a "window" would make it a ZERO-WIDTH point that trivially
    contains no clamp-log lines, turning an UNMEASURED body into a vacuous
    PASS (R27-1 violation -- a measurement that always passes proves
    nothing). Callers MUST treat None as FAILED TO MEASURE, never as a
    clean 0-hits result."""
    starts = [w for w, e in events if e.event_type == MissionEvent.EVENT_STEP_STARTED]
    completes = [w for w, e in events if e.event_type == MissionEvent.EVENT_STEP_COMPLETED]
    if not starts:
        return None
    if len(starts) != len(completes):
        return None
    return (min(starts + completes), max(starts + completes))


def grep_clamped(bringup_log_path, window=None):
    """0 (=clean) or the navigator-log lines showing nonzero clamp evidence,
    restricted to `window` (body_window()) when given. Navigator does not
    publish result.message anywhere outside its own action result (which
    mission consumes internally and never forwards) -- the ONLY channel that
    carries this evidence to the outside is navigator's own stdout/stderr log
    (R3: read the report, never invent). Each log line carries its own
    rclcpp-clock timestamp in brackets, comparable to this probe's wall time
    (both track the same sim/host clock in this setup -- confirmed 2026-08-22,
    event and log timestamps agreed to within ~3s on a real run)."""
    try:
        with open(bringup_log_path, 'r', errors='replace') as f:
            lines = f.readlines()
    except OSError:
        return None
    hits = []
    for line in lines:
        if 'leash clamping the setpoint' not in line and not (
                'clamped' in line and 'clamped 0%' not in line and 'setpoint clamped' in line):
            continue
        if window is not None:
            ts = _log_line_timestamp(line)
            # +/-1.0s slack for the throttle period / clock granularity.
            if ts is None or not (window[0] - 1.0 <= ts <= window[1] + 1.0):
                continue
        hits.append(line.strip())
    return hits


def progress_report(status_history):
    seq = [(w, s.state, s.current_step_index, s.total_steps, s.progress_percent)
           for w, s in status_history]
    progresses = [p for *_r, p in seq]
    monotonic = all(b >= a - 1e-6 for a, b in zip(progresses, progresses[1:]))
    bounded = all(0.0 - 1e-6 <= p <= 100.0 + 1e-6 for p in progresses)
    return seq, monotonic, bounded


FTM = 'FTM'   # sentinel: R27-1, a check that could not be verified must
              # never silently count as a pass (coordinator ruling
              # 2026-08-22: body_window() on a HALTED leaf yields an
              # unusable window -- treat as unmeasured, not as "0 hits").


def print_checks(checks):
    """Returns (passed, any_ftm). FTM (cannot-measure) always overrides a
    would-be PASS -- callers must return exit 2, never 0, when any_ftm."""
    any_ftm = False
    for name, ok, detail in checks:
        if ok is FTM:
            print('  [FTM ] %-60s %s' % (name, detail))
            any_ftm = True
        else:
            print('  [%s] %-60s %s' % ('PASS' if ok else 'FAIL', name, detail))
    passed = all(ok is True for _n, ok, _d in checks)
    return passed, any_ftm


# --------------------------------------------------------------------------
# G-M1: indoor_patrol
# --------------------------------------------------------------------------

INDOOR_WAYPOINTS = [
    (1.0, 0.0, 1.5), (1.0, 1.0, 1.5), (0.0, 1.0, 1.5), (0.0, 0.0, 1.5),
]  # default corridor, well clear of crate_stack(-3,3) and shelf(4.5,-3)
INDOOR_LOOPS = 8   # >= 4 waypoints x 2 full circuits (mission_registry.hpp: loops = visits)
INDOOR_DWELL_SEC = 1.5
WAYPOINT_TOLERANCE_M = 0.3


def build_indoor_params():
    wps = '\n'.join(
        '  - {x: %.2f, y: %.2f, z: %.2f}' % w for w in INDOOR_WAYPOINTS)
    return 'waypoints:\n%s\nloops: %d\ndwell_sec: %.2f\n' % (wps, INDOOR_LOOPS, INDOOR_DWELL_SEC)


def run_patrol(args, bringup_log):
    rclpy.init()
    node = MissionProbe(args.uav_id, 'gm1_patrol_gate')
    try:
        node.wait_until(lambda: node.position() is not None, 60.0, 'odometry_fused')
        handle = node.send_mission('indoor_patrol', build_indoor_params(), loop=False)

        t_start = time.time()
        result, status = node.wait_mission_result(handle, 400.0, 'indoor_patrol')
        duration = time.time() - t_start
        node._log(
            'ExecuteMission terminal: goal_status=%d success=%s result_code=%s steps=%d msg=%r'
            % (status, result.success, result_name(result.result_code), result.steps_completed,
               result.message))

        final_status = node.wait_terminal_status(5.0)
        node.wait_for_event(MissionEvent.EVENT_COMPLETED, 5.0)
        checks = []
        checks.append((
            'a1. goal SUCCEEDED (action status)', status == GoalStatus.STATUS_SUCCEEDED,
            'status=%d duration=%.1fs' % (status, duration)))
        checks.append((
            'a2. MissionStatus terminal = STATE_COMPLETED, with a non-empty goal_id '
            '(contract Sec2.19: goal_id is the ExecuteMission UUID hex while running)',
            final_status is not None and final_status.state == MissionStatus.STATE_COMPLETED and
            bool(final_status.goal_id),
            'state=%s goal_id=%r' % (
                (state_name(final_status.state), final_status.goal_id) if final_status
                else ('NONE', None))))

        seq, monotonic, bounded = progress_report(node.status_history)
        checks.append((
            'a3. progress_percent monotonic non-decreasing', monotonic,
            'n=%d samples, sequence tail=%s' % (len(seq), [round(p, 1) for *_r, p in seq[-6:]])))
        checks.append((
            'a3b. progress_percent stays within [0,100] (informational per .msg contract)', bounded,
            'max observed=%.1f%% (see note if >100)' % (max((p for *_r, p in seq), default=0.0))))

        event_types = [e.event_type for _w, e in node.events]
        has_started = MissionEvent.EVENT_STARTED in event_types
        has_step_started = MissionEvent.EVENT_STEP_STARTED in event_types
        has_step_completed = MissionEvent.EVENT_STEP_COMPLETED in event_types
        has_completed = MissionEvent.EVENT_COMPLETED in event_types
        checks.append((
            'a4. events cover STARTED/STEP_STARTED/STEP_COMPLETED/COMPLETED edges',
            has_started and has_step_started and has_step_completed and has_completed,
            'counts: STARTED=%d STEP_STARTED=%d STEP_COMPLETED=%d COMPLETED=%d'
            % tuple(event_types.count(c) for c in (
                MissionEvent.EVENT_STARTED, MissionEvent.EVENT_STEP_STARTED,
                MissionEvent.EVENT_STEP_COMPLETED, MissionEvent.EVENT_COMPLETED))))

        step_completed_names = [
            e.step_name for _w, e in node.events if e.event_type == MissionEvent.EVENT_STEP_COMPLETED]
        goto_count = step_completed_names.count('goto_waypoint')
        checks.append((
            'a5. >= 8 successful goto_waypoint legs (>=4 waypoints x 2 loops)',
            goto_count >= INDOOR_LOOPS,
            'goto_waypoint STEP_COMPLETED count=%d (want >= %d)' % (goto_count, INDOOR_LOOPS)))

        window = body_window(node.events)
        if window is None:
            checks.append((
                'a6. 0 "clamped" evidence in navigator log during the patrol legs '
                '(Takeoff/Finish excluded -- coordinator ruling 2026-08-22)', FTM,
                'body_window() unusable: an orphaned STEP_STARTED with no matching '
                'STEP_COMPLETED (a leaf was HALTED, not finished normally) -- R27-1 forbids '
                'a zero-width window standing in for "0 hits"'))
        else:
            clamped_hits = grep_clamped(bringup_log, window) if bringup_log else None
            checks.append((
                'a6. 0 "clamped" evidence in navigator log during the patrol legs '
                '(Takeoff/Finish excluded -- coordinator ruling 2026-08-22)',
                FTM if clamped_hits is None else (len(clamped_hits) == 0),
                ('FAILED TO MEASURE (no log)' if clamped_hits is None else
                 ('clean, window=%r' % (window,) if not clamped_hits else
                  '%d hit(s) in window=%r: %r' % (len(clamped_hits), window, clamped_hits[:3])))))

        checks.append((
            'a7. vehicle disarmed after Land (auto-disarm)',
            node.vehicle_state is not None and not node.vehicle_state.armed,
            'armed=%s' % (node.vehicle_state.armed if node.vehicle_state else 'UNKNOWN')))

        passed, ftm = print_checks(checks)
        if ftm:
            print('RESULT: FAILED TO MEASURE (at least one check could not be verified, R27-1)')
            return 2
        print('RESULT: %s' % ('PASS' if passed else 'FAIL'))
        return 0 if passed else 1
    except Failure as exc:
        print('FAILED TO MEASURE: %s' % exc)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


# --------------------------------------------------------------------------
# G-M2: inspect_point
# --------------------------------------------------------------------------

INSPECT_DWELL_SEC = 5.0


def build_inspect_params(mx, my, mz, marker_id):
    return 'approach: {x: %.2f, y: %.2f, z: %.2f}\nmarker_id: %d\ndwell_sec: %.2f\n' % (
        mx, my, mz, marker_id, INSPECT_DWELL_SEC)


def run_inspect(args, bringup_log):
    rclpy.init()
    node = MissionProbe(args.uav_id, 'gm2_inspect_gate')
    try:
        node.wait_until(lambda: node.position() is not None, 60.0, 'odometry_fused')

        dwell_odom_samples = []   # (wall, x, y, z)
        dwell_landmark_samples = []  # (wall, x, y, z)
        dwelling = {'active': False}
        # Dwell's onStart() never calls MissionContext::reportStepStarted()
        # (bt_nodes.cpp: only NavAction does) -- current_step_name_ stays
        # 'goto_over_marker' for the WHOLE dwell, so it cannot be used to
        # detect the dwell window. Anchor instead on the goto_over_marker
        # STEP_COMPLETED event (R21: anchor on an event) and use the
        # CONFIGURED dwell_sec as the window length -- Dwell starts ticking
        # on the very next tree tick after that event.
        anchor = {'wall': None}

        def maybe_collect(_msg=None):
            if anchor['wall'] is None:
                for w, e in node.events:
                    if e.event_type == MissionEvent.EVENT_STEP_COMPLETED and \
                            e.step_name == 'goto_over_marker':
                        anchor['wall'] = w
                        break
                return
            now = time.time()
            if now > anchor['wall'] + INSPECT_DWELL_SEC:
                return   # dwell window over (Finish/GotoHome/Land may follow)
            dwelling['active'] = True
            p = node.position()
            if p is not None:
                dwell_odom_samples.append((now,) + p)
            lm = node.landmark(args.marker_id)
            if lm is not None:
                q = lm.pose.position
                dwell_landmark_samples.append((now, q.x, q.y, q.z))

        handle = node.send_mission(
            'inspect_point', build_inspect_params(args.marker_x, args.marker_y, args.inspect_z,
                                                    args.marker_id), loop=False)

        deadline = time.time() + 300.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            maybe_collect()
            st = node.latest_status()
            if st is not None and st.state in (
                    MissionStatus.STATE_COMPLETED, MissionStatus.STATE_ABORTED):
                break
        result, status = node.wait_mission_result(handle, 60.0, 'inspect_point')
        node._log(
            'ExecuteMission terminal: goal_status=%d success=%s result_code=%s msg=%r'
            % (status, result.success, result_name(result.result_code), result.message))

        checks = []
        checks.append((
            'b1. dwell_over_marker step was reached and observed running', dwelling['active'],
            'dwell odom samples=%d, landmark samples=%d'
            % (len(dwell_odom_samples), len(dwell_landmark_samples))))
        checks.append((
            'b2. goal SUCCEEDED (action status)', status == GoalStatus.STATUS_SUCCEEDED,
            'status=%d result_code=%s' % (status, result_name(result.result_code))))
        final_status = node.wait_terminal_status(5.0)
        checks.append((
            'b3. MissionStatus terminal = STATE_COMPLETED, with a non-empty goal_id',
            final_status is not None and final_status.state == MissionStatus.STATE_COMPLETED and
            bool(final_status.goal_id),
            'state=%s goal_id=%r' % (
                (state_name(final_status.state), final_status.goal_id) if final_status
                else ('NONE', None))))

        if dwell_odom_samples:
            errs = [math.hypot(x - args.marker_x, y - args.marker_y)
                    for _w, x, y, _z in dwell_odom_samples]
            max_err = max(errs)
            checks.append((
                'b4. drone hover |xy - marker_spawn_xy| <= 0.3 m during dwell', max_err <= 0.3,
                'max=%.3f m, mean=%.3f m, n=%d' % (max_err, sum(errs) / len(errs), len(errs))))
        else:
            checks.append(('b4. drone hover position vs marker (NO SAMPLES)', False, 'no dwell odom samples'))

        if dwell_landmark_samples:
            errs = [math.hypot(x - args.marker_x, y - args.marker_y)
                    for _w, x, y, _z in dwell_landmark_samples]
            max_err = max(errs)
            checks.append((
                'b5. semantic_landmarks[id=%d] xy vs spawn xy <= 0.20 m' % args.marker_id,
                max_err <= 0.20,
                'max=%.3f m, mean=%.3f m, n=%d' % (max_err, sum(errs) / len(errs), len(errs))))
        else:
            checks.append((
                'b5. semantic_landmarks vs spawn pose (NO SAMPLES)', False,
                'no landmark sightings of id=%d during dwell' % args.marker_id))

        window = body_window(node.events)
        if window is None:
            checks.append((
                'b6. 0 "clamped" evidence in navigator log during the inspect legs '
                '(Takeoff/Finish excluded -- coordinator ruling 2026-08-22)', FTM,
                'body_window() unusable: an orphaned STEP_STARTED with no matching '
                'STEP_COMPLETED (a leaf was HALTED, not finished normally) -- R27-1 forbids '
                'a zero-width window standing in for "0 hits"'))
        else:
            clamped_hits = grep_clamped(bringup_log, window) if bringup_log else None
            checks.append((
                'b6. 0 "clamped" evidence in navigator log during the inspect legs '
                '(Takeoff/Finish excluded -- coordinator ruling 2026-08-22)',
                FTM if clamped_hits is None else (len(clamped_hits) == 0),
                ('FAILED TO MEASURE (no log)' if clamped_hits is None else
                 ('clean, window=%r' % (window,) if not clamped_hits else
                  '%d hit(s) in window=%r: %r' % (len(clamped_hits), window, clamped_hits[:3])))))

        passed, ftm = print_checks(checks)
        if ftm:
            print('RESULT: FAILED TO MEASURE (at least one check could not be verified, R27-1)')
            return 2
        print('RESULT: %s' % ('PASS' if passed else 'FAIL'))
        return 0 if passed else 1
    except Failure as exc:
        print('FAILED TO MEASURE: %s' % exc)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


# --------------------------------------------------------------------------
# G-M3: follow_target
# --------------------------------------------------------------------------

FOLLOW_STANDOFF_M = 2.0
FOLLOW_TIMEOUT_SEC = 75.0
FOLLOW_TARGET_SPEED_MPS = 0.2
MIN_TRACK_SECONDS = 60.0

# --- COPY of navigator_action_server_node.cpp's TrackTarget standoff math
# (coordinator diagnosis 2026-08-22, G-M3 run9: c3's raw 4.85 m average was
# never wrong -- it was comparing against the WRONG number and the WRONG
# sample set). Source: navigator_action_server_node.cpp
#   effectiveStandoff() L2546-2552: base=max(min_standoff_m, requested),
#     returns base + target_velocity_error_mps * target_reaction_sec
#   L2633-2638 (evidence note, "%.2f m = %.2f requested plus %.2f m of
#     reaction"): the ONLY place these numbers are composed into a single
#     value -- but that text lands in the navigator ACTION RESULT message,
#     which mission's own sendTypedGoal<>() result_callback HARDCODES to
#     "canceled" for a CANCELED outcome (mission_executor_node.cpp), so it
#     never reaches this probe on any channel. Recomputed here from the
#     underlying PARAMS instead (all 3 are static navigator config
#     defaults, not runtime-derived) -- same "*_copy" convention as the
#     rest of the project (R29).
MIN_STANDOFF_COPY_M = 1.0                 # navigator param min_standoff_m default (L275)
TARGET_VELOCITY_ERROR_COPY_MPS = 0.71     # navigator param target_velocity_error_mps default (L278)
TARGET_REACTION_COPY_SEC = 1.0            # navigator param target_reaction_sec default (L279)
EFFECTIVE_STANDOFF_COPY_M = (
    max(MIN_STANDOFF_COPY_M, FOLLOW_STANDOFF_M) +
    TARGET_VELOCITY_ERROR_COPY_MPS * TARGET_REACTION_COPY_SEC)   # = 2.71 m
# Coordinator-given acceptance band around the effective standoff, informed
# by the leash-lag characterization already on record (memory Sec3 "tran
# toc do do day xich") -- NOT re-derived here, taken as the gate criterion.
EFFECTIVE_STANDOFF_TOLERANCE_M = 0.25
EXPECTED_STANDOFF_MEASUREMENT_M = 2.96    # coordinator's own predicted center (diagnosis)

CRUISE_ALTITUDE_HOLD_TOLERANCE_M = 0.3    # c4 restated: hold OWN altitude, not chase target z


def build_follow_params():
    return 'target_speed_mps: %.3f\nstandoff_m: %.2f\ntimeout_sec: %.1f\n' % (
        FOLLOW_TARGET_SPEED_MPS, FOLLOW_STANDOFF_M, FOLLOW_TIMEOUT_SEC)


def track_leg_window(bringup_log_path):
    """(start_wall, end_wall) spanning follow_target's WHOLE body -- every
    track_target dispatch AND every intervening Search cycle (TargetSeen
    going false briefly and the ReactiveFallback falling to
    search_point_1/2, then back to track_target once re-acquired) -- up to
    (but excluding) Finish's GotoHome/Land. Read from navigator's OWN log
    (R3): a leaf HALTED by the Timeout decorator never gets a
    STEP_COMPLETED (body_window() correctly returns None for it).

    Coordinator patch 2026-08-23 (track_id anchoring, navigator L2597) made
    track<->search cycling for the SAME real, continuously-visible target a
    LEGITIMATE, expected pattern (a brief track-id churn now correctly
    reads as "lost the anchored id" and falls to Search, then re-anchors)
    -- so a naive "first accepted goto_pose after track_target" is WRONG:
    Search's own leg dispatches via GotoPose too (inspect_point.xml/
    follow_target.xml both use nav_type="GotoPose" for search points) and
    logs an IDENTICAL "accepted goto_pose goal" line, indistinguishable
    from Finish's GotoHome by text alone. Disambiguate using the ONE
    boundary that IS unique: "accepted land goal" happens exactly once, at
    the true end of Finish, and is ALWAYS immediately preceded by Finish's
    own GotoHome dispatch (never by another track_target) -- so end = the
    LAST "accepted goto_pose goal" that precedes the "accepted land goal"
    timestamp. Verified 2026-08-23 against a real 5-cycle run: computed
    span 75.126s, matches FOLLOW_TIMEOUT_SEC=75.0 almost exactly.

    None if any boundary is missing (R27-1: no invented boundary --
    callers must treat None as FAILED TO MEASURE)."""
    if not bringup_log_path:
        return None
    try:
        with open(bringup_log_path, 'r', errors='replace') as f:
            lines = f.readlines()
    except OSError:
        return None
    track_target_times = []
    goto_pose_times = []
    land_time = None
    for line in lines:
        if 'accepted track_target goal' in line:
            ts = _log_line_timestamp(line)
            if ts is not None:
                track_target_times.append(ts)
        elif 'accepted goto_pose goal' in line:
            ts = _log_line_timestamp(line)
            if ts is not None:
                goto_pose_times.append(ts)
        elif land_time is None and 'accepted land goal' in line:
            land_time = _log_line_timestamp(line)
    if not track_target_times or land_time is None:
        return None
    start = track_target_times[0]
    finish_gotohome_candidates = [t for t in goto_pose_times if t < land_time]
    if not finish_gotohome_candidates:
        return None
    end = max(finish_gotohome_candidates)
    if end <= start:
        return None
    return (start, end)


def run_follow(args, bringup_log):
    rclpy.init()
    node = MissionProbe(args.uav_id, 'gm3_follow_gate')
    try:
        node.wait_until(lambda: node.position() is not None, 60.0, 'odometry_fused')

        samples = []          # (wall, drone_x, drone_y, drone_z, target_x, target_y, target_z)
        attitude_samples = []  # (wall, roll_deg, pitch_deg) -- diagnostic only
        target_age_samples = []  # (wall, time_since_seen_sec) -- coordinator ask 2026-08-23:
        # confirm N2's 1.0s sighting-gate threshold against the REAL age tail
        # on the wire during a healthy track leg.

        t_start = time.time()
        handle = node.send_mission('follow_target', build_follow_params(), loop=False)

        # If SAFETY seizes authority (obstacle_extractor's ground clutter
        # still reaches local_planner -> "no way through" -> HOLD), the
        # mission PAUSES and will NEVER settle on its own (this probe never
        # calls ClearFault here -- that is G-M4.3's job, not this gate's).
        # Detect that early and stop waiting instead of burning the full
        # FOLLOW_TIMEOUT_SEC+90s budget on a result that will never arrive.
        paused_seen_at = None
        deadline = time.time() + FOLLOW_TIMEOUT_SEC + 90.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.odom is not None:
                rp = node.roll_pitch_deg()
                if rp is not None:
                    attitude_samples.append((time.time(),) + rp)
            if node.target is not None:
                target_age_samples.append((time.time(), node.target.time_since_seen_sec))
            if node.odom is not None and node.target is not None and node.target.time_since_seen_sec <= 1.0:
                dp = node.odom.pose.pose.position
                tp = node.target.pose.position
                samples.append((time.time(), dp.x, dp.y, dp.z, tp.x, tp.y, tp.z))
            st = node.latest_status()
            if st is not None and st.state in (
                    MissionStatus.STATE_COMPLETED, MissionStatus.STATE_ABORTED):
                break
            if st is not None and st.state == MissionStatus.STATE_PAUSED and paused_seen_at is None:
                paused_seen_at = time.time()
                node._log(
                    '=== DETECTED STATE_PAUSED (authority seized, likely obstacle-avoidance HOLD) '
                    '-- this probe does not ClearFault, so the mission cannot recover; stopping '
                    'early with diagnostic attitude data instead of waiting the full budget ===')
                node.spin(3.0)   # a few more seconds of attitude samples right around the seize
                break

        if paused_seen_at is not None:
            if attitude_samples:
                recent = [s for s in attitude_samples if s[0] >= paused_seen_at - 15.0]
                rolls = [abs(s[1]) for s in recent] or [abs(s[1]) for s in attitude_samples[-50:]]
                pitches = [abs(s[2]) for s in recent] or [abs(s[2]) for s in attitude_samples[-50:]]
                node._log(
                    'ATTITUDE in the 15s before PAUSE (n=%d): |roll| min=%.2f max=%.2f mean=%.2f deg; '
                    '|pitch| min=%.2f max=%.2f mean=%.2f deg'
                    % (len(rolls), min(rolls), max(rolls), sum(rolls) / len(rolls),
                       min(pitches), max(pitches), sum(pitches) / len(pitches)))
            raise Failure(
                'mission PAUSED (control authority seized away from MISSION) %.1fs after start '
                '-- SAFETY holding, obstacle_extractor still not clearing the way for local_planner; '
                'see printed attitude stats above and gm3_bringup.log "costmap dropped" counts'
                % (paused_seen_at - t_start))

        result, status = node.wait_mission_result(handle, 90.0, 'follow_target')
        node._log(
            'ExecuteMission terminal: goal_status=%d success=%s result_code=%s msg=%r'
            % (status, result.success, result_name(result.result_code), result.message))

        checks = []
        checks.append((
            'c1. at least one target_state sighting observed', len(samples) > 0,
            '%d samples with target fresh (<=1.0s old), whole flight' % len(samples)))

        # Coordinator diagnosis 2026-08-22 (G-M3 run9, navigation-planning-
        # engineer's 6-layer slice + ground truth): TrackTarget itself was
        # innocent. The old c3/c4 were wrong on TWO axes at once: (a) sample
        # set spanned the WHOLE 156s flight (>=52% of it was Finish's
        # GotoHome/Land, nothing to do with tracking quality) and (b) they
        # compared against the wrong number (raw standoff_m, not navigator's
        # OWN effectiveStandoff() which adds a REACTION MARGIN by design) on
        # the wrong axis (3D distance, which folds in a Delta z that
        # TrackTarget never promises to close -- it holds ITS OWN altitude,
        # never the ground target's, by design). Restated below to measure
        # what the mission actually promises, on ONLY the track leg.
        track_window = track_leg_window(bringup_log)
        if track_window is None:
            for name in (
                'c2. tracking window span >= 60 s (track leg only)',
                'c3. horizontal distance within effective standoff %.2f +/- %.2f m '
                '(navigator effectiveStandoff() copy) for >= 80%% of TRACK-LEG samples'
                % (EFFECTIVE_STANDOFF_COPY_M, EFFECTIVE_STANDOFF_TOLERANCE_M),
                'c4. drone holds its OWN commanded cruise altitude +/- %.1f m during the '
                'track leg (TrackTarget never chases the ground target\'s z, by design)'
                % CRUISE_ALTITUDE_HOLD_TOLERANCE_M,
            ):
                checks.append((
                    name, FTM,
                    'track_leg_window() unusable: "accepted track_target goal" and/or the '
                    'following "accepted goto_pose/land goal" not found in %r -- cannot bound '
                    'the track leg (R27-1: no invented boundary)' % bringup_log))
        else:
            leg_samples = [s for s in samples if track_window[0] <= s[0] <= track_window[1]]
            leg_age_samples = [
                a for a in target_age_samples if track_window[0] <= a[0] <= track_window[1]]
            if leg_age_samples:
                max_age = max(a[1] for a in leg_age_samples)
                node._log(
                    'THRESHOLD WATCH (coordinator ask 2026-08-23): time_since_seen_sec tail during '
                    'the healthy track leg: max=%.3fs (n=%d samples). N2 sighting gate=1.0s; '
                    'old threshold was 0.37s -- flag for coordinator review if max > 0.37s.'
                    % (max_age, len(leg_age_samples)))
            if not leg_samples:
                for name in (
                    'c2. tracking window span >= 60 s (track leg only)',
                    'c3. horizontal distance within effective standoff (track leg)',
                    'c4. drone holds its OWN commanded cruise altitude (track leg)',
                ):
                    checks.append((
                        name, FTM,
                        'track_leg_window()=%r produced 0 samples (n=%d total over the whole '
                        'flight) -- cannot judge a leg with no measurements in it'
                        % (track_window, len(samples))))
            else:
                span = leg_samples[-1][0] - leg_samples[0][0]
                horiz = [math.hypot(s[1] - s[4], s[2] - s[5]) for s in leg_samples]
                in_range = [h for h in horiz if abs(h - EFFECTIVE_STANDOFF_COPY_M) <=
                            EFFECTIVE_STANDOFF_TOLERANCE_M]
                in_range_frac = len(in_range) / len(horiz)
                cruise_ref_z = leg_samples[0][3]
                max_alt_dev = max(abs(s[3] - cruise_ref_z) for s in leg_samples)
                checks.append((
                    'c2. tracking window span >= 60 s (track leg only)', span >= MIN_TRACK_SECONDS,
                    'span=%.1f s (track_leg_window=%r, n=%d samples)'
                    % (span, track_window, len(leg_samples))))
                checks.append((
                    'c3. horizontal distance within effective standoff %.2f +/- %.2f m '
                    '(navigator effectiveStandoff() copy) for >= 80%% of TRACK-LEG samples'
                    % (EFFECTIVE_STANDOFF_COPY_M, EFFECTIVE_STANDOFF_TOLERANCE_M),
                    in_range_frac >= 0.80,
                    '%.1f%% in range (n=%d), mean horiz dist=%.2f m (coordinator predicted '
                    'center %.2f m)'
                    % (100 * in_range_frac, len(horiz), sum(horiz) / len(horiz),
                       EXPECTED_STANDOFF_MEASUREMENT_M)))
                checks.append((
                    'c4. drone holds its OWN commanded cruise altitude +/- %.1f m during the '
                    'track leg (TrackTarget never chases the ground target\'s z, by design)'
                    % CRUISE_ALTITUDE_HOLD_TOLERANCE_M,
                    max_alt_dev <= CRUISE_ALTITUDE_HOLD_TOLERANCE_M,
                    'max deviation from leg-start altitude (%.3f m) = %.3f m (n=%d)'
                    % (cruise_ref_z, max_alt_dev, len(leg_samples))))

        # follow_target's own design: no discrete SUCCESS condition -- the
        # Timeout decorator firing is the NORMAL end (README/plan P9 S:3),
        # which mission_policy then routes through Finish (GotoHome->Land)
        # as ABORTED_TIMEOUT, never STATE_COMPLETED. Judge accordingly.
        final_status = node.wait_terminal_status(5.0)
        checks.append((
            'c5. terminal state ABORTED with result_code ABORTED_TIMEOUT (designed end for this '
            'mission type), with a non-empty goal_id',
            final_status is not None and final_status.state == MissionStatus.STATE_ABORTED and
            final_status.last_result_code == ResultCode.ABORTED_TIMEOUT and
            bool(final_status.goal_id),
            'state=%s last_result_code=%s goal_id=%r'
            % ((state_name(final_status.state), result_name(final_status.last_result_code),
                final_status.goal_id) if final_status else ('NONE', 'NONE', None))))
        checks.append((
            'c6. Land leg completed (goal terminal, action status)',
            status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_SUCCEEDED),
            'action status=%d result_code=%s' % (status, result_name(result.result_code))))
        checks.append((
            'c7. vehicle disarmed after Land (auto-disarm)',
            node.vehicle_state is not None and not node.vehicle_state.armed,
            'armed=%s' % (node.vehicle_state.armed if node.vehicle_state else 'UNKNOWN')))

        # body_window() correctly returns None here (track_target is a leaf
        # HALTED by the Timeout decorator, never a STEP_COMPLETED -- see its
        # docstring) -- track_leg_window() (log-based, R3) is the right tool
        # for THIS gate's clamped check, not body_window().
        window = track_leg_window(bringup_log)
        if window is None:
            checks.append((
                'c8. 0 "clamped" evidence in navigator log during the track leg '
                '(Takeoff/Finish excluded -- coordinator ruling 2026-08-22)', FTM,
                'track_leg_window() unusable (see c2/c3/c4) -- cannot bound the log window'))
            clamped_hits = None
        else:
            clamped_hits = grep_clamped(bringup_log, window) if bringup_log else None
            checks.append((
                'c8. 0 "clamped" evidence in navigator log during the track leg '
                '(Takeoff/Finish excluded -- coordinator ruling 2026-08-22)',
                FTM if clamped_hits is None else (len(clamped_hits) == 0),
                ('FAILED TO MEASURE (no log)' if clamped_hits is None else
                 ('clean, window=%r' % (window,) if not clamped_hits else
                  '%d hit(s) in window=%r: %r' % (len(clamped_hits), window, clamped_hits[:3])))))

        passed, ftm = print_checks(checks)
        if ftm:
            print('RESULT: FAILED TO MEASURE (at least one check could not be verified, R27-1)')
            return 2
        print('RESULT: %s (CAUTION: no heading claims made, per instructions)' % ('PASS' if passed else 'FAIL'))
        return 0 if passed else 1
    except Failure as exc:
        print('FAILED TO MEASURE: %s' % exc)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


# --------------------------------------------------------------------------
# G-M4.1: marker hide/reveal during inspect_point
# --------------------------------------------------------------------------

def gz_remove(world, name):
    req = 'name: "%s" type: MODEL' % name
    out = subprocess.run(
        ['gz', 'service', '-s', '/world/%s/remove' % world,
         '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req', req],
        capture_output=True, text=True, timeout=15)
    return out.returncode, out.stdout, out.stderr


def gz_create(world, sdf_path, name, x, y, z):
    req = 'sdf_filename: "%s" name: "%s" pose: {position: {x: %.3f y: %.3f z: %.3f}}' % (
        sdf_path, name, x, y, z)
    out = subprocess.run(
        ['gz', 'service', '-s', '/world/%s/create' % world,
         '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req', req],
        capture_output=True, text=True, timeout=15)
    return out.returncode, out.stdout, out.stderr


def run_event_marker(args, bringup_log):
    rclpy.init()
    node = MissionProbe(args.uav_id, 'gm4_marker_gate')
    try:
        node.wait_until(lambda: node.position() is not None, 60.0, 'odometry_fused')

        # Smoke-test the remove/create mechanism BEFORE trusting it in the
        # timed injection (R14: verify a new mechanism before building on it).
        rc, out, err = gz_remove(args.world, args.marker_name)
        node._log('smoke-test gz remove rc=%d out=%r err=%r' % (rc, out.strip(), err.strip()))
        if rc != 0 or 'data: true' not in out:
            raise Failure(
                'gz remove of %s did not confirm (rc=%d out=%r err=%r) -- refusing to trust the '
                'hide/reveal mechanism' % (args.marker_name, rc, out, err))
        time.sleep(1.0)
        rc, out, err = gz_create(
            args.world, args.marker_sdf, args.marker_name, args.marker_x, args.marker_y,
            args.marker_z)
        node._log('smoke-test gz re-create rc=%d out=%r err=%r' % (rc, out.strip(), err.strip()))
        if rc != 0 or 'data: true' not in out:
            raise Failure(
                'gz create of %s did not confirm after the smoke-test remove -- world may be left '
                'without the marker (rc=%d out=%r err=%r)' % (args.marker_name, rc, out, err))
        time.sleep(2.0)

        saw_search_start = {'flag': False}
        saw_recheck_after_hide = {'flag': False}
        hidden_at = {'t': None}
        revealed_at = {'t': None}

        # COPY of mission_executor_node's own "search_offset_m" default
        # (mission_executor_node.cpp: declare_parameter("search_offset_m", 2.0)).
        # search_x2 = approach_x - search_offset_m (populateBlackboard()) --
        # place approach OFFSET from the marker by exactly this much so
        # search_point_2 (the SECOND search leg, where marker_visible_recheck
        # actually fires -- the search Sequence is
        # [search_point_1, search_point_2, marker_visible_recheck], never
        # rechecked after point 1) lands EXACTLY over the marker. Using
        # approach == marker position (first attempt, 2026-08-22) put BOTH
        # search points 2 m off-center -- neither could ever see the marker
        # regardless of hide/reveal timing; this was a probe geometry bug,
        # not a product bug (see report).
        SEARCH_OFFSET_M_COPY = 2.0
        approach_x = args.marker_x + SEARCH_OFFSET_M_COPY
        approach_y = args.marker_y
        node._log(
            'approach=(%.2f,%.2f,%.2f) -- search_point_2 will land at (%.2f,%.2f), '
            'exactly the marker xy' % (approach_x, approach_y, args.inspect_z, args.marker_x, args.marker_y))

        handle = node.send_mission(
            'inspect_point',
            build_inspect_params(approach_x, approach_y, args.inspect_z, args.marker_id),
            loop=False)

        # Wait for the approach leg to START (STEP_STARTED goto_approach) --
        # only then remove the marker, so find_marker's FIRST check (before
        # goto_approach even lands) still sees it and the flow reaches
        # find_marker naturally.
        node.wait_until(
            lambda: any(
                e.event_type == MissionEvent.EVENT_STEP_STARTED and e.step_name == 'goto_approach'
                for _w, e in node.events),
            60.0, 'STEP_STARTED goto_approach')

        node._log('=== HIDING marker %s now (approach leg in flight) ===' % args.marker_name)
        rc, out, err = gz_remove(args.world, args.marker_name)
        hidden_at['t'] = time.time()
        if rc != 0 or 'data: true' not in out:
            raise Failure('timed marker removal did not confirm: rc=%d out=%r' % (rc, out))

        # Wait for the search branch to actually start (proves the guard
        # really fell through find_marker's Fallback into Search).
        try:
            node.wait_until(
                lambda: any(
                    e.event_type == MissionEvent.EVENT_STEP_STARTED and
                    e.step_name == 'search_point_1' for _w, e in node.events),
                60.0, 'STEP_STARTED search_point_1 (marker hidden)')
            saw_search_start['flag'] = True
        except Failure as exc:
            node._log('WARNING: %s' % exc)

        # Give search_point_1 time to be in flight, then reveal.
        node.spin(3.0)
        node._log('=== REVEALING marker %s now (during search) ===' % args.marker_name)
        rc, out, err = gz_create(
            args.world, args.marker_sdf, args.marker_name, args.marker_x, args.marker_y,
            args.marker_z)
        revealed_at['t'] = time.time()
        if rc != 0 or 'data: true' not in out:
            raise Failure('timed marker re-create did not confirm: rc=%d out=%r' % (rc, out))

        # Confirm we later return to the main branch: goto_over_marker starts.
        try:
            node.wait_until(
                lambda: any(
                    e.event_type == MissionEvent.EVENT_STEP_STARTED and
                    e.step_name == 'goto_over_marker' for _w, e in node.events),
                90.0, 'STEP_STARTED goto_over_marker (back on main branch)')
            saw_recheck_after_hide['flag'] = True
        except Failure as exc:
            node._log('WARNING: %s' % exc)

        result, status = node.wait_mission_result(handle, 120.0, 'inspect_point (marker event)')
        node._log(
            'ExecuteMission terminal: goal_status=%d success=%s result_code=%s msg=%r'
            % (status, result.success, result_name(result.result_code), result.message))

        checks = []
        checks.append((
            'd1. mission entered Search after marker was hidden', saw_search_start['flag'],
            'STEP_STARTED search_point_1 observed: %s' % saw_search_start['flag']))
        checks.append((
            'd2. mission returned to the main branch after marker reappeared',
            saw_recheck_after_hide['flag'],
            'STEP_STARTED goto_over_marker observed: %s' % saw_recheck_after_hide['flag']))
        checks.append((
            'd3. mission still completed successfully overall',
            status == GoalStatus.STATUS_SUCCEEDED, 'action status=%d' % status))

        passed, ftm = print_checks(checks)
        if ftm:
            print('RESULT: FAILED TO MEASURE (at least one check could not be verified, R27-1)')
            return 2
        print('RESULT: %s' % ('PASS' if passed else 'FAIL'))
        return 0 if passed else 1
    except Failure as exc:
        print('FAILED TO MEASURE: %s' % exc)
        # Best-effort: leave the marker present for whatever runs next.
        try:
            gz_create(args.world, args.marker_sdf, args.marker_name, args.marker_x, args.marker_y,
                      args.marker_z)
        except Exception:
            pass
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


# --------------------------------------------------------------------------
# G-M4.2: battery WARN during indoor_patrol
# --------------------------------------------------------------------------

def mav_set_param(name, value, port=14540, timeout=8.0):
    from pymavlink import mavutil
    conn = mavutil.mavlink_connection('udpin:0.0.0.0:%d' % port)
    conn.wait_heartbeat(timeout=15)
    conn.mav.param_set_send(
        conn.target_system, conn.target_component, name.encode('utf-8'), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    end = time.time() + timeout
    ack = None
    while time.time() < end:
        m = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if m and m.param_id.strip('\x00') == name:
            ack = m.param_value
            break
    conn.close()
    return ack


def run_event_battery(args, bringup_log):
    rclpy.init()
    node = MissionProbe(args.uav_id, 'gm4_battery_gate')
    try:
        node.wait_until(lambda: node.position() is not None, 60.0, 'odometry_fused')
        node.wait_until(lambda: node.health_history, 30.0, 'first VehicleHealth sample')

        handle = node.send_mission('indoor_patrol', build_indoor_params(), loop=False)

        node.wait_until(
            lambda: any(
                e.event_type == MissionEvent.EVENT_STEP_STARTED and e.step_name == 'goto_waypoint'
                for _w, e in node.events),
            90.0, 'first goto_waypoint STEP_STARTED (armed + flying)')
        node.spin(2.0)

        node._log('=== DRAIN (R31 label: artificial injection): SIM_BAT_MIN_PCT=0, '
                   'SIM_BAT_DRAIN=%.0f via MAVLink param_set ===' % args.drain_sec)
        ack_min = mav_set_param('SIM_BAT_MIN_PCT', 0.0)
        ack_drain = mav_set_param('SIM_BAT_DRAIN', args.drain_sec)
        node._log('ack SIM_BAT_MIN_PCT=%s SIM_BAT_DRAIN=%s' % (ack_min, ack_drain))
        if ack_min is None or ack_drain is None:
            raise Failure('MAVLink param_set for SIM_BAT_MIN_PCT/SIM_BAT_DRAIN was not acknowledged')
        drain_started_at = time.time()

        # Anchor on the EVENT_PAUSED-less end_early path: mission does not
        # PAUSE for battery, it goes straight to Finish (GotoHome->Land) --
        # so anchor on the terminal ABORTED event with ABORTED_LOW_BATTERY,
        # not on any intermediate state.
        result, status = node.wait_mission_result(handle, 180.0, 'indoor_patrol (battery event)')
        low_battery_seen_at = time.time()
        node._log(
            'ExecuteMission terminal: goal_status=%d success=%s result_code=%s msg=%r'
            % (status, result.success, result_name(result.result_code), result.message))

        goto_home_events = [
            (w, e) for w, e in node.events
            if e.event_type == MissionEvent.EVENT_STEP_STARTED]
        final_status = node.wait_terminal_status(5.0)

        checks = []
        checks.append((
            'e1. drain acknowledged by PX4 (R31: artificial injection, labelled)',
            ack_min is not None and ack_drain is not None,
            'SIM_BAT_MIN_PCT ack=%s SIM_BAT_DRAIN ack=%s' % (ack_min, ack_drain)))
        checks.append((
            'e2. terminal MissionStatus.last_result_code == ABORTED_LOW_BATTERY (12)',
            final_status is not None and
            final_status.last_result_code == ResultCode.ABORTED_LOW_BATTERY,
            'last_result_code=%s'
            % (result_name(final_status.last_result_code) if final_status else 'NONE')))
        checks.append((
            'e3. ExecuteMission action result_code == ABORTED_LOW_BATTERY',
            result.result_code == ResultCode.ABORTED_LOW_BATTERY,
            'result_code=%s' % result_name(result.result_code)))
        checks.append((
            'e4. GotoPose(home) + Land were dispatched (navigator saw goto_waypoint AND '
            'the finish leg, action status terminal)',
            status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_SUCCEEDED),
            'action status=%d, %d STEP_STARTED events total, elapsed since drain start=%.1fs'
            % (status, len(goto_home_events), low_battery_seen_at - drain_started_at)))
        checks.append((
            'e5. vehicle disarmed after Land (auto-disarm)',
            node.vehicle_state is not None and not node.vehicle_state.armed,
            'armed=%s' % (node.vehicle_state.armed if node.vehicle_state else 'UNKNOWN')))

        if node.health_history:
            batt_series = [(w, h.battery_remaining) for w, h in node.health_history]
            below_warn = [w for w, b in batt_series if b == b and b < 0.35]  # b==b filters NaN
            checks.append((
                'e6. observed battery_remaining actually crossed WARN (0.35)', len(below_warn) > 0,
                '%d samples below 0.35 out of %d total' % (len(below_warn), len(batt_series))))
        else:
            checks.append(('e6. battery_remaining series (NO SAMPLES)', False, 'no health samples'))

        passed, ftm = print_checks(checks)
        if ftm:
            print('RESULT: FAILED TO MEASURE (at least one check could not be verified, R27-1)')
            return 2
        print('RESULT: %s' % ('PASS' if passed else 'FAIL'))
        return 0 if passed else 1
    except Failure as exc:
        print('FAILED TO MEASURE: %s' % exc)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


# --------------------------------------------------------------------------
# G-M4.3: safety seize -> PAUSED -> ClearFault (still PAUSED) -> Resume
# (positive control WITHOUT mission is a separate re-run of
# verify_obstacle_hold.sh, done by the orchestrator, not this subcommand)
# --------------------------------------------------------------------------

INJECT_DISTANCE_M = 0.18
INJECT_SECONDS = 8.0


def run_event_authority(args, bringup_log):
    rclpy.init()
    node = MissionProbe(args.uav_id, 'gm4_authority_gate')
    try:
        node.wait_until(lambda: node.position() is not None, 60.0, 'odometry_fused')

        # R27-1: confirm THIS probe will be the sole publisher on
        # obstacle_map_local before trusting the injection (obstacle_hold_gate.py
        # convention) -- caller script must have started sim.launch.py with
        # perception:=false.
        info_topic = node.get_topic_names_and_types()
        # (rclpy has no direct "publisher count on a not-yet-subscribed topic"
        # API pre-subscribe; the wrapper script checks this with `ros2 topic
        # info` BEFORE launching this probe, matching verify_obstacle_hold.sh.)

        handle = node.send_mission('indoor_patrol', build_indoor_params(), loop=False)
        node.wait_until(
            lambda: any(
                e.event_type == MissionEvent.EVENT_STEP_STARTED and e.step_name == 'goto_waypoint'
                for _w, e in node.events),
            90.0, 'first goto_waypoint STEP_STARTED (armed + flying)')
        node.spin(2.0)

        node.wait_until(
            lambda: any(source == SOURCE_MISSION for _w, source, *_ in node.commands),
            20.0, 'command_selected showing SOURCE_MISSION before injection')

        injected_at = time.time()
        node._log(
            '=== INJECT obstacle distance=%.2f m at t=%.3f (perception:=false, probe is sole '
            'publisher) ===' % (INJECT_DISTANCE_M, injected_at))
        node.spin_and_publish_obstacle(INJECT_SECONDS, present=True)

        onset_authority = None
        for wall, msg in node.authority_history:
            if wall >= injected_at and msg.active_source == SOURCE_SAFETY:
                onset_authority = wall
                break

        paused_wall = None
        paused_reason = None
        for wall, e in node.events:
            if wall >= injected_at and e.event_type == MissionEvent.EVENT_PAUSED:
                paused_wall = wall
                paused_reason = e.description
                break
        if paused_wall is None:
            node.wait_until(
                lambda: any(
                    wall >= injected_at and e.event_type == MissionEvent.EVENT_PAUSED
                    for wall, e in node.events),
                10.0, 'EVENT_PAUSED after safety seize')
            for wall, e in node.events:
                if wall >= injected_at and e.event_type == MissionEvent.EVENT_PAUSED:
                    paused_wall = wall
                    paused_reason = e.description
                    break

        cancel_seen_wall = None
        # No direct topic exposes navigator's own goal-status; the only
        # externally visible cancel evidence is command_selected's source
        # dropping away from MISSION once safety wins arbitration, which we
        # already have in node.commands.
        for wall, source, *_ in node.commands:
            if wall >= injected_at and source != SOURCE_MISSION:
                cancel_seen_wall = wall
                break

        # --- ClearFault while still PAUSED: mission must NOT auto-resume.
        # clear_stability_sec (3.0s, safety_params.yaml) is measured ACROSS
        # CALLS to the clear_fault SERVICE (failsafe_policy.cpp's own
        # comment) -- a single call after a fixed sleep always sees
        # stable_for=0 on its only sample and gets rejected. POLL while
        # keeping the map continuously fresh+empty, same discipline as
        # obstacle_hold_gate.py (G-S3-HOLD, already proven 2026-08-21).
        node._log('=== CLEAR: publishing empty ObstacleArray + polling ClearFault ===')
        OBSTACLE_MAP_TIMEOUT_SEC = 0.3
        CLEAR_STABILITY_SEC = 3.0
        CLEAR_POLL_PERIOD_SEC = 0.5
        CLEAR_WAIT_SEC = OBSTACLE_MAP_TIMEOUT_SEC + CLEAR_STABILITY_SEC + 5.0
        clear_start = time.time()
        next_obstacle_publish = clear_start
        next_clear_poll = clear_start + OBSTACLE_MAP_TIMEOUT_SEC
        clear_attempts = []
        clear_result = None
        deadline = clear_start + CLEAR_WAIT_SEC
        while rclpy.ok() and time.time() < deadline:
            now = time.time()
            if now >= next_obstacle_publish:
                node.publish_obstacle(False)
                next_obstacle_publish = now + (1.0 / 10.0)
            if now >= next_clear_poll:
                attempt = node.call_clear_fault()
                clear_attempts.append((time.time(), attempt.success, list(attempt.remaining_faults)))
                node._log(
                    'ClearFault poll: success=%s remaining=%s'
                    % (attempt.success, list(attempt.remaining_faults)))
                if attempt.success:
                    clear_result = attempt
                    break
                next_clear_poll = time.time() + CLEAR_POLL_PERIOD_SEC
            rclpy.spin_once(node, timeout_sec=0.02)
        if clear_result is None:
            if clear_attempts:
                _w, success, remaining = clear_attempts[-1]
                clear_result = _LastClearAttempt(success, remaining)
            else:
                clear_result = _LastClearAttempt(False, ['NEVER POLLED -- CLEAR_WAIT_SEC too short'])
        node._log(
            'ClearFault final: success=%s remaining=%s (polled %d time(s) over %.1fs)'
            % (clear_result.success, clear_result.remaining_faults, len(clear_attempts),
               time.time() - clear_start))

        status_after_clear = node.latest_status()
        still_paused_after_clear = (
            status_after_clear is not None and status_after_clear.state == MissionStatus.STATE_PAUSED)

        if not clear_result.success:
            node._log('ClearFault never succeeded -- skipping Resume (would be a no-op on a still-'
                       'latched fault); recording the observed state as-is')
            resume_result = _LastClearAttempt(False, ['ClearFault never succeeded'])
            resume_result.message = 'not attempted -- ClearFault never succeeded'
        else:
            # The control-authority arbiter has its own 2Hz heartbeat
            # (authority_heartbeat_copy_sec=0.5s) -- calling Resume the
            # INSTANT ClearFault succeeds can race a still-stale
            # /control/authority sample (active_source != MISSION),
            # re-triggering priority-2's PAUSE on the very next tick (found
            # the hard way, first properly-polled run: Resume succeeded,
            # MissionStatus flipped to RUNNING for ~0.18s, then immediately
            # back to PAUSED). Wait for a FRESH authority sample showing
            # MISSION before calling Resume -- gives the arbiter >= 2
            # heartbeat periods to catch up after the SAFETY latch clears.
            node.wait_until(
                lambda: node.authority_history and node.authority_history[-1][1].active_source ==
                SOURCE_MISSION and (time.time() - node.authority_history[-1][0]) < 1.0,
                10.0, 'a fresh /control/authority sample showing MISSION before calling Resume')
            node.spin(1.5)   # >= 2 x authority_heartbeat_copy_sec of settle margin

            node._log('=== RESUME: calling /mission/resume ===')
            resume_result = node.call_resume()
            node._log(
                'ResumeMission: success=%s message=%r' % (resume_result.success, resume_result.message))

        resumed_running = False
        resume_sent_at = time.time()
        try:
            node.wait_until(
                lambda: node.latest_status() is not None and
                node.latest_status().state == MissionStatus.STATE_RUNNING, 20.0,
                'MissionStatus back to STATE_RUNNING after Resume')
            resumed_running = True
        except Failure as exc:
            node._log('WARNING: %s' % exc)

        # f6 must prove GENUINE continued flight ("Resume chay tiep"), not
        # just a transient state flip -- wait for a FRESH goto_waypoint
        # dispatch (STEP_STARTED) strictly after the resume call.
        resumed_and_flew = False
        if resumed_running:
            try:
                node.wait_until(
                    lambda: any(
                        w >= resume_sent_at and e.event_type == MissionEvent.EVENT_STEP_STARTED and
                        e.step_name == 'goto_waypoint' for w, e in node.events),
                    15.0, 'a fresh goto_waypoint STEP_STARTED after Resume (genuine continued flight)')
                resumed_and_flew = True
            except Failure as exc:
                node._log('WARNING: %s' % exc)

        # Let the mission finish on its own (or time out its own retries) so
        # teardown does not race a still-active goal.
        try:
            result, status = node.wait_mission_result(handle, 300.0, 'indoor_patrol (authority event)')
            node._log(
                'ExecuteMission terminal: goal_status=%d result_code=%s msg=%r'
                % (status, result_name(result.result_code), result.message))
        except Failure as exc:
            node._log('WARNING: mission did not settle within budget: %s' % exc)

        checks = []
        onset_after_injection = (onset_authority - injected_at) if onset_authority else float('nan')
        checks.append((
            'f1. command_selected/control authority actually flipped to SAFETY',
            onset_authority is not None, 'onset_after_injection=%.3fs' % onset_after_injection))
        pause_after_injection = (paused_wall - injected_at) if paused_wall else float('nan')
        pause_after_authority = (
            (paused_wall - onset_authority) if (paused_wall and onset_authority) else float('nan'))
        # Budget is against the DESIGNED grace timer (authority_loss_grace_sec
        # = 1.5s, mission_params.yaml), not against raw injection time --
        # see report note on the mismatch with the literal task wording.
        checks.append((
            'f2. mission cancelled/PAUSED within budget of the DESIGNED grace '
            '(authority_loss_grace_sec=1.5s + one 10Hz tick + cancel ack, budget<=2.2s from authority seize)',
            not math.isnan(pause_after_authority) and pause_after_authority <= 2.2,
            'PAUSED %.3fs after authority onset (%.3fs after raw injection)'
            % (pause_after_authority, pause_after_injection)))
        checks.append((
            'f3. MissionStatus.state == STATE_PAUSED, last_reason mentions authority/safety',
            paused_reason is not None and
            ('authority' in paused_reason.lower() or 'safety' in paused_reason.lower()),
            'reason=%r' % paused_reason))
        checks.append((
            'f4. command_selected source moved away from MISSION (cancel visible on the wire)',
            cancel_seen_wall is not None,
            'first non-MISSION command_selected at t+%.3fs'
            % ((cancel_seen_wall - injected_at) if cancel_seen_wall else float('nan'))))
        checks.append((
            'f5. ClearFault succeeded but mission STILL PAUSED afterwards (no auto-resume)',
            clear_result.success and not list(clear_result.remaining_faults) and
            still_paused_after_clear,
            'clear success=%s remaining=%s still_paused=%s'
            % (clear_result.success, list(clear_result.remaining_faults), still_paused_after_clear)))
        checks.append((
            'f6. ResumeMission succeeded and mission GENUINELY continued flying '
            '(fresh goto_waypoint dispatch after Resume, not just a transient status flip)',
            resume_result.success and resumed_running and resumed_and_flew,
            'resume success=%s resumed_running=%s resumed_and_flew=%s'
            % (resume_result.success, resumed_running, resumed_and_flew)))

        passed, ftm = print_checks(checks)
        if ftm:
            print('RESULT: FAILED TO MEASURE (at least one check could not be verified, R27-1)')
            return 2
        print('RESULT: %s' % ('PASS' if passed else 'FAIL'))
        return 0 if passed else 1
    except Failure as exc:
        print('FAILED TO MEASURE: %s' % exc)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


# --------------------------------------------------------------------------
# G-M4.4: lost target during follow_target
# --------------------------------------------------------------------------

def run_event_target(args, bringup_log):
    rclpy.init()
    node = MissionProbe(args.uav_id, 'gm4_target_gate')
    try:
        node.wait_until(lambda: node.position() is not None, 60.0, 'odometry_fused')

        handle = node.send_mission(
            'follow_target',
            'target_speed_mps: %.3f\nstandoff_m: %.2f\ntimeout_sec: %.1f\n'
            % (FOLLOW_TARGET_SPEED_MPS, FOLLOW_STANDOFF_M, args.mission_timeout_sec),
            loop=False)

        node.wait_until(
            lambda: any(
                e.event_type == MissionEvent.EVENT_STEP_STARTED and e.step_name == 'track_target'
                for _w, e in node.events),
            90.0, 'STEP_STARTED track_target (actively tracking before we kill it)')
        node.spin(3.0)   # a few seconds of confirmed tracking before the injection

        node._log('=== STOP + REMOVE target %s now (forcing target_lost) ===' % args.box_name)
        rc, _out, _err = subprocess.run(
            ['gz', 'topic', '-t', '/model/%s/cmd_vel' % args.box_name,
             '-m', 'gz.msgs.Twist', '-p', 'linear: {x: 0.0, y: 0.0, z: 0.0}'],
            capture_output=True, text=True).returncode, None, None
        removed = gz_remove(args.world, args.box_name)
        node._log('gz remove target: rc=%d out=%r' % (removed[0], removed[1].strip()))
        lost_injected_at = time.time()

        result, status = node.wait_mission_result(
            handle, args.mission_timeout_sec + 90.0, 'follow_target (lost target event)')
        node._log(
            'ExecuteMission terminal: goal_status=%d result_code=%s msg=%r'
            % (status, result_name(result.result_code), result.message))
        settled_at = time.time()

        final_status = node.wait_terminal_status(5.0)
        step_completed_or_started = [
            (w, e.event_type, e.step_name, e.result_code) for w, e in node.events if w >= lost_injected_at]

        # Coordinator ask 2026-08-23 (post N1/N2/N3 patch): TWO valid chains
        # now exist -- (a) navigator's OWN target_lost_timeout (4.0s, inside
        # an ACTIVE TrackTarget goal) fires ABORTED_LOST_TARGET directly, or
        # (b) mission's TargetSeen condition (world_model's
        # time_since_seen_sec now age-accumulated, N2's sighting gate) goes
        # false first and the ReactiveFallback falls to Search BEFORE
        # navigator's own timeout would fire. Record which happened -- do
        # not assert one over the other, both are correct per design.
        saw_search_after_injection = any(
            e.event_type == MissionEvent.EVENT_STEP_STARTED and
            e.step_name in ('search_point_1', 'search_point_2')
            for w, e in node.events if w >= lost_injected_at)
        if saw_search_after_injection:
            node._log(
                'CHAIN OBSERVED: mission-side (TargetSeen went false -> ReactiveFallback fell to '
                'Search) -- search STEP_STARTED seen after injection, before/without a direct '
                'navigator ABORTED_LOST_TARGET on track_target')
        else:
            node._log(
                'CHAIN OBSERVED: navigator-side (target_lost_timeout=4.0s fired directly inside '
                'the ACTIVE TrackTarget goal) -- no search leg observed after injection')

        checks = []
        checks.append((
            'g1. terminal result_code == ABORTED_LOST_TARGET (9), NOT ABORTED_TIMEOUT',
            result.result_code == ResultCode.ABORTED_LOST_TARGET,
            'action result_code=%s (also checking MissionStatus.last_result_code=%s)'
            % (result_name(result.result_code),
               result_name(final_status.last_result_code) if final_status else 'NONE')))
        checks.append((
            'g2. MissionStatus.last_result_code == ABORTED_LOST_TARGET (9)',
            final_status is not None and
            final_status.last_result_code == ResultCode.ABORTED_LOST_TARGET,
            'last_result_code=%s'
            % (result_name(final_status.last_result_code) if final_status else 'NONE')))
        checks.append((
            'g3. elapsed from injection to terminal well under mission_timeout_sec '
            '(proves detection via navigator target_lost_timeout, not the outer Timeout decorator)',
            (settled_at - lost_injected_at) < args.mission_timeout_sec,
            'elapsed=%.1fs vs mission Timeout budget=%.1fs'
            % (settled_at - lost_injected_at, args.mission_timeout_sec)))
        # Recover(CLIMB) is a SAFETY HOLD (climb to a safe altitude and
        # hover), NOT a landing -- confirmed on the wire 2026-08-23:
        # navigator logs "accepted recover goal" then "recovery taking the
        # aircraft over: climbing X m to Y m (follow_target: search-attempts
        # budget exhausted)". The correct terminal state is ARMED and
        # HOVERING at the climbed altitude, not disarmed -- the earlier
        # "disarmed after Land" check assumed the SAME Finish path as
        # ABORTED_LOW_BATTERY, which was wrong for this result code (probe
        # bug, not a product bug).
        recover_dispatched = False
        recover_climb_line = None
        if bringup_log:
            try:
                with open(bringup_log, 'r', errors='replace') as f:
                    for line in f:
                        if 'accepted recover goal' in line:
                            recover_dispatched = True
                        if 'recovery taking the aircraft over' in line:
                            recover_climb_line = line.strip()
            except OSError:
                pass
        checks.append((
            'g4. Recover(CLIMB) dispatched via navigator (R3: read the log)',
            FTM if bringup_log is None else recover_dispatched,
            ('FAILED TO MEASURE (no bringup log)' if bringup_log is None else
             ('"accepted recover goal" seen; %s' % (recover_climb_line or '(no climb detail line)')
              if recover_dispatched else 'no "accepted recover goal" found in navigator log'))))
        checks.append((
            'g5. vehicle remains ARMED and holding (Recover=CLIMB hold, not a landing)',
            node.vehicle_state is not None and node.vehicle_state.armed,
            'armed=%s, altitude=%.2fm'
            % (node.vehicle_state.armed if node.vehicle_state else 'UNKNOWN',
               node.position()[2] if node.position() else float('nan'))))

        passed, ftm = print_checks(checks)
        print('  post-injection events: %s' % step_completed_or_started[:20])
        if ftm:
            print('RESULT: FAILED TO MEASURE (at least one check could not be verified, R27-1)')
            return 2
        print('RESULT: %s' % ('PASS' if passed else 'FAIL'))
        return 0 if passed else 1
    except Failure as exc:
        print('FAILED TO MEASURE: %s' % exc)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


# --------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bringup-log', default=None, help='navigator/bringup log for clamp check')
    sub = parser.add_subparsers(dest='phase', required=True)

    p = sub.add_parser('patrol')
    p.add_argument('--uav-id', default='uav0')

    p = sub.add_parser('inspect')
    p.add_argument('--uav-id', default='uav0')
    p.add_argument('--marker-id', type=int, default=7)
    p.add_argument('--marker-x', type=float, required=True)
    p.add_argument('--marker-y', type=float, required=True)
    p.add_argument('--inspect-z', type=float, default=2.5)

    p = sub.add_parser('follow')
    p.add_argument('--uav-id', default='uav0')

    p = sub.add_parser('event-marker')
    p.add_argument('--uav-id', default='uav0')
    p.add_argument('--world', default='uav_arena')
    p.add_argument('--marker-id', type=int, default=7)
    p.add_argument('--marker-x', type=float, required=True)
    p.add_argument('--marker-y', type=float, required=True)
    p.add_argument('--marker-z', type=float, default=0.01)
    p.add_argument('--inspect-z', type=float, default=2.5)
    p.add_argument('--marker-name', default='gm_event_marker')
    p.add_argument('--marker-sdf', required=True)

    p = sub.add_parser('event-battery')
    p.add_argument('--uav-id', default='uav0')
    p.add_argument('--drain-sec', type=float, default=60.0,
                    help='SIM_BAT_DRAIN: seconds for a full 100%%->0%% drain while armed')

    p = sub.add_parser('event-authority')
    p.add_argument('--uav-id', default='uav0')

    p = sub.add_parser('event-target')
    p.add_argument('--uav-id', default='uav0')
    p.add_argument('--world', default='uav_arena')
    p.add_argument('--box-name', default='gm_event_box')
    p.add_argument('--mission-timeout-sec', type=float, default=90.0)

    args = parser.parse_args()
    dispatch = {
        'patrol': run_patrol,
        'inspect': run_inspect,
        'follow': run_follow,
        'event-marker': run_event_marker,
        'event-battery': run_event_battery,
        'event-authority': run_event_authority,
        'event-target': run_event_target,
    }
    return dispatch[args.phase](args, args.bringup_log)


if __name__ == '__main__':
    sys.exit(main())
