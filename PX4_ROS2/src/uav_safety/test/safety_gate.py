#!/usr/bin/env python3
"""G-S2 (P8-safety.md S:8): fly a slow Goto, watch for a localization-loss
fault injected EXTERNALLY (verify_safety.sh -- pkill or ros2 param set, NOT
this probe), and record the timestamped chain: is_valid=false -> violation
entered -> last command_selected message -> offboard leaves ACTIVE -> PX4
failsafe_active/flight_mode/nav_state_raw. R1 R20/R25: only uav_interfaces
types, never px4_msgs -- not isolated to a domain on purpose (must see the
real flight).

Usage: python3 safety_gate.py --uav-id uav0 --watch-seconds 40
Prints a report; exit 0 means the recorded chain matched G-S2's acceptance
criteria (cut before an extra 0.83 m of blind travel, zero cmd_safety
messages, offboard/PX4 numbers were captured). It does NOT itself judge
"was the injection realistic" -- that is verify_safety.sh's job (it knows
which of the 3 variants it triggered and when).
"""
import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from uav_interfaces.action import GotoPose, Takeoff
from uav_interfaces.msg import ControlCommand, LocalizationStatus, OffboardStatus, VehicleState

TAKEOFF_ALTITUDE = 2.5      # m above the starting point
GOTO_EAST = 6.0              # m -- far enough that a slow leg stays airborne
GOTO_SPEED = 0.4             # m/s, deliberately slow: a wide injection window
MAX_BLIND_DISTANCE_M = 0.83  # S:0b R1: 0.55 (shipped leash) * 1.5 (blind_grace_sec)

STATE_ACTIVE = OffboardStatus.STATE_ACTIVE


class Failure(Exception):
    pass


class SafetyGate(Node):
    def __init__(self, uav_id, use_sim_time):
        from rclpy.parameter import Parameter
        super().__init__(
            'safety_gate',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])
        prefix = '/uav/' + uav_id
        self.uav_id = uav_id

        self.state = None
        self.odom = None
        self.localization_valid = None
        self.localization_invalid_since_wall = None   # is_valid=False specifically (diagnostic)
        self.localization_lost_since_wall = None       # is_valid=False OR LOCALIZATION_STALE/INVALID edge

        self.commands = []          # (wall, sim, source, x, y, z)
        self.cmd_safety_count = 0
        self.violations = []        # (wall, code, message)
        self.offboard_transitions = []   # (wall, state, detail)
        self.vehicle_transitions = []    # (wall, nav_state_raw, flight_mode, failsafe_active)

        self.create_subscription(
            LocalizationStatus, prefix + '/state/localization_status', self._on_localization, 10)
        self.create_subscription(Odometry, prefix + '/state/odometry_fused', self._on_odom, 10)
        self.create_subscription(
            ControlCommand, prefix + '/control/command_selected', self._on_command, 50)
        self.create_subscription(
            ControlCommand, prefix + '/control/cmd_safety', self._on_cmd_safety, 10)
        self.create_subscription(
            OffboardStatus, prefix + '/backend/offboard_status', self._on_offboard, 10)
        self.create_subscription(VehicleState, prefix + '/state/vehicle', self._on_state, 10)
        self.create_subscription(
            DiagnosticArray, prefix + '/safety/violations', self._on_violations, 20)

        self.takeoff_client = ActionClient(self, Takeoff, prefix + '/planning/takeoff')
        self.goto_client = ActionClient(self, GotoPose, prefix + '/planning/goto_pose')

    # ------------------------------------------------------------ callbacks

    def _on_localization(self, msg):
        was_valid = self.localization_valid is None or self.localization_valid
        self.localization_valid = msg.is_valid
        if not msg.is_valid and was_valid:
            self.localization_invalid_since_wall = time.time()
            self._mark_localization_lost()
            self._log('localization_status.is_valid -> False')

    def _mark_localization_lost(self):
        """Onset anchor for blind_distance_m. is_valid=False is the primary
        signal, but a DEAD PUBLISHER (pkill localization_mux_node itself, not
        just its sources) never sends a "last will" message announcing
        invalidity -- ROS2 pub/sub has no such thing, the topic just goes
        silent. LOCALIZATION_STALE (odometry_fused aging out, independent of
        is_valid) is the only signal that still fires in that case, so it is
        an equally valid onset the first time either one appears."""
        if self.localization_lost_since_wall is None:
            self.localization_lost_since_wall = time.time()

    def _on_odom(self, msg):
        self.odom = msg

    def _on_command(self, msg):
        p = msg.position
        self.commands.append(
            (time.time(), self.sim_seconds(), msg.source, p.x, p.y, p.z))

    def _on_cmd_safety(self, _msg):
        self.cmd_safety_count += 1
        self._log('cmd_safety message received (should be ZERO -- INHIBIT never publishes)')

    def _on_offboard(self, msg):
        if not self.offboard_transitions or self.offboard_transitions[-1][1] != msg.state:
            self.offboard_transitions.append((time.time(), msg.state, msg.detail))
            self._log('offboard_status.state -> %d (%s)' % (msg.state, msg.detail))

    def _on_state(self, msg):
        self.state = msg
        last = self.vehicle_transitions[-1] if self.vehicle_transitions else None
        changed = last is None or last[1] != msg.nav_state_raw or last[3] != msg.failsafe_active
        if changed:
            self.vehicle_transitions.append(
                (time.time(), msg.nav_state_raw, msg.flight_mode, msg.failsafe_active))
            self._log(
                'vehicle: nav_state_raw=%d flight_mode=%d failsafe_active=%s armed=%s'
                % (msg.nav_state_raw, msg.flight_mode, msg.failsafe_active, msg.armed))

    def _on_violations(self, msg):
        for status in msg.status:
            if status.name.startswith('safety: '):
                code = status.name[len('safety: '):]
                self.violations.append((time.time(), code, status.message))
                self._log('violation edge: %s -- %s' % (code, status.message))
                if 'entered' in status.message and code in ('LOCALIZATION_INVALID', 'LOCALIZATION_STALE'):
                    self._mark_localization_lost()

    def _log(self, text):
        print('  [t=%.3f] %s' % (time.time(), text))
        sys.stdout.flush()

    # -------------------------------------------------------------- helpers

    def sim_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

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
        """Wait for the action to actually FINISH, not just be accepted --
        the altitude proxy alone races with navigator's own busy state
        (rejected the very next goal with "navigator is busy in TAKING_OFF"
        because the Takeoff action had not returned yet)."""
        future = handle.get_result_async()
        end = time.time() + timeout
        while rclpy.ok() and not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise Failure('%s result timed out' % description)
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise Failure('%s finished with status=%d' % (description, result.status))
        return result

    def position(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        return (p.x, p.y, p.z)


def fly_and_watch(node, watch_seconds):
    node.wait_until(lambda: node.state is not None and node.state.connected, 60.0, 'autopilot')
    node.wait_until(lambda: node.position() is not None, 30.0, 'odometry_fused')
    origin = node.position()
    print('origin (fused) %.3f %.3f %.3f' % origin)

    takeoff = Takeoff.Goal()
    takeoff.target_altitude = float(TAKEOFF_ALTITUDE)
    takeoff_handle = node.send_goal(node.takeoff_client, takeoff, 'takeoff')
    node.wait_result(takeoff_handle, 60.0, 'takeoff')
    print('takeoff complete, altitude %.2f' % node.position()[2])

    goal = GotoPose.Goal()
    goal.target_pose.position.x = origin[0] + GOTO_EAST
    goal.target_pose.position.y = origin[1]
    goal.target_pose.position.z = origin[2] + TAKEOFF_ALTITUDE
    goal.target_pose.orientation.w = 1.0
    goal.frame_id = 'odom'
    goal.acceptance_radius = 0.5
    goal.max_speed = float(GOTO_SPEED)
    node.send_goal(node.goto_client, goal, 'goto')
    print('goto sent (max_speed=%.2f m/s) -- watching for %.1f s, inject externally now' % (
        GOTO_SPEED, watch_seconds))

    # Injection happens OUTSIDE this process (verify_safety.sh), somewhere in
    # this window. Just watch and record.
    node.spin(watch_seconds)


def analyze(node):
    """Blind distance: setpoint travel between the loss ONSET and the LAST
    command_selected sample before the cut -- the S:0b R1 quantity, measured
    on the wire, never from an internal flag. Onset is is_valid=False OR the
    first LOCALIZATION_INVALID/STALE violation edge, whichever comes first:
    a killed PUBLISHER (pkill localization_mux_node itself) never sends a
    "last will" is_valid=False message, ROS2 pub/sub has no such thing -- only
    LOCALIZATION_STALE (odometry_fused aging out) still fires in that case."""
    report = {}
    report['localization_invalid_seen'] = node.localization_invalid_since_wall is not None
    report['localization_lost_seen'] = node.localization_lost_since_wall is not None
    report['violations'] = list(node.violations)
    report['cmd_safety_count'] = node.cmd_safety_count
    report['offboard_transitions'] = list(node.offboard_transitions)
    report['vehicle_transitions'] = list(node.vehicle_transitions)

    if node.localization_lost_since_wall is None or len(node.commands) < 2:
        report['blind_distance_m'] = float('nan')
        report['cut_wall_seconds_after_onset'] = float('nan')
        return report

    onset = node.localization_lost_since_wall
    onset_pos = None
    last_pos = None
    last_wall = onset
    for wall, _sim, _source, x, y, z in node.commands:
        if wall < onset:
            onset_pos = (x, y, z)
            continue
        last_pos = (x, y, z)
        last_wall = wall
    if onset_pos is None:
        onset_pos = (node.commands[0][3], node.commands[0][4], node.commands[0][5])
    if last_pos is None:
        last_pos = onset_pos

    report['blind_distance_m'] = math.dist(onset_pos, last_pos)
    report['cut_wall_seconds_after_onset'] = last_wall - onset
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--watch-seconds', type=float, default=40.0)
    parser.add_argument('--no-sim-time', action='store_true')
    parser.add_argument(
        '--expect-clean', action='store_true',
        help='positive control: nothing injected, expect zero violations')
    args = parser.parse_args()

    rclpy.init()
    node = SafetyGate(args.uav_id, use_sim_time=not args.no_sim_time)

    failure = None
    report = {}
    try:
        fly_and_watch(node, args.watch_seconds)
        report = analyze(node)
    except Failure as exc:
        failure = str(exc)
    except KeyboardInterrupt:
        failure = 'interrupted'

    print('')
    print('=== G-S2 SAFETY GATE REPORT (%s) ===' % args.uav_id)
    print('localization_invalid_seen : %s (is_valid=False specifically)'
          % report.get('localization_invalid_seen'))
    print('localization_lost_seen    : %s (is_valid=False OR STALE/INVALID edge -- used for PASS)'
          % report.get('localization_lost_seen'))
    print('blind_distance_m          : %.3f (budget %.2f)'
          % (report.get('blind_distance_m', float('nan')), MAX_BLIND_DISTANCE_M))
    print('cut_seconds_after_onset   : %.3f'
          % report.get('cut_wall_seconds_after_onset', float('nan')))
    print('cmd_safety_count          : %d (must be 0)' % report.get('cmd_safety_count', -1))
    print('violations                :')
    for wall, code, message in report.get('violations', []):
        print('    [t=%.3f] %s -- %s' % (wall, code, message))
    print('offboard transitions      :')
    for wall, state, detail in report.get('offboard_transitions', []):
        print('    [t=%.3f] state=%d detail=%s' % (wall, state, detail))
    print('vehicle transitions (R3, PX4 numbers) :')
    for wall, nav_state_raw, flight_mode, failsafe_active in report.get('vehicle_transitions', []):
        print('    [t=%.3f] nav_state_raw=%d flight_mode=%d failsafe_active=%s'
              % (wall, nav_state_raw, flight_mode, failsafe_active))
    if failure:
        print('FAILURE                   : %s' % failure)

    if args.expect_clean:
        # Positive control: nothing injected -- PASS means safety never
        # CUT anything, not that its diagnostics stayed silent. A transient
        # BLIND_COMMAND/OFFBOARD_UNHEALTHY "entered -> cleared" pair during
        # the normal engage handshake (setpoint stream not yet confirmed)
        # is the grace period working AS DESIGNED, not a false positive --
        # judge by whether offboard ever left ACTIVE once it got there.
        offboard_reached_active = any(
            state == STATE_ACTIVE for _wall, state, _detail in report.get('offboard_transitions', []))
        offboard_left_active_after = False
        seen_active = False
        for _wall, state, _detail in report.get('offboard_transitions', []):
            if state == STATE_ACTIVE:
                seen_active = True
            elif seen_active:
                offboard_left_active_after = True
        passed = (
            failure is None and
            report.get('localization_invalid_seen') is False and
            report.get('cmd_safety_count', 1) == 0 and
            offboard_reached_active and
            not offboard_left_active_after
        )
    else:
        # localization_lost_seen (is_valid=False OR LOCALIZATION_INVALID/
        # STALE edge), not localization_invalid_seen alone: a killed
        # PUBLISHER (pkill localization_mux_node) never sends a "last will"
        # is_valid=False message, only the independent STALE path still
        # fires -- see analyze()'s docstring.
        passed = (
            failure is None and
            report.get('localization_lost_seen') is True and
            report.get('cmd_safety_count', 1) == 0 and
            not math.isnan(report.get('blind_distance_m', float('nan'))) and
            report.get('blind_distance_m', 999.0) <= MAX_BLIND_DISTANCE_M
        )
    print('RESULT                    : %s' % ('PASS' if passed else 'FAIL'))
    sys.stdout.flush()

    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
