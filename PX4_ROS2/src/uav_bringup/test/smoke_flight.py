#!/usr/bin/env python3
"""Regression flight: arm -> takeoff -> goto -> land -> disarm. Exit 0 = all thresholds met."""

import argparse
import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from uav_interfaces.msg import (
    ControlAuthority, ControlCommand, OffboardStatus, SafetyState, VehicleState)
from uav_interfaces.srv import Arm, Disarm, SetFlightMode

AUTHORITY_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Same shape, and it has to be: /safety/state is Reliable/TransientLocal (contract 2.18).
# Subscribing with anything else delivers NOTHING, with no error anywhere -- which would
# read as "safety was quiet".
SAFETY_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Prefixes of recommended_action that mean the supervisor wanted the aircraft. Contract
# 2.18 requires prefix comparison: the dry-run forms carry a trailing explanation.
SAFETY_ACTING_PREFIXES = ('hold', 'inhibit', 'would_hold', 'would_inhibit')

# Codes seen on a healthy M5 that have a written reason. A code NOT in here is reported as
# unexplained rather than assumed benign -- this list is not a place to quiet things down.
SAFETY_EXPLAINED_CODES = {
    'OFFBOARD_UNHEALTHY':
        'M5 stops streaming during the landing phase on purpose (monitoring=False there), '
        'so the supervisor correctly sees the offboard link go quiet',
    # Traced 2026-08-26 with the recorders run_d3_p5_dynamic.sh now keeps. The cause is
    # NOT a gap and NOT a lost frame: over the flight every stream held 0 lost frames and
    # a longest gap of 0.04-0.07 s. What happened was 2 samples out of 216 on the
    # DETAILED array reading 'flat image, may be legitimate' -- camera_health_node's own
    # words -- on front/rgb (distinct_values 2, against 47-256 in normal samples) and
    # front/depth in the same instant. Both front sensors going flat together is the
    # aircraft climbing with nothing ahead of it: uav_arena is empty in front by design
    # (the obstacle gate has to SPAWN a box to see anything), so RGB is near-uniform sky
    # and depth has nothing inside its 19.1 m clip. camera_grace_sec is 0.0 on purpose
    # (safety_params.yaml: REPORT-only, never latches, and no numbered threshold exists
    # yet), so one such sample violates immediately. recommended_action stayed 'none' in
    # every sample -- the aircraft was never asked to do anything about it.
    'CAMERA_STREAM_UNHEALTHY':
        'the front pair reads flat while the aircraft climbs with empty arena ahead of '
        'it (RGB distinct_values 2, depth uniform inside its 19.1 m clip); '
        'camera_grace_sec is 0 s by design so one sample is enough, and the code is '
        'REPORT-only -- recommended_action never left none',
}

# An explanation that covers every cause of a code is not a shield, it is an amnesty:
# a dead bridge and an empty sky raise the SAME code. So CAMERA_STREAM_UNHEALTHY is
# explained only while the detailed diagnostics say the degradation was the flat-image
# one that was actually traced. Anything else -- a gap, a lost frame, a stream that
# stopped -- leaves it unexplained and blocking, which is where it started.
CAMERA_EXPLAINED_REASONS = ('flat image',)

SAFETY_LEVEL_NAMES = {
    SafetyState.LEVEL_UNKNOWN: 'UNKNOWN',
    SafetyState.LEVEL_OK: 'OK',
    SafetyState.LEVEL_WARNING: 'WARNING',
    SafetyState.LEVEL_ERROR: 'ERROR',
    SafetyState.LEVEL_EMERGENCY: 'EMERGENCY',
}

TAKEOFF_ALTITUDE = 2.5          # m above the starting point
GOTO_EAST = 3.0                 # m east of the starting point
ALTITUDE_TOLERANCE = 0.3        # m
HORIZONTAL_TOLERANCE = 0.5      # m
SETTLE_SECONDS = 2.0            # how long the target must be held
ARM_BUDGET_SECONDS = 120.0      # PX4 preflight clears on its own; see armWithinBudget
ARM_RETRY_SECONDS = 5.0
MIN_STREAM_RATE = 2.0           # Hz, PX4 drops offboard below this

STATE_ACTIVE = OffboardStatus.STATE_ACTIVE
STATE_FAULT = OffboardStatus.STATE_FAULT
MODE_LAND = VehicleState.FLIGHT_MODE_LAND


class Failure(Exception):
    """Raised when a threshold is missed; carries the reason for the report."""


class SmokeFlight(Node):
    def __init__(self, uav_id, use_sim_time=True):
        # Must share the backend's clock; see README.
        super().__init__(
            'smoke_flight',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)],
        )
        prefix = '/uav/' + uav_id

        self.uav_id = uav_id
        self.state = None
        self.odom = None
        self.offboard = None
        self.target = None          # None means publish nothing
        self.anchor = None          # first flight's start, so repeats stay in place
        self.monitoring = False     # off while landing, where offboard loss is expected
        self.safety_violations = []
        # Observed, not judged -- see this file's S8 note. worst_level starts at None so
        # "never received" stays distinguishable from "received, and it was OK".
        self.safety_samples = 0
        self.safety_worst_level = None
        self.safety_codes = []
        self.safety_actions = []
        self.camera_degraded_reasons = set()
        self.authority = None
        self._authority_stamp = None    # Y16/B2-d3: SIM-time of the last authority msg

        self.command_pub = self.create_publisher(
            ControlCommand, prefix + '/control/cmd_test', 10)
        self.create_subscription(VehicleState, prefix + '/state/vehicle', self._on_state, 10)
        self.create_subscription(Odometry, prefix + '/state/odometry_raw', self._on_odom, 10)
        self.create_subscription(
            OffboardStatus, prefix + '/backend/offboard_status', self._on_offboard, 10)
        self.create_subscription(
            ControlAuthority, prefix + '/control/authority', self._on_authority, AUTHORITY_QOS)
        self.create_subscription(
            SafetyState, prefix + '/safety/state', self._on_safety, SAFETY_QOS)
        self.create_subscription(
            DiagnosticArray, prefix + '/diagnostics/perception', self._on_perception, 10)

        self.arm_client = self.create_client(Arm, prefix + '/backend/arm')
        self.disarm_client = self.create_client(Disarm, prefix + '/backend/disarm')
        self.mode_client = self.create_client(SetFlightMode, prefix + '/backend/set_mode')

        self.create_timer(0.05, self._stream)      # 20 Hz


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
        if self.monitoring and msg.failsafe_active:
            self._violation('failsafe activated')

    def _on_odom(self, msg):
        self.odom = msg

    def _on_authority(self, msg):
        self.authority = msg
        self._authority_stamp = self.get_clock().now()

    def _authority_is_fresh(self, source):
        # Y16: TransientLocal means a STALE cached message (from before this
        # node started, or from a previous flight) can satisfy an active_source
        # check without a single new message ever having arrived -- checking
        # only the value, never freshness, cannot tell "granted and confirmed
        # alive" from "last known value, node might be dead". 2 Hz heartbeat
        # means a healthy arbiter always has something well under 1 s old.
        #
        # B2-d3: measured on SIM time (this node runs use_sim_time=True, same
        # as control_authority_manager_node), not the wall clock -- under a
        # low RTF a wall-clock "< 1 s" window can elapse before even one
        # sim-time heartbeat period has passed, aborting the flight on a
        # false "is control_authority_manager_node running?".
        if self.authority is None or self._authority_stamp is None:
            return False
        if self.authority.active_source != source:
            return False
        age_sec = (self.get_clock().now() - self._authority_stamp).nanoseconds * 1e-9
        return age_sec < 1.0

    def _on_offboard(self, msg):
        self.offboard = msg
        if not self.monitoring:
            return
        if msg.state == STATE_FAULT:
            self._violation('offboard status went to FAULT: ' + msg.detail)
        elif msg.state == STATE_ACTIVE and msg.setpoint_rate_hz < MIN_STREAM_RATE:
            self._violation('setpoint rate fell to %.1f Hz' % msg.setpoint_rate_hz)

    def _on_safety(self, msg):
        """Record what the supervisor said. No verdict yet -- see the S8 note above."""
        self.safety_samples += 1
        if self.safety_worst_level is None or msg.level > self.safety_worst_level:
            self.safety_worst_level = msg.level
        for code in msg.violation_codes:
            if code not in self.safety_codes:
                self.safety_codes.append(code)
        action = (msg.recommended_action or '').strip()
        if action and action not in self.safety_actions:
            self.safety_actions.append(action)

    def safetyActedCodes(self):
        """recommended_action values meaning the supervisor wanted to take the aircraft."""
        return [a for a in self.safety_actions if a.startswith(SAFETY_ACTING_PREFIXES)]

    def _on_perception(self, msg):
        for status in msg.status:
            if status.level != 0 and 'camera' in status.name:
                self.camera_degraded_reasons.add(status.message)

    def cameraReasonsOutsideTheExplanation(self):
        return sorted(
            r for r in self.camera_degraded_reasons
            if not any(known in r for known in CAMERA_EXPLAINED_REASONS))

    def safetyUnexplainedCodes(self):
        unexplained = [c for c in self.safety_codes if c not in SAFETY_EXPLAINED_CODES]
        # The camera explanation only holds for the reason it was written about, and
        # "no detail arrived" is not evidence that the benign one applies (O3).
        if 'CAMERA_STREAM_UNHEALTHY' in self.safety_codes:
            if not self.camera_degraded_reasons or self.cameraReasonsOutsideTheExplanation():
                unexplained.append('CAMERA_STREAM_UNHEALTHY')
        return unexplained

    def safetyFailure(self):
        """Reason M5 should fail on safety grounds, or None.

        Silence is a failure too: /safety/state is TransientLocal, so a subscriber with the
        wrong QoS receives nothing at all and no error is raised anywhere. An empty record
        must never read as 'safety was quiet' (O3).
        """
        if self.safety_samples == 0:
            return ('/safety/state never arrived -- QoS mismatch or the supervisor was not '
                    'running. Not measured is not OK')
        acted = self.safetyActedCodes()
        if acted:
            return ('safety wanted the aircraft during a regression flight: '
                    'recommended_action reached %s (codes: %s)'
                    % (acted, self.safety_codes or 'none'))
        return None

    def safetyReport(self):
        if self.safety_samples == 0:
            return 'NO SAMPLE -- /safety/state never arrived (QoS mismatch, or safety not running)'
        level = SAFETY_LEVEL_NAMES.get(self.safety_worst_level, str(self.safety_worst_level))
        unexplained = self.safetyUnexplainedCodes()
        note = ''
        if unexplained:
            note = '  ⚠️ UNEXPLAINED: %s' % unexplained
        return 'worst=%s samples=%d actions=%s codes=%s%s' % (
            level, self.safety_samples, self.safety_actions or 'none',
            self.safety_codes or 'none', note)

    def _violation(self, reason):
        if reason not in self.safety_violations:
            self.safety_violations.append(reason)


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

    def hold_until(self, predicate, timeout, description):
        """Requires the condition to stay true for SETTLE_SECONDS, not just blink."""
        end = time.time() + timeout
        stable_since = None
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                stable_since = stable_since or time.time()
                if time.time() - stable_since >= SETTLE_SECONDS:
                    return
            else:
                stable_since = None
        raise Failure('timed out waiting for %s' % description)

    def position(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
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


    def safetyBlockers(self):
        """Everything in the supervisor's report that must stop a PASS.

        An undocumented code counts. LOCALIZATION_JUMP printed a warning and passed for
        days; measured on 2026-08-25 it was a real 2.217 m step in 100 ms on the pose
        every planner, the mission and the supervisor itself read. A warning nobody is
        obliged to act on is a warning nobody reads.
        """
        blockers = []
        acting = self.safetyFailure()
        if acting:
            blockers.append(acting)
        unexplained = self.safetyUnexplainedCodes()
        if unexplained:
            blockers.append('safety raised %s and nobody has written down what it means'
                            % ', '.join(unexplained))
        return blockers


    def armWithinBudget(self):
        """Wait out a transient preflight refusal instead of failing the whole run.

        Measured 2026-08-25: EKF2 reset its quaternion mid-flight, estimator_status
        mag_test_ratio went non-finite, and the next arm was denied by the
        COM_ARM_EKF_YAW check -- while yaw was within 1.7 deg of Gazebo ground truth.
        The aircraft really was not ready; giving up after one call was this test's
        defect, not the vehicle's. The wait is REPORTED, never absorbed: a flight that
        needed a minute to arm is itself a finding.
        """
        started = time.time()
        tries = 0
        last = ''
        while time.time() - started < ARM_BUDGET_SECONDS:
            tries += 1
            try:
                self.call(self.arm_client, Arm.Request(), 15.0, 'arm')
                return time.time() - started, tries
            except Failure as refusal:
                last = str(refusal)
                self.spin(ARM_RETRY_SECONDS)
        raise Failure('arm refused for %.0f s over %d attempt(s); last: %s'
                      % (time.time() - started, tries, last))


    def fly_once(self, index):
        report = {'flight': index}

        # Y16: a stale ControlAuthority carried over from a PREVIOUS flight
        # (or, on flight 1, a latched message from before this node existed)
        # must never be mistaken for THIS flight's own confirmation.
        self.authority = None
        self._authority_stamp = None

        self.wait_until(
            lambda: self.state is not None and self.state.connected,
            60.0, 'autopilot connection')
        self.wait_until(lambda: self.odom is not None, 30.0, 'odometry')

        origin = self.position()
        report['origin_z'] = origin[2]

        # Gives the gateway something to stream while priming.
        self.target = origin
        # Stale ACTIVE would satisfy the wait below; see README.
        self.offboard = None
        self.spin(1.5)

        self.wait_until(
            lambda: self._authority_is_fresh(ControlAuthority.SOURCE_TEST),
            10.0, 'control authority granted to TEST (fresh within 1 s) -- '
            'is control_authority_manager_node running?')

        # Offboard first, then arm; see README.
        self.wait_until(
            lambda: self.offboard is not None and self.offboard.state == STATE_ACTIVE,
            30.0, 'offboard engagement')
        report['arm_wait'], report['arm_tries'] = self.armWithinBudget()
        self.monitoring = True      # guard only the powered-flight phase

        # --- takeoff ---
        cruise = (origin[0], origin[1], origin[2] + TAKEOFF_ALTITUDE)
        self.target = cruise
        self.hold_until(
            lambda: abs(self.position()[2] - cruise[2]) <= ALTITUDE_TOLERANCE,
            60.0, 'takeoff altitude')
        report['altitude_error'] = abs(self.position()[2] - cruise[2])

        # --- goto ---
        # Shuttle, not walk east: 3 flights would exit the room.
        if self.anchor is None:
            self.anchor = origin
        east = GOTO_EAST if index % 2 == 1 else 0.0
        waypoint = (self.anchor[0] + east, self.anchor[1], origin[2] + TAKEOFF_ALTITUDE)
        self.target = waypoint
        self.hold_until(
            lambda: math.dist(self.position()[:2], waypoint[:2]) <= HORIZONTAL_TOLERANCE,
            60.0, 'waypoint arrival')
        report['horizontal_error'] = math.dist(self.position()[:2], waypoint[:2])

        # --- land ---
        # Mode first, stop streaming after; reverse trips failsafe.
        request = SetFlightMode.Request()
        request.mode = MODE_LAND
        self.monitoring = False     # handing control over, so stop guarding offboard
        self.call(self.mode_client, request, 15.0, 'set land mode')
        self.spin(1.0)
        self.target = None

        landed = time.time()
        try:
            self.wait_until(lambda: not self.state.armed, 40.0, 'automatic disarm')
            report['disarm'] = 'automatic'
        except Failure:
            self.call(self.disarm_client, Disarm.Request(), 15.0, 'disarm')
            report['disarm'] = 'commanded'
        report['land_seconds'] = time.time() - landed

        self.spin(3.0)          # let things settle before the next flight
        return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--flights', type=int, default=3)
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument(
        '--no-sim-time', action='store_true',
        help='Use the wall clock. Required on real hardware and without /clock.')
    args = parser.parse_args()

    rclpy.init()
    node = SmokeFlight(args.uav_id, use_sim_time=not args.no_sim_time)

    reports = []
    failure = None
    try:
        for i in range(1, args.flights + 1):
            print('--- flight %d/%d ---' % (i, args.flights))
            sys.stdout.flush()
            reports.append(node.fly_once(i))
            print('    ok')
            sys.stdout.flush()
    except Failure as exc:
        failure = str(exc)
    except KeyboardInterrupt:
        failure = 'interrupted'
    finally:
        node.monitoring = False
        node.target = None

    print('')
    print('=== SMOKE FLIGHT REPORT ===')
    for r in reports:
        print('flight %d: alt_err=%.2f m  horiz_err=%.2f m  land=%.0f s (%s)'
              '  arm=%.1f s over %d try(s)'
              % (r['flight'], r['altitude_error'], r['horizontal_error'],
                 r['land_seconds'], r['disarm'], r['arm_wait'], r['arm_tries']))
    print('completed  : %d/%d' % (len(reports), args.flights))
    # Renamed in the report because the old label lied: this list is fed only by
    # _on_offboard(), so "violations: none" only ever meant the offboard link was healthy.
    print('offboard   : %s' % (node.safety_violations or 'no fault'))
    print('safety     : %s' % node.safetyReport())
    if failure:
        print('FAILURE    : %s' % failure)

    blockers = node.safetyBlockers()
    for blocker in blockers:
        print('safety FAIL: %s' % blocker)
    passed = (failure is None and not node.safety_violations
              and not blockers and len(reports) == args.flights)
    print('RESULT     : %s' % ('PASS' if passed else 'FAIL'))
    sys.stdout.flush()

    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
