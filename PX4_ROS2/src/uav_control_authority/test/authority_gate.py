#!/usr/bin/env python3
"""G-CA2: TEST flies a Goto leg, a fake OPERATOR steals authority mid-leg and
holds position, then releases it back to TEST. Run via verify_control_authority.sh.
No px4_msgs import (R1): PX4 cadence is read through /backend/offboard_status only.
"""

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from uav_interfaces.msg import ControlAuthority, ControlCommand, OffboardStatus, VehicleState
from uav_interfaces.srv import Arm, Disarm, SetFlightMode

TAKEOFF_ALTITUDE = 2.5          # m above the starting point
GOTO_EAST = 5.0                 # m east; long enough to still be mid-leg at inject time
ALTITUDE_TOLERANCE = 0.3        # m
HORIZONTAL_TOLERANCE = 0.5      # m
SETTLE_SECONDS = 2.0            # how long the waypoint must be held
MIN_STREAM_RATE = 2.0           # Hz, PX4 drops offboard below this (G-CA2 item b)

INJECT_AT_FRACTION = 0.4        # fraction of the leg covered before OPERATOR steals authority
INJECT_HOLD_SEC = 5.0           # how long the fake OPERATOR holds position
POST_LAND_WATCH_SEC = 5.0       # extra window to prove b's own measurement can go red (item f)

AUTHORITY_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

STATE_ACTIVE = OffboardStatus.STATE_ACTIVE
STATE_FAULT = OffboardStatus.STATE_FAULT
MODE_LAND = VehicleState.FLIGHT_MODE_LAND
SOURCE_TEST = ControlAuthority.SOURCE_TEST
SOURCE_OPERATOR = ControlAuthority.SOURCE_OPERATOR


class Failure(Exception):
    """Raised when the flight cannot be completed; the run reports no gate result."""


class AuthorityGate(Node):
    def __init__(self, uav_id, use_sim_time=True):
        super().__init__(
            'authority_gate',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)],
        )
        prefix = '/uav/' + uav_id

        self.uav_id = uav_id
        self.state = None
        self.odom = None
        self.offboard = None
        self.authority = None
        self._authority_stamp = None    # Y16/B2-d3: SIM-time of the last authority msg
        self.target = None              # TEST stream target; None means publish nothing
        self.operator_target = None     # OPERATOR stream target; None means publish nothing
        self.monitoring = False         # True only while actually flying (guards item b's window)
        self.safety_violations = []

        # Continuous logs, timestamped on the sim clock so windows can be sliced later.
        self.authority_log = []         # (t, active_source, previous_source, reason)
        self.offboard_log = []          # (t, monitoring, state, setpoint_rate_hz)
        self.command_log = []           # (t, source, x, y, z) from command_selected

        self.command_pub = self.create_publisher(ControlCommand, prefix + '/control/cmd_test', 10)
        self.operator_pub = self.create_publisher(
            ControlCommand, prefix + '/control/cmd_operator', 10)
        self.create_subscription(VehicleState, prefix + '/state/vehicle', self._on_state, 10)
        self.create_subscription(Odometry, prefix + '/state/odometry_raw', self._on_odom, 10)
        self.create_subscription(
            OffboardStatus, prefix + '/backend/offboard_status', self._on_offboard, 10)
        self.create_subscription(
            ControlAuthority, prefix + '/control/authority', self._on_authority, AUTHORITY_QOS)
        self.create_subscription(
            ControlCommand, prefix + '/control/command_selected', self._on_command, 50)

        self.arm_client = self.create_client(Arm, prefix + '/backend/arm')
        self.disarm_client = self.create_client(Disarm, prefix + '/backend/disarm')
        self.mode_client = self.create_client(SetFlightMode, prefix + '/backend/set_mode')

        self.create_timer(0.05, self._stream_test)      # 20 Hz
        self.create_timer(0.05, self._stream_operator)  # 20 Hz


    def _require_well_formed(self, cmd):
        # The arbiter DROPs these silently (plan S:3c); catch it here so a bug
        # aborts loudly instead of looking like a hung flight.
        if cmd.header.frame_id != 'odom':
            raise Failure('probe command frame_id %r is not odom' % cmd.header.frame_id)
        if cmd.header.stamp.sec == 0 and cmd.header.stamp.nanosec == 0:
            raise Failure('probe command stamp is unset')
        fields = (cmd.position.x, cmd.position.y, cmd.position.z, cmd.yaw)
        if not all(math.isfinite(v) for v in fields):
            raise Failure('probe command has a non-finite field: %r' % (fields,))

    def _build_command(self, target, source):
        cmd = ControlCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'odom'
        cmd.uav_id = self.uav_id
        cmd.control_mode = ControlCommand.MODE_POSITION
        cmd.position.x, cmd.position.y, cmd.position.z = target
        cmd.yaw = 0.0
        cmd.source = source
        self._require_well_formed(cmd)
        return cmd

    def _stream_test(self):
        if self.target is None:
            return
        self.command_pub.publish(self._build_command(self.target, SOURCE_TEST))

    def _stream_operator(self):
        if self.operator_target is None:
            return
        self.operator_pub.publish(self._build_command(self.operator_target, SOURCE_OPERATOR))

    def _on_state(self, msg):
        self.state = msg
        if self.monitoring and msg.failsafe_active:
            self._violation('failsafe activated')

    def _on_odom(self, msg):
        self.odom = msg

    def _on_authority(self, msg):
        self.authority = msg
        self._authority_stamp = self.sim_seconds()
        self.authority_log.append((self.sim_seconds(), msg.active_source,
                                    msg.previous_source, msg.reason))

    def _authority_is_fresh(self, source):
        # Y16: TransientLocal means a STALE cached message can satisfy an
        # active_source value check without a single new message ever having
        # arrived -- freshness (2 Hz heartbeat, so well under 1 s when alive)
        # is what actually tells "granted and confirmed alive" from "last
        # known value, arbiter might be dead".
        #
        # B2-d3: measured on SIM time (self.sim_seconds(), same clock as
        # control_authority_manager_node's own use_sim_time=True), not the
        # wall clock -- under a low RTF a wall-clock "< 1 s" window can elapse
        # before even one sim-time heartbeat period has passed, aborting the
        # flight on a false "is control_authority_manager_node running?".
        if self.authority is None or self._authority_stamp is None:
            return False
        if self.authority.active_source != source:
            return False
        return (self.sim_seconds() - self._authority_stamp) < 1.0

    def _on_offboard(self, msg):
        self.offboard = msg
        self.offboard_log.append((self.sim_seconds(), self.monitoring, msg.state,
                                   msg.setpoint_rate_hz))
        if not self.monitoring:
            return
        if msg.state == STATE_FAULT:
            self._violation('offboard status went to FAULT: ' + msg.detail)
        elif msg.state == STATE_ACTIVE and msg.setpoint_rate_hz < MIN_STREAM_RATE:
            self._violation('setpoint rate fell to %.1f Hz' % msg.setpoint_rate_hz)

    def _on_command(self, msg):
        # source is recorded too: the /control/authority topic is a SEPARATE
        # stream, and its notification can arrive at this subscriber a tick
        # after (or before) the correlated command_selected message -- the
        # only reliable way to find the true transition boundary is to look
        # at command_selected's OWN source field, not to cross-correlate by
        # wall/sim timestamp against a different topic.
        self.command_log.append((self.sim_seconds(), msg.source, msg.position.x,
                                  msg.position.y, msg.position.z))

    def _violation(self, reason):
        if reason not in self.safety_violations:
            self.safety_violations.append(reason)


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

    def hold_until(self, predicate, timeout, description):
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


    def fly(self):
        report = {}

        self.wait_until(
            lambda: self.state is not None and self.state.connected, 60.0, 'autopilot connection')
        self.wait_until(lambda: self.odom is not None, 30.0, 'odometry')

        origin = self.position()

        # Gives the gateway something to stream while priming.
        self.target = origin
        self.offboard = None
        self.spin(1.5)

        self.wait_until(
            lambda: self._authority_is_fresh(SOURCE_TEST),
            10.0, 'control authority granted to TEST (fresh within 1 s) -- '
            'is control_authority_manager_node running?')

        self.wait_until(
            lambda: self.offboard is not None and self.offboard.state == STATE_ACTIVE,
            30.0, 'offboard engagement')
        self.call(self.arm_client, Arm.Request(), 15.0, 'arm')
        self.monitoring = True

        # --- takeoff ---
        cruise = (origin[0], origin[1], origin[2] + TAKEOFF_ALTITUDE)
        self.target = cruise
        self.hold_until(
            lambda: abs(self.position()[2] - cruise[2]) <= ALTITUDE_TOLERANCE,
            60.0, 'takeoff altitude')
        report['altitude_error'] = abs(self.position()[2] - cruise[2])

        # --- goto leg, with a fake OPERATOR injection mid-way ---
        waypoint = (origin[0] + GOTO_EAST, origin[1], origin[2] + TAKEOFF_ALTITUDE)
        self.target = waypoint
        leg_start = self.position()
        leg_distance = math.dist(leg_start[:2], waypoint[:2])
        # Trigger on distance covered, not a fixed delay: a fixed delay can be
        # wrong by a lot if PX4's own transit speed differs from the guess,
        # and arriving before injecting makes item c's continuity check
        # vacuous (a 0.000 m "jump" that never actually tested anything).
        self.wait_until(
            lambda: math.dist(self.position()[:2], leg_start[:2]) >= INJECT_AT_FRACTION * leg_distance,
            30.0, 'reaching the midpoint of the goto leg before injecting OPERATOR')

        hold_position = self.position()
        report['inject_start_sim_t'] = self.sim_seconds()
        self.operator_target = hold_position
        self.wait_until(
            lambda: self._authority_is_fresh(SOURCE_OPERATOR),
            5.0, 'authority handed up to OPERATOR (fresh within 1 s)')
        report['operator_granted_sim_t'] = self.sim_seconds()

        self.spin(INJECT_HOLD_SEC)

        report['inject_stop_sim_t'] = self.sim_seconds()
        self.operator_target = None
        self.wait_until(
            lambda: self._authority_is_fresh(SOURCE_TEST),
            5.0, 'authority released back to TEST (fresh within 1 s)')
        report['test_regained_sim_t'] = self.sim_seconds()

        # Finish the original leg -- self.target never moved during the injection.
        self.hold_until(
            lambda: math.dist(self.position()[:2], waypoint[:2]) <= HORIZONTAL_TOLERANCE,
            60.0, 'waypoint arrival')
        report['horizontal_error'] = math.dist(self.position()[:2], waypoint[:2])

        # --- land ---
        report['land_started_sim_t'] = self.sim_seconds()
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

        # Item f: keep recording after everything has gone quiet, so the
        # negative control below has a real post-land window to look at.
        self.spin(POST_LAND_WATCH_SEC)

        return report


def analyse(node, report):
    """Post-flight: turn the three logs into the a-f verdicts (R0: measured, not assumed)."""
    result = {'failures': []}

    # a. authority sequence TEST -> OPERATOR -> TEST, in order, with timestamps.
    transitions = []
    last_source = None
    for t, active, previous, reason in node.authority_log:
        if active != last_source:
            transitions.append((t, active, previous, reason))
            last_source = active
    result['transitions'] = transitions
    sources_seen = [s for _, s, _, _ in transitions]
    up_index = None
    down_index = None
    for i in range(len(sources_seen) - 1):
        if sources_seen[i] == SOURCE_TEST and up_index is None:
            # first OPERATOR after this TEST
            for j in range(i + 1, len(sources_seen)):
                if sources_seen[j] == SOURCE_OPERATOR:
                    up_index = j
                    break
            if up_index is not None:
                for k in range(up_index + 1, len(sources_seen)):
                    if sources_seen[k] == SOURCE_TEST:
                        down_index = k
                        break
                break
    result['sequence_ok'] = up_index is not None and down_index is not None
    if not result['sequence_ok']:
        result['failures'].append('a: TEST->OPERATOR->TEST sequence not observed on /control/authority')

    # b. offboard status during the monitored flight window: never FAULT, never < 2 Hz.
    flight_samples = [(t, state, rate) for t, mon, state, rate in node.offboard_log if mon]
    result['flight_samples'] = len(flight_samples)
    result['b_min_rate'] = min((r for _, _, r in flight_samples), default=None)
    result['b_had_fault'] = any(s == STATE_FAULT for _, s, _ in flight_samples)
    if not flight_samples:
        result['failures'].append('b: FAILED TO MEASURE - no offboard_status samples during the flight window')
    else:
        if result['b_had_fault']:
            result['failures'].append('b: offboard_status hit STATE_FAULT during the flight')
        if result['b_min_rate'] < MIN_STREAM_RATE:
            result['failures'].append(
                'b: setpoint_rate_hz dropped to %.2f Hz during the flight (< %.1f)'
                % (result['b_min_rate'], MIN_STREAM_RATE))

    # c/d. continuity + max gap around each transition on command_selected.
    # Find the boundary from command_selected's OWN source field, not by
    # cross-correlating timestamps against the separate /control/authority
    # topic -- that topic's notification can arrive a tick before or after
    # the correlated command_selected message at THIS subscriber, which
    # silently mis-brackets the pair and makes the jump read as 0.
    def find_boundary(sample_list, from_source, to_source):
        for i in range(len(sample_list) - 1):
            if sample_list[i][1] == from_source and sample_list[i + 1][1] == to_source:
                return sample_list[i], sample_list[i + 1]
        return None

    def jump_and_gap(pair):
        if pair is None:
            return None, None
        (t0, s0, x0, y0, z0), (t1, s1, x1, y1, z1) = pair
        return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2), t1 - t0

    up_pair = find_boundary(node.command_log, SOURCE_TEST, SOURCE_OPERATOR)
    down_pair = find_boundary(node.command_log, SOURCE_OPERATOR, SOURCE_TEST)
    result['up_jump_m'], result['up_gap_s'] = jump_and_gap(up_pair)
    result['down_jump_m'], result['down_gap_s'] = jump_and_gap(down_pair)
    if up_pair is None:
        result['failures'].append('c/d: no TEST->OPERATOR boundary found on command_selected itself')
    if down_pair is None:
        result['failures'].append('c/d: no OPERATOR->TEST boundary found on command_selected itself')

    def window_max_gap(pair, radius=1.5):
        if pair is None:
            return None
        center_t = pair[0][0]
        samples = sorted(s[0] for s in node.command_log if abs(s[0] - center_t) <= radius)
        if len(samples) < 2:
            return None
        return max(b - a for a, b in zip(samples, samples[1:]))

    result['up_window_max_gap_s'] = window_max_gap(up_pair)
    result['down_window_max_gap_s'] = window_max_gap(down_pair)

    # f. positive control: after land+disarm, the very same measurement must go red.
    post_land_samples = [(t, state, rate) for t, mon, state, rate in node.offboard_log if not mon]
    # Ignore the pre-arm samples at the head of the log (also monitoring=False).
    post_land_samples = [s for s in post_land_samples if s[0] >= report['land_started_sim_t']]
    result['post_land_samples'] = len(post_land_samples)
    result['f_min_rate'] = min((r for _, _, r in post_land_samples), default=None)
    result['f_detected_drop'] = (
        result['f_min_rate'] is not None and result['f_min_rate'] < MIN_STREAM_RATE)
    if not post_land_samples:
        result['failures'].append('f: FAILED TO MEASURE - no offboard_status samples after landing')
    elif not result['f_detected_drop']:
        result['failures'].append(
            'f: setpoint_rate_hz never dropped below %.1f Hz after landing (min seen %.2f) -- '
            'the b measurement has not been proven to detect a real problem'
            % (MIN_STREAM_RATE, result['f_min_rate']))

    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument(
        '--no-sim-time', action='store_true',
        help='Use the wall clock. Required on real hardware and without /clock.')
    args = parser.parse_args()

    rclpy.init()
    node = AuthorityGate(args.uav_id, use_sim_time=not args.no_sim_time)

    report = None
    failure = None
    try:
        report = node.fly()
    except Failure as exc:
        failure = str(exc)
    except KeyboardInterrupt:
        failure = 'interrupted'
    finally:
        node.monitoring = False
        node.target = None
        node.operator_target = None

    print()
    print('=== G-CA2 CONTROL AUTHORITY FLIGHT GATE ===')
    if failure:
        print('FAILURE (flight did not complete): %s' % failure)
        print('authority transitions logged so far: %d' % len(node.authority_log))
        print('offboard samples logged so far     : %d' % len(node.offboard_log))
        print('command_selected samples logged so far: %d' % len(node.command_log))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    result = analyse(node, report)

    print('altitude error   %.3f m' % report['altitude_error'])
    print('horizontal error %.3f m' % report['horizontal_error'])
    print('land             %.0f s (%s)' % (report['land_seconds'], report['disarm']))
    print('safety violations: %s' % (node.safety_violations or 'none'))
    print()
    print('--- a. authority sequence (dedup, first value of each run) ---')
    for t, active, previous, reason in result['transitions']:
        print('  t=%.2f  active=%d previous=%d  reason=%s' % (t, active, previous, reason))
    print('  [%s] a. TEST -> OPERATOR -> TEST observed in order'
          % ('PASS' if result['sequence_ok'] else 'FAIL'))

    print()
    print('--- b. offboard status across the monitored flight window ---')
    print('  samples %d, min setpoint_rate_hz %s, STATE_FAULT seen: %s'
          % (result['flight_samples'],
             ('%.2f' % result['b_min_rate']) if result['b_min_rate'] is not None else 'n/a',
             result['b_had_fault']))
    print('  [%s] b. never STATE_FAULT, setpoint_rate_hz never < %.1f Hz'
          % ('PASS' if (result['flight_samples'] and not result['b_had_fault']
             and result['b_min_rate'] is not None and result['b_min_rate'] >= MIN_STREAM_RATE)
             else 'FAIL', MIN_STREAM_RATE))

    print()
    print('--- c. position continuity across the two transitions ---')
    print('  up   (TEST->OPERATOR): jump %s m, gap %s s'
          % (('%.4f' % result['up_jump_m']) if result['up_jump_m'] is not None else 'n/a',
             ('%.3f' % result['up_gap_s']) if result['up_gap_s'] is not None else 'n/a'))
    print('  down (OPERATOR->TEST): jump %s m, gap %s s'
          % (('%.4f' % result['down_jump_m']) if result['down_jump_m'] is not None else 'n/a',
             ('%.3f' % result['down_gap_s']) if result['down_gap_s'] is not None else 'n/a'))
    print('  reference only (not a hard gate): G-N6 handover jump 0.0254-0.0256 m')

    print()
    print('--- d. max inter-arrival gap on command_selected around each transition ---')
    print('  up window   max gap %s s'
          % (('%.3f' % result['up_window_max_gap_s'])
             if result['up_window_max_gap_s'] is not None else 'n/a'))
    print('  down window max gap %s s'
          % (('%.3f' % result['down_window_max_gap_s'])
             if result['down_window_max_gap_s'] is not None else 'n/a'))
    print('  reference only: unit-test measured 0.201 s, gateway command_timeout_sec cap 0.500 s')

    print()
    print('--- f. positive control: does the b measurement actually go red after landing? ---')
    print('  post-land samples %d, min setpoint_rate_hz %s'
          % (result['post_land_samples'],
             ('%.2f' % result['f_min_rate']) if result['f_min_rate'] is not None else 'n/a'))
    print('  [%s] f. setpoint_rate_hz dropped below %.1f Hz once the stream stopped'
          % ('PASS' if result['f_detected_drop'] else 'FAIL', MIN_STREAM_RATE))

    node.destroy_node()
    rclpy.shutdown()

    print()
    if result['failures']:
        print('RESULT: FAIL')
        for reason in result['failures']:
            print('  - ' + reason)
        return 1
    if node.safety_violations:
        print('RESULT: FAIL (safety violations logged above)')
        return 1
    print('RESULT: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
