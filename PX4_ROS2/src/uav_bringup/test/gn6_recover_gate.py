#!/usr/bin/env python3
"""G-N6: inject Recover mid-Goto, three times (HOLD / CLIMB / RETURN_HOME), plus
a quick TYPE_LAND rejection check. Run via scripts/run_gn6.sh. Model: uav0_nav.
Recover.action interface is frozen; the SERVER is being written in parallel -
see the report for every place this probe had to assume its behaviour.
NOT RUN by the author - sim access is reserved for the verifier-runner."""

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
from uav_interfaces.action import GotoPose, Land, Recover, Takeoff
from uav_interfaces.msg import ControlCommand, OffboardStatus, VehicleState

TAKEOFF_ALTITUDE = 2.5          # m, matches the other flight gates
LEG_LENGTH = 8.5                # m, above the "at least 8 m" requirement
LEG_SPEED = 0.5                 # m/s, per the D4 brief
ACCEPTANCE_RADIUS = 0.3         # m, navigator's shipped default

# Event-based trigger (R21): send Recover once the aircraft has actually moved
# this far from the goto start, never on a wall-clock guess.
PREEMPT_TRIGGER_DISTANCE_M = 3.0

CLIMB_DELTA_M = 1.5             # safe_altitude = z at preempt + 1.5, per the D4 brief
HOLD_RADIUS_M = 0.3             # per the D4 brief
HOLD_WINDOW_SEC = 3.0           # per the D4 brief
CLIMB_TOLERANCE_M = 0.3         # matches acceptance_radius: converging on an absolute z
Z_MONOTONIC_SLACK_M = 0.05      # numerical noise allowance for "must not lose altitude"
# navigation_params.yaml recovery.min_home_clearance_m = 1.0 ("a return that ends
# AT home is a landing") - the shipped config implies RETURN_HOME deliberately
# does NOT converge to 0 m from home. 1.0 + a 1.0 m pursuit/settle margin: this
# is a documented ASSUMPTION, not a value read from a running server - see report.
RETURN_HOME_HORIZONTAL_TOLERANCE_M = 2.0

# max_speed 0.55, stream_hz 20.0 (navigation_params.yaml): the largest step the
# setpoint stream may take between two ticks without a hidden jump.
MAX_SPEED_MPS = 0.55
STREAM_HZ = 20.0
CONTINUITY_STEP_LIMIT_M = (MAX_SPEED_MPS / STREAM_HZ) * 3.0
CONTINUITY_WINDOW_SEC = 1.0     # +- around the preempt moment

OFFBOARD_STREAM_FLOOR_HZ = 2.0  # PX4's hard requirement (CLAUDE.md S3 / memory R0)
RATE_WINDOW_SEC = 1.0

DETECT_SETTLE_SEC = 2.0         # how long a detector predicate must hold before we act on it
WALL_CLOCK_SAFETY_FACTOR = 6.0
MIN_WALL_ALLOWANCE_SEC = 15.0
EXPECTED_CHECKS = 22            # 1 TYPE_LAND + 3 trials x 7; a vanished check is not a PASS


class Failure(Exception):
    """Raised when the measurement itself cannot be trusted; no verdict follows."""


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def worst(values, label):
    """max() silently passes a NaN through; a missing sample must fail loudly."""
    if not values or any(value != value for value in values):
        raise Failure('%s has an unmeasured sample: %s' % (label, values))
    return max(values)


def longest_continuous_run_within(samples, reference, radius, axes):
    """samples: [(t, (x,y,z)), ...]. Longest span during which every sample
    stayed within `radius` of `reference` on the given axis indices."""
    best = 0.0
    run_start = None
    for t, point in samples:
        gap = math.sqrt(sum((point[a] - reference[a]) ** 2 for a in axes))
        if gap <= radius:
            if run_start is None:
                run_start = t
            best = max(best, t - run_start)
        else:
            run_start = None
    return best


def max_consecutive_step(commands):
    if len(commands) < 2:
        return 0.0
    return max(math.dist(commands[i][1], commands[i - 1][1]) for i in range(1, len(commands)))


def min_windowed_rate(timestamps, window_sec):
    """Same sliding-window technique as trajectory_gate.py's windowed_speed,
    applied to a publish rate instead of a speed."""
    if len(timestamps) < 2:
        return float('nan')
    rates = []
    start = 0
    for end in range(len(timestamps)):
        while timestamps[end] - timestamps[start] > window_sec:
            start += 1
        span = timestamps[end] - timestamps[start]
        if span >= window_sec * 0.5:
            rates.append((end - start) / span)
    return min(rates) if rates else float('nan')


class RecoverGate(Node):
    def __init__(self, uav_id):
        super().__init__(
            'gn6_recover_gate',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        prefix = '/uav/' + uav_id
        self.uav_id = uav_id

        self.state = None
        self.offboard = None
        self.position_now = None

        self.recording = False
        self.commands = []          # (sim seconds, (x, y, z))
        self.odometry = []          # (sim seconds, (x, y, z))
        self.state_samples = []     # (sim seconds, VehicleState)
        self.offboard_samples = []  # (sim seconds, OffboardStatus)

        self.create_subscription(Odometry, prefix + '/state/odometry_fused', self._on_odom, 20)
        self.create_subscription(
            ControlCommand, prefix + '/control/command_selected', self._on_command, 50)
        self.create_subscription(VehicleState, prefix + '/state/vehicle', self._on_state, 10)
        self.create_subscription(
            OffboardStatus, prefix + '/backend/offboard_status', self._on_offboard, 10)

        self.takeoff_client = ActionClient(self, Takeoff, prefix + '/planning/takeoff')
        self.goto_client = ActionClient(self, GotoPose, prefix + '/planning/goto_pose')
        self.land_client = ActionClient(self, Land, prefix + '/planning/land')
        # Interface pinned by the brief; the server (B4b) is being written in
        # parallel with this probe - see the report.
        self.recover_client = ActionClient(self, Recover, prefix + '/planning/recover')

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
                (message.position.x, message.position.y, message.position.z)))

    def _on_state(self, message):
        self.state = message
        if self.recording:
            self.state_samples.append((self.sim_seconds(), message))

    def _on_offboard(self, message):
        self.offboard = message
        if self.recording:
            self.offboard_samples.append((self.sim_seconds(), message))

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

    def wait_for_behavior_or_completion(self, handle, predicate, settle_sec, timeout, description):
        """Recover's own termination semantics are not frozen (server written in
        parallel - see the report): accept EITHER the goal finishing on its own,
        or our own detector confirming the target state for `settle_sec`, after
        which we cancel it ourselves so the trial can move on."""
        result_future = handle.get_result_async()
        wall_deadline = time.time() + self.wall_allowance(timeout)
        detected_since = None
        while rclpy.ok() and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if result_future.done():
                return result_future.result(), 'completed on its own'
            if predicate():
                detected_since = detected_since or time.time()
                if time.time() - detected_since >= settle_sec:
                    cancel_future = handle.cancel_goal_async()
                    self.wait_future(cancel_future, 10.0, '%s cancel' % description)
                    result = self.wait_future(
                        result_future, 15.0, '%s result after probe-initiated cancel' % description)
                    return result, 'target state confirmed for %.1fs, canceled by the probe' % settle_sec
            else:
                detected_since = None
        raise Failure(
            'timed out waiting for %s (neither natural completion nor the '
            'probe\'s own detector fired within %.0fs)' % (description, timeout))


def check_type_land_rejected(node, record):
    goal = Recover.Goal()
    goal.recovery_type = Recover.Goal.TYPE_LAND
    goal.safe_altitude = 0.0
    goal.trigger_reason = 'gn6 gate: TYPE_LAND must be refused (decision 3, contract 2.5)'
    node.await_server(node.recover_client, 30.0, 'recover')
    future = node.recover_client.send_goal_async(goal)
    handle = node.wait_future(future, 15.0, 'recover(TYPE_LAND) goal response')
    if not handle.accepted:
        record('0. Recover(TYPE_LAND) is refused', True,
               'goal was not accepted (rejected at the goal callback)')
        return
    result = node.wait_result(handle, 30.0, 'recover(TYPE_LAND) result')
    record('0. Recover(TYPE_LAND) is refused', not result.result.success,
           'goal was ACCEPTED (see report - assumed rejection happens at the goal '
           'callback); result success=%s message=%s'
           % (result.result.success, result.result.message))


def run_trial(node, record, name, recovery_type, home_xy):
    origin = node.position_now
    print('\n--- trial %s: takeoff from (%.3f, %.3f, %.3f) ---' % ((name,) + origin))

    node.recording = True
    node.commands = []
    node.odometry = []
    node.state_samples = []
    node.offboard_samples = []

    takeoff = Takeoff.Goal()
    takeoff.target_altitude = float(TAKEOFF_ALTITUDE)
    handle = node.send_goal(node.takeoff_client, takeoff, '%s takeoff' % name)
    result = node.wait_result(handle, 120.0, '%s takeoff' % name)
    if result.status != GoalStatus.STATUS_SUCCEEDED:
        raise Failure('%s: takeoff did not succeed: %s' % (name, result.result.message))
    takeoff_done_time = node.sim_seconds()

    hover_point = node.position_now
    goal_xyz = (hover_point[0] + LEG_LENGTH, hover_point[1], hover_point[2])

    goto = GotoPose.Goal()
    goto.target_pose.position.x = float(goal_xyz[0])
    goto.target_pose.position.y = float(goal_xyz[1])
    goto.target_pose.position.z = float(goal_xyz[2])
    goto.target_pose.orientation.w = 1.0
    goto.frame_id = 'odom'
    goto.acceptance_radius = float(ACCEPTANCE_RADIUS)
    goto.max_speed = float(LEG_SPEED)
    goto_handle = node.send_goal(node.goto_client, goto, '%s goto' % name)

    node.wait_until(
        lambda: node.position_now is not None
        and math.dist(node.position_now, hover_point) >= PREEMPT_TRIGGER_DISTANCE_M,
        90.0, '%s: aircraft to travel %.1f m before preempting' % (name, PREEMPT_TRIGGER_DISTANCE_M))
    preempt_position = node.position_now
    preempt_time = node.sim_seconds()
    print('%s: preempting at (%.3f, %.3f, %.3f), t=%.2f'
          % ((name,) + preempt_position + (preempt_time,)))

    recover_goal = Recover.Goal()
    recover_goal.recovery_type = recovery_type
    recover_goal.safe_altitude = float(preempt_position[2] + CLIMB_DELTA_M)
    recover_goal.trigger_reason = 'gn6 gate: mid-goto %s trial' % name
    recover_handle = node.send_goal(node.recover_client, recover_goal, '%s recover' % name)

    goto_result_future = goto_handle.get_result_async()
    node.wait_future(goto_result_future, 30.0, '%s: goto to be preempted' % name)
    goto_result = goto_result_future.result()
    print('%s: goto ended with status=%d message=%s'
          % (name, goto_result.status, goto_result.result.message))
    record('%s: 1. goto was preempted (CANCELED or ABORTED, not left running)' % name,
           goto_result.status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED),
           'status=%d (%s)' % (
               goto_result.status,
               'CANCELED' if goto_result.status == GoalStatus.STATUS_CANCELED
               else 'ABORTED' if goto_result.status == GoalStatus.STATUS_ABORTED
               else 'OTHER'))

    def frozen_setpoint():
        """Where a freeze actually parks the aircraft: the commanded position at
        the takeover, which leads the airframe by ~v/K_p (0.5 m at cruise). The
        aircraft settles THERE, not at its own position at the preempt instant."""
        for t, p in node.commands:
            if t >= preempt_time:
                return p
        return preempt_position

    if recovery_type == Recover.Goal.TYPE_HOLD:
        predicate = lambda: node.position_now is not None and math.dist(
            node.position_now, frozen_setpoint()) <= HOLD_RADIUS_M
        settle = HOLD_WINDOW_SEC
    elif recovery_type == Recover.Goal.TYPE_CLIMB_TO_SAFE_ALTITUDE:
        predicate = lambda: node.position_now is not None and abs(
            node.position_now[2] - recover_goal.safe_altitude) <= CLIMB_TOLERANCE_M
        settle = DETECT_SETTLE_SEC
    else:
        predicate = lambda: node.position_now is not None and math.hypot(
            node.position_now[0] - home_xy[0], node.position_now[1] - home_xy[1]
        ) <= RETURN_HOME_HORIZONTAL_TOLERANCE_M
        settle = DETECT_SETTLE_SEC

    recover_timeout = 180.0 if recovery_type == Recover.Goal.TYPE_RETURN_HOME else 90.0
    recover_result, how = node.wait_for_behavior_or_completion(
        recover_handle, predicate, settle, recover_timeout, '%s recover' % name)
    print('%s: recover result: status=%d success=%s executed_type=%d message=%s (%s)'
          % (name, recover_result.status, recover_result.result.success,
             recover_result.result.executed_type, recover_result.result.message, how))
    if recovery_type == Recover.Goal.TYPE_HOLD:
        # The recovery may complete on its own before the aircraft has settled on
        # the frozen setpoint; the hold that survives the goal is the claim, so
        # keep observing past the result before grading it.
        dwell_until = node.sim_seconds() + HOLD_WINDOW_SEC + 3.0
        node.wait_until(
            lambda: node.sim_seconds() >= dwell_until, 30.0, 'hold observation dwell')
    recover_end_time = node.sim_seconds()

    record('%s: 2. executed_type matches the type sent' % name,
           recover_result.result.executed_type == recovery_type,
           'sent %d, executed %d' % (recovery_type, recover_result.result.executed_type))

    post_preempt_odom = [(t, p) for t, p in node.odometry if preempt_time <= t <= recover_end_time]
    if not post_preempt_odom:
        raise Failure('%s: no odometry recorded between preempt and recover result' % name)

    if recovery_type == Recover.Goal.TYPE_HOLD:
        reference = frozen_setpoint()
        held = longest_continuous_run_within(
            post_preempt_odom, reference, HOLD_RADIUS_M, (0, 1, 2))
        record('%s: 3. HOLD kept position within 0.3 m of the frozen setpoint for >= 3 s' % name,
               held >= HOLD_WINDOW_SEC,
               'longest continuous run inside %.2f m of (%.2f, %.2f, %.2f): %.2f s (need %.1f)'
               % ((HOLD_RADIUS_M,) + tuple(reference) + (held, HOLD_WINDOW_SEC)))
    elif recovery_type == Recover.Goal.TYPE_CLIMB_TO_SAFE_ALTITUDE:
        z_values = [p[2] for _, p in post_preempt_odom]
        final_z = z_values[-1]
        reached = abs(final_z - recover_goal.safe_altitude) <= CLIMB_TOLERANCE_M
        dips = [max(0.0, a - b - Z_MONOTONIC_SLACK_M) for a, b in zip(z_values, z_values[1:])]
        worst_dip = worst(dips + [0.0], '%s climb altitude dip' % name)
        record('%s: 3. CLIMB reached safe_altitude +-0.3 m and never lost altitude' % name,
               reached and worst_dip <= 0.0,
               'final z %.3f m vs target %.3f m; worst dip between samples %.3f m'
               % (final_z, recover_goal.safe_altitude, worst_dip))
    else:
        tail = post_preempt_odom[-max(1, len(post_preempt_odom) // 5):]
        gaps = [math.hypot(p[0] - home_xy[0], p[1] - home_xy[1]) for _, p in tail]
        mean_gap = sum(gaps) / len(gaps)
        record('%s: 3. RETURN_HOME converged near the takeoff point' % name,
               mean_gap <= RETURN_HOME_HORIZONTAL_TOLERANCE_M,
               'mean horizontal distance to home over the tail %.3f m (limit %.2f, home=(%.3f,%.3f))'
               % (mean_gap, RETURN_HOME_HORIZONTAL_TOLERANCE_M, home_xy[0], home_xy[1]))

    window_commands = [
        (t, p) for t, p in node.commands
        if preempt_time - CONTINUITY_WINDOW_SEC <= t <= preempt_time + CONTINUITY_WINDOW_SEC]
    step = max_consecutive_step(window_commands)
    record('%s: 4. setpoint stayed continuous through the handover' % name,
           len(window_commands) >= 4 and step <= CONTINUITY_STEP_LIMIT_M,
           'largest consecutive step %.4f m over %d samples near t=%.2f (limit %.4f = '
           'max_speed/stream_hz x 3)' % (step, len(window_commands), preempt_time,
                                          CONTINUITY_STEP_LIMIT_M))

    # From the moment takeoff succeeded (armed+offboard are already established
    # by then) through the end of the recovery - "suot" (throughout), not just
    # around the handover.
    trial_state = [s for t, s in node.state_samples if takeoff_done_time <= t <= recover_end_time]
    trial_offboard = [
        s for t, s in node.offboard_samples if takeoff_done_time <= t <= recover_end_time]
    armed_ok = bool(trial_state) and all(s.armed for s in trial_state)
    # ACTIVE, not merely not-FAULT: a fall back to IDLE/STREAMING is offboard lost.
    offboard_ok = bool(trial_offboard) and all(
        s.state == OffboardStatus.STATE_ACTIVE for s in trial_offboard)
    record('%s: 5. stayed armed and offboard through the whole trial' % name,
           armed_ok and offboard_ok,
           'armed samples %d (all armed=%s), offboard samples %d (all ACTIVE=%s), '
           'window takeoff-done to recover-end'
           % (len(trial_state), armed_ok, len(trial_offboard), offboard_ok))

    all_command_times = [t for t, _ in node.commands]
    min_rate = min_windowed_rate(all_command_times, RATE_WINDOW_SEC)
    record('%s: 6. setpoint rate never fell below the PX4 offboard floor' % name,
           min_rate == min_rate and min_rate >= OFFBOARD_STREAM_FLOOR_HZ,
           'min rate in any %.0fs window: %.2f Hz (floor %.1f)'
           % (RATE_WINDOW_SEC, min_rate, OFFBOARD_STREAM_FLOOR_HZ))

    node.recording = False

    land = Land.Goal()
    land.use_precision_landing = False
    handle = node.send_goal(node.land_client, land, '%s land' % name)
    result = node.wait_result(handle, 120.0, '%s land' % name)
    node.wait_until(lambda: node.state is not None and not node.state.armed, 60.0, '%s disarm' % name)
    record('%s: 7. land disarmed the aircraft' % name,
           result.status == GoalStatus.STATUS_SUCCEEDED, 'status=%d' % result.status)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    args = parser.parse_args()

    rclpy.init()
    node = RecoverGate(args.uav_id)
    checks = []
    failure = None

    def record(name, passed, detail):
        checks.append((name, bool(passed), detail))

    try:
        node.wait_until(lambda: node.state is not None and node.state.connected, 60.0, 'autopilot')
        node.wait_until(lambda: node.position_now is not None, 60.0, 'odometry_fused')

        check_type_land_rejected(node, record)

        trials = [
            ('HOLD', Recover.Goal.TYPE_HOLD),
            ('CLIMB_TO_SAFE_ALTITUDE', Recover.Goal.TYPE_CLIMB_TO_SAFE_ALTITUDE),
            ('RETURN_HOME', Recover.Goal.TYPE_RETURN_HOME),
        ]
        for name, recovery_type in trials:
            home_xy = node.position_now[:2]      # this trial's own takeoff point
            run_trial(node, record, name, recovery_type, home_xy)
    except Failure as exc:
        failure = str(exc)
    except KeyboardInterrupt:
        failure = 'interrupted'

    print()
    print('=== G-N6 recover gate (uav0_nav) ===')
    for name, ok, detail in checks:
        print('  [%s] %-62s %s' % ('PASS' if ok else 'FAIL', name, detail))

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
