#!/usr/bin/env python3
"""V1 (P8.6 verifier brief): boot-time-only proof that B2 (battery NaN
default, not 0.0) and B5 (armed && OFFBOARD && command_fresh gates the 4
LATCHING codes) hold ON THE WIRE. Stack is up, DISARMED, no goal ever sent.

Two parts:
  A) 30s clean-boot window -- four NEGATIVE assertions must hold the whole
     time: no BATTERY_* violation edge, safety/state.recommended_action
     stays in {none, report}, /uav/<id>/diagnostics/control_authority never
     shows latch_active=true with latch_level=SAFETY, and battery_remaining
     is either NaN or a plausible [0,1] fraction (recorded either way).
  B) R27-3 positive counter-check, still disarmed: inject OBSTACLE_TOO_CLOSE
     directly on /world/obstacle_map_local for a few seconds. This is the
     bite-proof for the negative assertions above -- OBSTACLE_TOO_CLOSE MUST
     edge into the violations stream (REPORT layer alive, probe can read
     it), but recommended_action/latch must still never engage HOLD,
     because action_gate == armed && OFFBOARD && command_fresh is false
     while disarmed (B5).

R1/R20/R25: only uav_interfaces + diagnostic_msgs types, never px4_msgs.
Not isolated to a domain on purpose (must see the real safety stack).

Usage: python3 boot_no_latch_gate.py --uav-id uav0
Exit 0 = every assertion held with evidence; exit 1 otherwise (reason
printed). FAILED TO MEASURE is reported as its own outcome, never folded
into PASS (R27-1).
"""
import argparse
import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from std_msgs.msg import Header
from uav_interfaces.msg import Obstacle, ObstacleArray, SafetyState, VehicleHealth, VehicleState

CLEAN_WINDOW_SEC = 30.0
MEASURE_CHECK_SEC = 5.0        # R27-1: every topic must show >=1 msg within this window

INJECT_DISTANCE_M = 0.18       # well under min_obstacle_distance_m (0.35)
INJECT_HZ = 10.0                # matches world_model_node's real cadence
INJECT_DURATION_SEC = 3.0

BATTERY_VIOLATION_CODES = {'BATTERY_WARN', 'BATTERY_CRITICAL', 'BATTERY_PX4_NO_ACTION'}
ALLOWED_ACTIONS = {'none', 'report'}

REQUIRED_TOPICS = (
    'safety/state', 'safety/violations', 'diagnostics/control_authority', 'state/health_px4',
)
# safety/violations is EDGE-triggered (publishViolationEdges only fires on a
# status CHANGE, no transient_local) -- a subscriber that attaches after the
# node has already settled can legitimately see 0 messages for a long time
# with the channel perfectly alive. R27-1's "measure before object" duty for
# THIS one topic is discharged by R27-3 instead: the OBSTACLE_TOO_CLOSE edge
# injected in phase B is the proof the channel and our parsing are wired
# correctly. The other three topics publish periodically and must show up
# in the short boot window like normal.
PERIODIC_TOPICS = tuple(t for t in REQUIRED_TOPICS if t != 'safety/violations')


class BootNoLatchGate(Node):
    def __init__(self, uav_id, use_sim_time):
        from rclpy.parameter import Parameter
        super().__init__(
            'boot_no_latch_gate',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])
        prefix = '/uav/' + uav_id
        self.uav_id = uav_id

        self.seen = {t: 0 for t in REQUIRED_TOPICS}

        self.safety_states = []      # (wall, recommended_action, level)
        self.violation_edges = []    # (wall, code, edge)
        self.authority_diag = []     # (wall, latch_active, latch_level)
        self.battery_samples = []    # (wall, battery_remaining)
        self.vehicle_states = []     # (wall, armed, flight_mode)

        self.create_subscription(
            SafetyState, prefix + '/safety/state', self._on_safety_state, 10)
        self.create_subscription(
            DiagnosticArray, prefix + '/safety/violations', self._on_violations, 20)
        self.create_subscription(
            DiagnosticArray, prefix + '/diagnostics/control_authority', self._on_authority_diag, 10)
        self.create_subscription(
            VehicleHealth, prefix + '/state/health_px4', self._on_health, 10)
        self.create_subscription(
            VehicleState, prefix + '/state/vehicle', self._on_vehicle_state, 10)

        self.obstacle_publisher = self.create_publisher(
            ObstacleArray, prefix + '/world/obstacle_map_local', 10)

    # ------------------------------------------------------------ callbacks

    def _on_safety_state(self, msg):
        self.seen['safety/state'] += 1
        self.safety_states.append((time.time(), msg.recommended_action, msg.level))
        self._log('safety/state action=%s level=%d' % (msg.recommended_action, msg.level))

    def _on_violations(self, msg):
        self.seen['safety/violations'] += 1
        for status in msg.status:
            if not status.name.startswith('safety: '):
                continue
            code = status.name[len('safety: '):]
            edge = status.message.split(':', 1)[0]
            self.violation_edges.append((time.time(), code, edge))
            self._log('violation edge: %s -- %s' % (code, edge))

    def _on_authority_diag(self, msg):
        self.seen['diagnostics/control_authority'] += 1
        for status in msg.status:
            if 'arbitration' not in status.name:
                continue
            latch_active = None
            latch_level = None
            for kv in status.values:
                if kv.key == 'latch_active':
                    latch_active = (kv.value == 'true')
                elif kv.key == 'latch_level':
                    latch_level = kv.value
            self.authority_diag.append((time.time(), latch_active, latch_level))
            if latch_active:
                self._log('diagnostics/control_authority latch_active=true latch_level=%s' % latch_level)

    def _on_health(self, msg):
        self.seen['state/health_px4'] += 1
        self.battery_samples.append((time.time(), msg.battery_remaining))

    def _on_vehicle_state(self, msg):
        self.vehicle_states.append((time.time(), msg.armed, msg.flight_mode))

    def _log(self, text):
        print('  [t=%.3f] %s' % (time.time(), text))
        sys.stdout.flush()

    # -------------------------------------------------------------- helpers

    def spin(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_and_inject(self, seconds):
        end = time.time() + seconds
        period = 1.0 / INJECT_HZ
        next_publish = time.time()
        while rclpy.ok() and time.time() < end:
            now = time.time()
            if now >= next_publish:
                self._publish_obstacle()
                next_publish = now + period
            rclpy.spin_once(self, timeout_sec=0.02)

    def _publish_obstacle(self):
        msg = ObstacleArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.uav_id = self.uav_id
        msg.sensing_range = 5.0
        obstacle = Obstacle()
        obstacle.obstacle_id = 1
        obstacle.shape = Obstacle.SHAPE_SPHERE
        obstacle.center = Point(x=1.0, y=0.0, z=0.0)
        obstacle.size = Vector3(x=0.3, y=0.3, z=0.3)
        obstacle.distance = float(INJECT_DISTANCE_M)
        obstacle.confidence = 1.0
        obstacle.position_uncertainty = 0.05
        msg.obstacles = [obstacle]
        self.obstacle_publisher.publish(msg)


def check_measurement(node):
    """R27-1: every PERIODIC topic must show >=1 message within a short
    window -- missing any of them makes the whole run FAILED TO MEASURE,
    never silently PASS. safety/violations is edge-triggered (see
    PERIODIC_TOPICS comment) and is checked separately, at the end, after
    the R27-3 injection has had a chance to produce an edge."""
    node.spin(MEASURE_CHECK_SEC)
    missing = [t for t in PERIODIC_TOPICS if node.seen[t] == 0]
    return missing


def analyze_window(node, window_start, window_label):
    """Shared analysis for both phase A (clean boot) and phase B (after
    injection) -- same four assertions, applied to whatever fell in
    [window_start, now)."""
    battery_bad_edges = [
        (w, code, edge) for w, code, edge in node.violation_edges
        if w >= window_start and code in BATTERY_VIOLATION_CODES and edge == 'entered'
    ]
    bad_actions = [
        (w, action) for w, action, _lvl in node.safety_states
        if w >= window_start and action not in ALLOWED_ACTIONS
    ]
    safety_latches = [
        (w, active, level) for w, active, level in node.authority_diag
        if w >= window_start and active and level == 'SAFETY'
    ]
    battery_in_window = [(w, v) for w, v in node.battery_samples if w >= window_start]
    battery_out_of_range = [
        (w, v) for w, v in battery_in_window
        if not (math.isnan(v) or (0.0 <= v <= 1.0))
    ]
    return {
        'label': window_label,
        'battery_bad_edges': battery_bad_edges,
        'bad_actions': bad_actions,
        'safety_latches': safety_latches,
        'battery_samples': battery_in_window,
        'battery_out_of_range': battery_out_of_range,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--no-sim-time', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = BootNoLatchGate(args.uav_id, use_sim_time=not args.no_sim_time)

    print('=== V1 boot-no-latch: R27-1 measurement check (%.0fs) ===' % MEASURE_CHECK_SEC)
    missing = check_measurement(node)
    if missing:
        print('FAILED TO MEASURE: no message on %s within %.0fs' % (missing, MEASURE_CHECK_SEC))
        rclpy.shutdown()
        return 2

    for t in REQUIRED_TOPICS:
        print('  %-32s: %d msg seen so far' % (t, node.seen[t]))

    print()
    print('=== phase A: %.0fs clean-boot window, disarmed, no goal ===' % CLEAN_WINDOW_SEC)
    phase_a_start = time.time()
    node.spin(CLEAN_WINDOW_SEC)
    phase_a = analyze_window(node, phase_a_start, 'phase A (clean boot)')

    armed_during_a = any(w >= phase_a_start and armed for w, armed, _m in node.vehicle_states)
    print('vehicle_state.armed observed true during phase A: %s (must be False)' % armed_during_a)

    print()
    print('=== phase B (R27-3 positive counter-check): inject OBSTACLE_TOO_CLOSE, still disarmed ===')
    phase_b_start = time.time()
    print('injecting distance=%.2f m @ %.1f Hz for %.1fs' % (INJECT_DISTANCE_M, INJECT_HZ, INJECT_DURATION_SEC))
    node.spin_and_inject(INJECT_DURATION_SEC)
    node.spin(2.0)  # settle so any edge has time to arrive
    phase_b = analyze_window(node, phase_a_start, 'phase B (cumulative, injection included)')

    saw_obstacle_entered = any(
        code == 'OBSTACLE_TOO_CLOSE' and edge == 'entered'
        for w, code, edge in node.violation_edges if w >= phase_b_start)

    armed_during_b = any(w >= phase_b_start and armed for w, armed, _m in node.vehicle_states)

    if node.seen['safety/violations'] == 0:
        print()
        print('FAILED TO MEASURE: 0 messages ever seen on safety/violations, even after the '
              'R27-3 injection -- the channel/subscription cannot be trusted (see PERIODIC_TOPICS '
              'comment for why this is checked here, not in the boot-time R27-1 window)')
        rclpy.shutdown()
        return 2

    print()
    print('=== V1 BOOT-NO-LATCH REPORT (%s) ===' % args.uav_id)
    for phase in (phase_a, phase_b):
        print('-- %s --' % phase['label'])
        print('  BATTERY_* violation edges entered : %d %s' % (
            len(phase['battery_bad_edges']), phase['battery_bad_edges']))
        print('  recommended_action outside {none,report}: %d %s' % (
            len(phase['bad_actions']), phase['bad_actions']))
        print('  latch_active=true & latch_level=SAFETY   : %d %s' % (
            len(phase['safety_latches']), phase['safety_latches']))
        battery_values = [v for _w, v in phase['battery_samples']]
        print('  battery_remaining samples (n=%d): min=%s max=%s nan_count=%d' % (
            len(battery_values),
            min([v for v in battery_values if not math.isnan(v)], default=float('nan')),
            max([v for v in battery_values if not math.isnan(v)], default=float('nan')),
            sum(1 for v in battery_values if math.isnan(v))))
        print('  battery_remaining out of [0,1]/NaN       : %d %s' % (
            len(phase['battery_out_of_range']), phase['battery_out_of_range']))

    print()
    print('OBSTACLE_TOO_CLOSE entered during injection (REPORT layer alive): %s' % saw_obstacle_entered)
    print('vehicle_state.armed observed true anywhere in the run: %s (must be False throughout)'
          % (armed_during_a or armed_during_b))

    assertion_a_no_battery = len(phase_a['battery_bad_edges']) == 0
    assertion_a_action = len(phase_a['bad_actions']) == 0
    assertion_a_no_latch = len(phase_a['safety_latches']) == 0
    assertion_a_battery_range = len(phase_a['battery_out_of_range']) == 0
    assertion_b_no_battery = len(phase_b['battery_bad_edges']) == 0
    assertion_b_action = len(phase_b['bad_actions']) == 0
    assertion_b_no_latch = len(phase_b['safety_latches']) == 0
    assertion_b_bite = saw_obstacle_entered  # R27-3: negative assertions must have a counter-proof
    assertion_never_armed = not (armed_during_a or armed_during_b)

    passed = (
        assertion_a_no_battery and assertion_a_action and assertion_a_no_latch and
        assertion_a_battery_range and assertion_b_no_battery and assertion_b_action and
        assertion_b_no_latch and assertion_b_bite and assertion_never_armed)

    print()
    print('sub-verdicts: a_no_battery=%s a_action=%s a_no_latch=%s a_battery_range=%s '
          'b_no_battery=%s b_action=%s b_no_latch=%s b_bite(R27-3)=%s never_armed=%s'
          % (assertion_a_no_battery, assertion_a_action, assertion_a_no_latch,
             assertion_a_battery_range, assertion_b_no_battery, assertion_b_action,
             assertion_b_no_latch, assertion_b_bite, assertion_never_armed))
    print('RESULT: %s' % ('PASS' if passed else 'FAIL'))
    sys.stdout.flush()

    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
