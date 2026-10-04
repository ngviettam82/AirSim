#!/usr/bin/env python3
"""S13 probe: fly the configuration the aircraft will actually fly.

THE GAP THIS CLOSES (P12.4, 2026-08-25). sim.launch.py has always set
require_obstacle_feed FALSE and real.launch.py sets it TRUE (P6 Decision 4). A grep over
scripts/ found no gate that has ever run with true, so the branches the aircraft takes had
never executed anywhere -- the sharpest being local_planner_node.cpp, where a Hold is
softened into a Clear ONLY when the flag is false:

    if (result.advice == Advice::Hold && !result.map_fresh && !require_obstacle_feed_) {
      RCLCPP_WARN_THROTTLE(..., "flying unguarded: %s (require_obstacle_feed is false)", ...);
      result.advice = Advice::Clear;
      result.checked_horizon_m = 0.0;
      result.reason = "map not required, nothing checked: " + result.reason;
    }

On the aircraft that softening does not happen and the Hold stands. Until now, the Hold
had never stood anywhere.

HOW IT KNOWS THE HOLD REALLY HAPPENED. The obvious version of this gate -- fly with the
flag on and see that nothing crashes -- proves nothing: an aircraft that never met a stale
map would pass it. The discriminator is in the message itself, and it is exact, because
the softening leaves a signature it cannot fake:

    flag false, map stale  ->  advice=CLEAR, checked_horizon_m == 0.0,
                               reason starts with "map not required, nothing checked"
    flag true,  map stale  ->  advice=HOLD,  the advisor's own reason

So the gate does BOTH runs and requires them to differ in that exact way. That is the
positive control (R27-3) and it costs one extra flight: without the false arm, a green
true arm could just mean the map never went stale, and nobody would know.

Usage: s13_obstacle_feed_gate.py --uav-id uav0 --require-feed true|false --seconds N
       Prints EVIDENCE lines and exits 0 when the arm behaved as that arm should.
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from uav_interfaces.action import GotoPose, Land, Takeoff
from uav_interfaces.srv import Arm
from uav_interfaces.msg import AvoidanceAdvice

ADVICE_NAMES = {
    AvoidanceAdvice.ADVICE_CLEAR: 'CLEAR',
    AvoidanceAdvice.ADVICE_ESCAPE: 'ESCAPE',
    AvoidanceAdvice.ADVICE_HOLD: 'HOLD',
}

# The exact prefix local_planner_node writes when it softens a Hold. Matching on it rather
# than on advice alone is what makes the two arms distinguishable even if the advisor's own
# reason changes.
SOFTENED_PREFIX = 'map not required, nothing checked'

# local_planner_node returns this BEFORE consulting the advisor, so a sample
# carrying it says nothing about how the flag is treated.
NO_PLAN_PREFIX = 'no active plan to check'


class AdviceProbe(Node):
    def __init__(self, uav_id):
        super().__init__('s13_obstacle_feed_probe')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        self.samples = 0
        self.by_advice = {}
        self.softened = 0
        self.stale_samples = 0
        self.stale_advice = {}
        self.idle_samples = 0
        self.reasons = []
        prefix = '/uav/%s' % uav_id
        self.create_subscription(
            AvoidanceAdvice, prefix + '/planning/avoidance', self._on_advice,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.takeoff_client = ActionClient(self, Takeoff, prefix + '/planning/takeoff')
        self.goto_client = ActionClient(self, GotoPose, prefix + '/planning/goto_pose')
        self.land_client = ActionClient(self, Land, prefix + '/planning/land')
        self.arm_client = self.create_client(Arm, prefix + '/backend/arm')

    def _await(self, future, timeout_sec):
        """Wait on WALL time, not sim time.

        This probe runs with use_sim_time, so get_clock().now() is 0 until the first
        /clock message is processed. Computing a deadline from it gives end = 0 + 20, and
        the first real /clock -- carrying a sim time already tens of seconds in, because
        the simulator started long before the probe did -- lands past that deadline at
        once. The result was "no reply within 20 s" printed after no waiting at all, and
        it looked like the arm service hanging. It was intermittent for the obvious
        reason: whether any /clock had arrived before the deadline was computed.

        A probe's own patience is a wall-clock question. R21 is about not letting wall
        time stand in for a measured property of the product -- it does not ask a client
        to time its own patience on a clock that starts at zero.
        """
        end = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > end:
                return None
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result()

    def run_action(self, client, goal, label, timeout_sec=90.0):
        """Send one goal and wait it out. Returns True when it finished at all."""
        if not client.wait_for_server(timeout_sec=30.0):
            print('[ EVIDENCE ] %s: action server never appeared' % label)
            return False
        handle = self._await(client.send_goal_async(goal), 20.0)
        if handle is None:
            # Not the same thing as a refusal, and the first version of this probe said it
            # was -- reporting "goal rejected" for a goal the navigator had accepted.
            print('[ EVIDENCE ] %s: no answer within 20 s (NOT a refusal)' % label)
            return False
        if not handle.accepted:
            print('[ EVIDENCE ] %s: goal refused by the server' % label)
            return False
        result = self._await(handle.get_result_async(), timeout_sec)
        print('[ EVIDENCE ] %s: finished=%s' % (label, result is not None))
        return result is not None

    def arm(self):
        """PX4 holds the aircraft down until armed; the navigator will stream at it anyway.

        The first run of this probe skipped this and spent the flight clamped at
        position_z 0.23 while the log filled with 'leash clamping the setpoint'.
        """
        if not self.arm_client.wait_for_service(timeout_sec=30.0):
            print('[ EVIDENCE ] arm: service never appeared')
            return False
        reply = self._await(self.arm_client.call_async(Arm.Request()), 20.0)
        if reply is None:
            print('[ EVIDENCE ] arm: no reply within 20 s (NOT a refusal)')
            return False
        print('[ EVIDENCE ] arm: success=%s message=%r'
              % (reply.success, getattr(reply, 'message', '')))
        return reply.success

    def fly(self):
        """The smallest flight that makes local_planner_node advise on something."""
        if not self.arm():
            return False
        takeoff = Takeoff.Goal()
        takeoff.target_altitude = 2.5
        if not self.run_action(self.takeoff_client, takeoff, 'takeoff'):
            return False
        goto = GotoPose.Goal()
        goto.target_pose = Pose()
        goto.target_pose.position.x = 5.0
        # Held at the takeoff altitude on purpose: a goal at z=0 is below min_altitude_m and
        # the navigator refuses it, which is what happened on the first run.
        goto.target_pose.position.z = 2.5
        goto.target_pose.orientation.w = 1.0
        goto.frame_id = 'odom'
        goto.acceptance_radius = 0.5
        goto.max_speed = 0.5
        flown = self.run_action(self.goto_client, goto, 'goto')
        self.run_action(self.land_client, Land.Goal(), 'land', timeout_sec=60.0)
        return flown

    def _on_advice(self, msg):
        self.samples += 1
        name = ADVICE_NAMES.get(msg.advice, str(msg.advice))
        self.by_advice[name] = self.by_advice.get(name, 0) + 1
        reason = (msg.reason or '').strip()
        if reason and reason not in self.reasons:
            self.reasons.append(reason)
        if reason.startswith(SOFTENED_PREFIX):
            self.softened += 1
        # Only a STALE map can distinguish the two arms: with a fresh map both flags give
        # the same answer, so counting every sample would drown the signal.
        if not msg.map_fresh and not reason.startswith(NO_PLAN_PREFIX):
            self.stale_samples += 1
            self.stale_advice[name] = self.stale_advice.get(name, 0) + 1
        elif not msg.map_fresh:
            self.idle_samples += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uav-id', default='uav0')
    ap.add_argument('--require-feed', required=True, choices=['true', 'false'])
    ap.add_argument('--seconds', type=float, default=45.0)
    args = ap.parse_args()
    required = args.require_feed == 'true'

    rclpy.init()
    probe = AdviceProbe(args.uav_id)
    flown = probe.fly()
    print('[ EVIDENCE ] flight completed=%s' % flown)
    # Keep listening briefly after landing: the last advice of a leg is as interesting as
    # the ones during it, and a short tail costs nothing.
    end = time.monotonic() + 5.0
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)

    print('[ EVIDENCE ] require_obstacle_feed=%s' % args.require_feed)
    print('[ EVIDENCE ] advice samples=%d  by advice=%s' % (probe.samples, probe.by_advice))
    print('[ EVIDENCE ] stale-map samples WITH a plan=%d  by advice=%s'
          % (probe.stale_samples, probe.stale_advice))
    print('[ EVIDENCE ] stale-map samples with NO plan=%d (not judged)' % probe.idle_samples)
    print('[ EVIDENCE ] softened-Hold samples=%d' % probe.softened)
    for reason in probe.reasons[:6]:
        print('[ EVIDENCE ] reason: %s' % reason)

    failures = []
    if not flown:
        # The first version printed this and passed anyway. Advice collected while the
        # aircraft sat on the ground says something about the planner, but it says nothing
        # about the configuration IN FLIGHT, which is the only claim S13 makes.
        failures.append('the flight did not complete, so nothing here describes the '
                        'configuration in flight. FAILED TO MEASURE')
    if probe.samples == 0:
        # Silence is not evidence of good behaviour (O3). The planner may not be running,
        # or the topic name may be wrong, and neither must read as "no problems seen".
        failures.append('no AvoidanceAdvice at all -- local_planner_node not running, '
                        'or the topic name is wrong. Not measured is not OK')
    if probe.samples and probe.stale_samples == 0:
        # Without a stale map the two arms are indistinguishable, so this run cannot
        # support any claim either way. Reported as unable-to-measure, never as a pass.
        failures.append('the obstacle map never went stale, so this run cannot tell the '
                        'two configurations apart. FAILED TO MEASURE')
    if probe.stale_samples and required:
        held = probe.stale_advice.get('HOLD', 0)
        if held == 0:
            failures.append('map went stale %d time(s) and the planner never held -- with '
                            'require_obstacle_feed the Hold must stand' % probe.stale_samples)
        if probe.softened:
            failures.append('the softening branch ran %d time(s) with the feed REQUIRED; '
                            'it must be unreachable in this arm' % probe.softened)
    if probe.stale_samples and not required:
        # The control arm. It has to show the softening, otherwise the true arm proves
        # nothing: both arms behaving identically would mean the flag changed nothing.
        if probe.softened == 0:
            failures.append('control arm did not soften any Hold, so the two arms are not '
                            'distinguishable and the true arm proves nothing')

    print()
    for f in failures:
        print('  FAIL  %s' % f)
    if failures:
        print('RESULT: S13 arm (require_obstacle_feed=%s) FAILED' % args.require_feed)
        return 1
    print('RESULT: S13 arm (require_obstacle_feed=%s) PASS' % args.require_feed)
    return 0


if __name__ == '__main__':
    sys.exit(main())
