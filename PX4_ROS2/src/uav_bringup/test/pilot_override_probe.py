#!/usr/bin/env python3
"""P11.6 -- prove the stack stands down when a pilot takes the aircraft, and only then.

WHAT IT SIMULATES. A real RC override puts PX4 into a stick-flown mode (POSCTL/MANUAL/
ALTCTL/ACRO/STAB). There is no radio here, so the mode change is commanded through our
own /backend/set_mode service -- FLIGHT_MODE_POSITION maps to POSCTL. That reproduces
the STATE the aircraft ends up in, which is what the stack reacts to; it does not
reproduce the radio link, and P11.5 remains the gate for that.

WHY IT NEEDED FIXING. Before P11.6, losing offboard into POSCTL was classified as
"autopilot left offboard mode unexpectedly" -> STATE_FAULT, and readyToEngage() was
free to fire again the moment a setpoint stream reappeared, because POSCTL is not in
autopilotOwnsTheFlight()'s list -- deliberately, since every flight engages offboard out
of a POSCTL ground state. So the stack would take the aircraft back from the pilot.

TWO ROUNDS, AND THE SECOND IS THE CONTROL.

  takeover  set POSCTL. Expect: no FAULT, detail names the pilot, and the aircraft STAYS
            in POSCTL -- the stack does not re-engage.
  control   set HOLD (AUTO_LOITER) instead. Neither autopilotOwnsTheFlight() nor
            pilotIsFlying() covers it, so the stack SHOULD re-engage offboard. If it
            does, the probe is demonstrably able to observe a re-grab -- which is what
            makes the takeover round's "it did not re-grab" mean something rather than
            being a probe that cannot see.

Usage: python3 pilot_override_probe.py [--uav-id uav0] [--round takeover|control]
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from uav_interfaces.msg import ControlAuthority, ControlCommand, OffboardStatus, VehicleState
from uav_interfaces.srv import Arm, SetFlightMode

STREAM_HZ = 20.0
ENGAGE_TIMEOUT = 90.0
OBSERVE_SEC = 25.0


class Failure(Exception):
    pass


class Probe(Node):
    def __init__(self, uav_id):
        super().__init__('pilot_override_probe', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        prefix = '/uav/' + uav_id
        self.uav_id = uav_id
        self.offboard = None
        self.state = None
        self.streaming = False

        self.command_pub = self.create_publisher(
            ControlCommand, prefix + '/control/cmd_test', 10)
        self.create_subscription(
            OffboardStatus, prefix + '/backend/offboard_status', self._on_offboard, 10)
        self.create_subscription(VehicleState, prefix + '/state/vehicle', self._on_state, 10)
        self.arm_client = self.create_client(Arm, prefix + '/backend/arm')
        self.mode_client = self.create_client(SetFlightMode, prefix + '/backend/set_mode')
        self.create_timer(1.0 / STREAM_HZ, self._stream)

    def _on_offboard(self, msg):
        self.offboard = msg

    def _on_state(self, msg):
        self.state = msg

    def _stream(self):
        if not self.streaming:
            return
        cmd = ControlCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'odom'
        cmd.uav_id = self.uav_id
        cmd.control_mode = ControlCommand.MODE_POSITION
        cmd.position.x, cmd.position.y, cmd.position.z = 0.0, 0.0, -2.0
        cmd.yaw = 0.0
        cmd.source = ControlAuthority.SOURCE_TEST
        self.command_pub.publish(cmd)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, predicate, timeout, what):
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise Failure('timed out waiting for %s' % what)

    def set_mode(self, mode, label):
        if not self.mode_client.wait_for_service(timeout_sec=20.0):
            raise Failure('/backend/set_mode never appeared')
        req = SetFlightMode.Request()
        req.mode = mode
        future = self.mode_client.call_async(req)
        end = time.time() + 20.0
        while rclpy.ok() and not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise Failure('set_mode(%s) never returned' % label)
        res = future.result()
        print('  set_mode(%s): success=%s msg=%r' % (label, res.success, res.message))
        return res.success


def run(uav_id, which):
    probe = Probe(uav_id)
    try:
        probe.wait_until(lambda: probe.offboard is not None, 90.0, 'offboard_status')
        probe.wait_until(lambda: probe.state is not None, 30.0, 'vehicle state')

        # Arm first so the pilot-override latch can distinguish "flying" from the ground
        # state -- on the ground PX4 sits in POSCTL on every single flight, and latching
        # there would stop the stack ever engaging.
        if probe.arm_client.wait_for_service(timeout_sec=20.0):
            fut = probe.arm_client.call_async(Arm.Request())
            end = time.time() + 20.0
            while rclpy.ok() and not fut.done() and time.time() < end:
                rclpy.spin_once(probe, timeout_sec=0.05)
            print('  arm: %s' % (fut.result().success if fut.done() else 'no reply'))

        probe.streaming = True
        probe.wait_until(
            lambda: probe.offboard.state == OffboardStatus.STATE_ACTIVE,
            ENGAGE_TIMEOUT, 'offboard ACTIVE')
        print('  offboard ACTIVE, detail=%r' % probe.offboard.detail)

        if which == 'takeover':
            mode, label, want_mode = VehicleState.FLIGHT_MODE_POSITION, 'POSITION (pilot)', \
                VehicleState.FLIGHT_MODE_POSITION
        else:
            mode, label, want_mode = VehicleState.FLIGHT_MODE_HOLD, 'HOLD (control round)', \
                VehicleState.FLIGHT_MODE_OFFBOARD
        probe.set_mode(mode, label)

        # Keep streaming: that is the whole point. A stack that stands down only because
        # the setpoints stopped has not stood down at all.
        probe.spin_for(OBSERVE_SEC)

        st = probe.offboard
        print('  after %.0fs: offboard state=%d detail=%r flight_mode=%d'
              % (OBSERVE_SEC, st.state, st.detail, probe.state.flight_mode))

        problems = []
        if which == 'takeover':
            if st.state == OffboardStatus.STATE_FAULT:
                problems.append('classified a deliberate pilot takeover as a FAULT')
            if 'pilot' not in st.detail.lower():
                problems.append('detail %r never names the pilot' % st.detail)
            if probe.state.flight_mode != want_mode:
                problems.append(
                    'aircraft left POSCTL (flight_mode=%d) -- the stack took it back '
                    'from the pilot' % probe.state.flight_mode)
        else:
            if probe.state.flight_mode != want_mode:
                problems.append(
                    'control round: the stack did NOT re-engage offboard (flight_mode=%d). '
                    'Then the takeover round proves nothing -- this probe cannot see a '
                    're-grab at all.' % probe.state.flight_mode)

        if problems:
            for p in problems:
                print('  FAIL: %s' % p)
            return 1
        print('  round %r: as expected' % which)
        return 0
    finally:
        probe.streaming = False
        probe.destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uav-id', default='uav0')
    ap.add_argument('--round', default='takeover', choices=['takeover', 'control'])
    args = ap.parse_args()

    rclpy.init()
    try:
        return run(args.uav_id, args.round)
    except Failure as exc:
        print('  FAILED TO MEASURE: %s' % exc)
        return 2
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
