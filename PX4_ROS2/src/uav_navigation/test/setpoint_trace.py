#!/usr/bin/env python3
"""Trace what the navigator commanded against what the aircraft did. Run via trace_navigator_setpoints.sh."""

import argparse
import math
import statistics
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from uav_interfaces.action import Takeoff
from uav_interfaces.msg import ControlCommand, OffboardStatus, VehicleState

WALL_CLOCK_SAFETY_FACTOR = 6.0


class Failure(Exception):
    """Raised when the flight cannot be started; the run reports no trace."""


class SetpointTrace(Node):
    def __init__(self, uav_id):
        super().__init__(
            'setpoint_trace',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        prefix = '/uav/' + uav_id
        self.uav_id = uav_id
        self.samples = []           # (t, commanded_z, fused_z, raw_z, lead_z, fused_xy)
        self.commands = 0
        self.first_command = None
        self.actual = None
        self.raw = None
        self.offboard = None
        self.state = None
        self.start = None

        self.create_subscription(
            ControlCommand, prefix + '/control/command_selected', self._on_command, 50)
        self.create_subscription(
            Odometry, prefix + '/state/odometry_fused', self._on_odometry, 20)
        # PX4 reads the setpoint in the EKF2 frame, which is what odometry_raw reports.
        self.create_subscription(
            Odometry, prefix + '/state/odometry_raw', self._on_raw, 20)
        self.create_subscription(
            OffboardStatus, prefix + '/backend/offboard_status', self._on_offboard, 10)
        self.create_subscription(
            VehicleState, prefix + '/state/vehicle', self._on_state, 10)
        self.takeoff_client = ActionClient(self, Takeoff, prefix + '/planning/takeoff')

    def _on_command(self, message):
        self.commands += 1
        if self.first_command is None:
            self.first_command = message
        if self.actual is None or self.start is None:
            return
        self.samples.append((
            self.seconds() - self.start,
            message.position.z,
            self.actual.z,
            self.raw.z if self.raw is not None else float('nan'),
            message.position.z - self.actual.z,
            math.hypot(self.actual.x, self.actual.y),
        ))

    def _on_odometry(self, message):
        self.actual = message.pose.pose.position

    def _on_raw(self, message):
        self.raw = message.pose.pose.position

    def _on_offboard(self, message):
        self.offboard = message

    def _on_state(self, message):
        self.state = message

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin(self, seconds):
        start = self.seconds()
        wall_deadline = time.time() + seconds * WALL_CLOCK_SAFETY_FACTOR
        while rclpy.ok() and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.seconds() - start >= seconds:
                return
        raise Failure('sim clock stalled after %.1f s of %.1f s' % (self.seconds() - start, seconds))

    def wait_until(self, predicate, timeout, description):
        wall_deadline = time.time() + timeout * WALL_CLOCK_SAFETY_FACTOR
        while rclpy.ok() and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise Failure('timed out waiting for %s' % description)

    def await_server(self, client, timeout, description):
        """Spinning wait: a blocking wait_for_server never refreshes the graph cache."""
        wall_deadline = time.time() + timeout
        while rclpy.ok() and time.time() < wall_deadline:
            if client.server_is_ready():
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise Failure('%s never appeared' % description)


def describe_command(command):
    def finite(value):
        return 'nan' if math.isnan(value) else '%.3f' % value

    return ('control_mode=%d source=%d frame=%s\n'
            '    position   (%s %s %s)\n'
            '    velocity   (%s %s %s)\n'
            '    accel      (%s %s %s)\n'
            '    yaw %s  yaw_rate %s  thrust %s'
            % (command.control_mode, command.source, command.header.frame_id,
               finite(command.position.x), finite(command.position.y), finite(command.position.z),
               finite(command.velocity.x), finite(command.velocity.y), finite(command.velocity.z),
               finite(command.acceleration.x), finite(command.acceleration.y),
               finite(command.acceleration.z),
               finite(command.yaw), finite(command.yaw_rate), finite(command.thrust)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--altitude', type=float, default=2.5)
    parser.add_argument('--seconds', type=float, default=45.0)
    args = parser.parse_args()

    rclpy.init()
    node = SetpointTrace(args.uav_id)
    try:
        node.wait_until(lambda: node.state is not None and node.state.connected, 60.0, 'autopilot')
        node.wait_until(lambda: node.actual is not None, 30.0, 'fused odometry')
        node.await_server(node.takeoff_client, 60.0, 'takeoff action server')

        origin_z = node.actual.z
        node.start = node.seconds()
        goal = Takeoff.Goal()
        goal.target_altitude = args.altitude
        future = node.takeoff_client.send_goal_async(goal)
        node.wait_until(lambda: future.done(), 20.0, 'takeoff goal response')
        handle = future.result()
        if not handle.accepted:
            raise Failure('takeoff goal was rejected')
        result_future = handle.get_result_async()
        node.spin(args.seconds)
    except Failure as exc:
        print('TRACE FAILED: %s' % exc)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print()
    print('=== navigator setpoint trace, takeoff %.2f m ===' % args.altitude)
    print('  commands received %d, samples %d' % (node.commands, len(node.samples)))
    if node.first_command is not None:
        print('  first command published:')
        print('    ' + describe_command(node.first_command))
    if not node.samples:
        print('  RESULT: no samples; is the navigator publishing?')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    target_z = origin_z + args.altitude
    print('  origin z %.3f -> target z %.3f' % (origin_z, target_z))
    print()
    print('    t(s)  commanded_z   fused_z     raw_z   lead(m)  fused-raw   fused_xy')
    step = max(1, len(node.samples) // 25)
    for sample in node.samples[::step]:
        elapsed, commanded_z, fused_z, raw_z, lead, fused_xy = sample
        print('  %6.1f      %7.3f   %7.3f   %7.3f   %7.3f    %7.3f    %7.3f'
              % (elapsed, commanded_z, fused_z, raw_z, lead, fused_z - raw_z, fused_xy))

    commanded = [sample[1] for sample in node.samples]
    fused = [sample[2] for sample in node.samples]
    raw = [sample[3] for sample in node.samples if not math.isnan(sample[3])]
    lead = [sample[4] for sample in node.samples]
    offsets = [sample[2] - sample[3] for sample in node.samples if not math.isnan(sample[3])]
    tail = node.samples[-max(1, len(node.samples) // 5):]

    print()
    print('  commanded z: max %.3f, final %.3f  (target %.3f)'
          % (max(commanded), commanded[-1], target_z))
    print('  fused z:     max %.3f, final %.3f' % (max(fused), fused[-1]))
    if raw:
        print('  raw z:       max %.3f, final %.3f  (the frame PX4 flies in)'
              % (max(raw), raw[-1]))
        print('  fused - raw: first %.3f, median %.3f, last %.3f'
              % (offsets[0], statistics.median(offsets), offsets[-1]))
    print('  lead (commanded - fused): median %.3f, max %.3f  (leash %.2f)'
          % (statistics.median(lead), max(lead), 1.0))
    print('  last fifth of run: commanded %.3f -> %.3f, fused %.3f -> %.3f'
          % (tail[0][1], tail[-1][1], tail[0][2], tail[-1][2]))

    print()
    print('  READING THE TRACE')
    clamped = statistics.median(lead[len(lead) // 2:])
    if max(commanded) < target_z - 0.15 and clamped > 0.85:
        print('    commanded z stalled while sitting ~1 m above the aircraft: the LEASH '
              'is clamping. Ask why the aircraft stopped closing that last metre.')
    elif max(commanded) < target_z - 0.15:
        print('    commanded z never reached the target and the leash was not saturated: '
              'the carrot step itself is the suspect')
    elif max(fused) < target_z - 0.30:
        print('    commanded z reached the target but the aircraft did not follow: '
              'the setpoint contract with the backend is the suspect')
    else:
        print('    both reached the target in this run')
    if raw and abs(offsets[-1] - offsets[0]) > 0.30:
        print('    fused-raw offset MOVED by %.3f m during the run: the two vertical '
              'datums are not anchored, which is the frame-contract fault'
              % (offsets[-1] - offsets[0]))
    elif raw and abs(statistics.median(offsets)) > 0.30:
        print('    fused-raw offset is a steady %.3f m: the navigator and PX4 disagree '
              'about z by a constant' % statistics.median(offsets))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
