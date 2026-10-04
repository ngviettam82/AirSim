#!/usr/bin/env python3
"""G-A0 final criterion: prove PX4 under AirSim HIL will actually arm.

A healthy telemetry stream is not the same as an armable aircraft, so this
reads arming_state back rather than trusting the command's acknowledgement.
"""
import rclpy
from px4_msgs.msg import VehicleCommand, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

ARMING_STATE_ARMED = 2


class TryArm(Node):
    def __init__(self):
        super().__init__('ga0_try_arm')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.status = None
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self._on_status, qos)
        self.pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

    def _on_status(self, msg):
        self.status = msg

    def send_arm(self):
        cmd = VehicleCommand()
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = TryArm()
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.status is not None:
            break
    if node.status is None:
        print('FAIL: no vehicle_status in 5 s')
        return 1

    print('truoc khi arm : arming_state=%d nav_state=%d preflight_pass=%s'
          % (node.status.arming_state, node.status.nav_state,
             node.status.pre_flight_checks_pass))

    for _ in range(30):
        node.send_arm()
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.status.arming_state == ARMING_STATE_ARMED:
            break

    armed = node.status.arming_state == ARMING_STATE_ARMED
    print('sau khi arm   : arming_state=%d' % node.status.arming_state)
    print('RESULT: %s' % ('ARMED' if armed else 'KHONG ARM DUOC'))

    if armed:
        cmd_disarm = VehicleCommand()
        cmd_disarm.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        cmd_disarm.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd_disarm.param1 = 0.0
        cmd_disarm.target_system = 1
        cmd_disarm.target_component = 1
        cmd_disarm.source_system = 1
        cmd_disarm.source_component = 1
        cmd_disarm.from_external = True
        node.pub.publish(cmd_disarm)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        print('da gui disarm : arming_state=%d' % node.status.arming_state)

    node.destroy_node()
    rclpy.shutdown()
    return 0 if armed else 1


if __name__ == '__main__':
    raise SystemExit(main())
