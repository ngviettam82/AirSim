#!/usr/bin/env python3
"""P10.8b diagnostic (not part of the o3-* gate rounds): isolates whether
the DEPLOYED control_authority_manager_node (not just the arbiter's own
gtest harness) actually detects a /clock regression through the standard
use_sim_time mechanism, without any bag/replay QoS complications.

Drives /clock and control/cmd_mission directly at hand-picked sim-time
values: MISSION goes live, then times out to NONE (forward jump), then the
clock is REGRESSED behind MISSION's last_valid_arrival_ stamp -- if
ageSecOrInf() gets called for that comparison the way authority_arbiter.cpp
predicts, clock_regressions on /diagnostics/control_authority must go > 0.

Usage: o3_clock_regression_synth.py --uav-id uav0
Prints BEFORE=<n> AFTER=<n> and exits 0 if AFTER > BEFORE, else 1.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header
from uav_interfaces.msg import ControlAuthority, ControlCommand


def stamp(sec):
    t = TimeMsg()
    t.sec = int(sec)
    t.nanosec = int((sec - int(sec)) * 1e9)
    return t


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    args = parser.parse_args()
    prefix = '/uav/%s' % args.uav_id

    rclpy.init()
    node = Node('o3_clock_regression_synth')

    clock_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    clock_pub = node.create_publisher(Clock, '/clock', clock_qos)
    cmd_pub = node.create_publisher(ControlCommand, prefix + '/control/cmd_mission', 10)

    latest = {'clock_regressions': None, 'active_source': None}

    def on_diag(msg):
        for status in msg.status:
            if status.name != 'control_authority: clock regressions':
                continue
            for kv in status.values:
                if kv.key == 'clock_regressions':
                    try:
                        latest['clock_regressions'] = int(float(kv.value))
                    except ValueError:
                        pass

    def on_authority(msg):
        latest['active_source'] = msg.active_source

    node.create_subscription(DiagnosticArray, prefix + '/diagnostics/control_authority', on_diag, 10)
    node.create_subscription(
        ControlAuthority, prefix + '/control/authority', on_authority,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    def publish_clock(sec):
        msg = Clock()
        msg.clock = stamp(sec)
        clock_pub.publish(msg)

    # monitor_timer_ ticks at kMonitorPeriodSec=0.05s but a ROS sim-time
    # timer fires (at most) ONCE per NEW /clock message that crosses its
    # deadline -- it does NOT replay N missed periods for one big jump.
    # publishDiagnostics() only runs on the 20th tick (1Hz). A single big
    # jump therefore ticks onMonitorTick() once, nowhere near 20 -- this
    # helper walks sim time forward in many small steps so the tick counter
    # actually crosses a 20-tick boundary and a diagnostics sample comes out.
    def tick_many(start_sec, n=26, step=0.06):
        for i in range(n):
            publish_clock(start_sec + i * step)
            pump(0.05)

    # t=10: establish sim time, let discovery settle, and cross a 20-tick
    # boundary so the FIRST diagnostics baseline is actually observable.
    tick_many(0.0, n=26, step=10.0 / 25.0)

    # Publish a fresh, valid MISSION command AT sim time 10.0.
    cmd = ControlCommand()
    cmd.header = Header()
    cmd.header.stamp = stamp(10.0)
    cmd.header.frame_id = 'odom'
    cmd.uav_id = args.uav_id
    cmd.control_mode = ControlCommand.MODE_POSITION
    cmd.position.x, cmd.position.y, cmd.position.z = 0.0, 0.0, 1.0
    cmd.yaw = 0.0
    cmd.source = ControlAuthority.SOURCE_MISSION
    for _ in range(5):
        cmd_pub.publish(cmd)
        publish_clock(10.0)
        pump(0.1)

    pump(0.5)
    before = latest['clock_regressions']
    active_after_cmd = latest['active_source']
    print('after cmd: active_source=%s clock_regressions=%s' % (active_after_cmd, before))

    # Forward jump to 11.0 (>> source_timeout_sec 0.20 + release_dwell_sec
    # 0.20 = 0.40s past the command's own stamp) -- MISSION should decay to
    # NONE via onTick's release-dwell branch. /control/authority publishes a
    # heartbeat at 2Hz (every 10th tick) regardless of change, so this is
    # independently observable even without crossing a 20-tick boundary.
    for _ in range(20):
        publish_clock(11.0)
        pump(0.1)
    after_decay_source = latest['active_source']
    print('after forward jump to 11.0: active_source=%s clock_regressions=%s'
          % (after_decay_source, latest['clock_regressions']))

    # REGRESS: jump back to 5.0 (well behind last_valid_arrival_[MISSION]
    # =10.0), then walk FORWARD in small steps -- each step below 10.0 is
    # ANOTHER onTick() call with now < last_valid_arrival_[MISSION], so
    # clock_regressions_ should climb by more than 1; the walk also crosses
    # the next 20-tick boundary so a diagnostics sample actually comes out
    # (see tick_many's docstring -- a single big jump does not).
    tick_many(5.0, n=26, step=0.3)
    after = latest['clock_regressions']
    print('after regression to 5.0 + tick walk: active_source=%s clock_regressions=%s'
          % (latest['active_source'], after))

    node.destroy_node()
    rclpy.shutdown()

    print('BEFORE=%s AFTER=%s' % (before, after))
    if before is None or after is None:
        print('FAILED TO MEASURE: never received a diagnostics sample')
        sys.exit(2)
    if after > before:
        print('PASS: clock_regressions increased across the regression')
        sys.exit(0)
    print('FAIL: clock_regressions did NOT increase despite a real /clock regression')
    sys.exit(1)


if __name__ == '__main__':
    main()
