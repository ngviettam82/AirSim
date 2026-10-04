#!/usr/bin/env python3
"""P10.8c diagnostic (not part of the o3-* gate rounds): isolates whether a
plain ROS2 sim-time timer (rclpy `create_timer` under `use_sim_time=True`,
the SAME mechanism `rclcpp::create_timer(this, get_clock(), ...)` uses --
control_authority_manager_node.cpp's monitor_timer_ included) FREEZES (stops
firing) after a large backward /clock jump, independent of ANY
uav_control_authority code. Pure rclpy timer + a script-driven /clock
publisher -- no uav_interfaces, no Gazebo.

Hypothesis under test (P10.8c Task 2, primary guess): a ROS-time timer's
next-fire deadline is `last_call_time + period` (rcl/timer.h:
rcl_timer_get_time_until_next_call), fixed at the moment of its last actual
fire. A backward /clock jump leaves that deadline in the sim FUTURE relative
to the new (lower) now -- rcl_timer_is_ready() stays false, so the timer does
not fire again until sim time climbs back past where it was BEFORE the jump.
This is generic rclpy/rclcpp sim-time timer behaviour, confirmed by reading
/opt/ros/humble/include/rcl/rcl/timer.h (no jump-callback hook exists on
rclcpp::TimerBase/GenericTimer -- grep confirmed zero matches for "jump" in
rclcpp/rclcpp/timer.hpp), not read off any uav_control_authority source.

Usage: o3_timer_freeze_probe.py [--period 0.05] [--forward-to 282] [--jump-to 34]
Prints fire count/timestamps around the jump and whether/when the timer
resumed, plus a FROZEN_UNTIL_CAUGHT_UP verdict. Exit 0 always (diagnostic
only, not a gate -- no product code is under test here).
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time as TimeMsg


def stamp(sec):
    t = TimeMsg()
    t.sec = int(sec)
    t.nanosec = int((sec - int(sec)) * 1e9)
    return t


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--period', type=float, default=0.05,
                         help='timer period, sec (0.05 = control_authority_manager_node kMonitorPeriodSec)')
    parser.add_argument('--forward-to', type=float, default=282.0,
                         help='sim time reached before the jump (matches the real o3-loop bag wrap point)')
    parser.add_argument('--jump-to', type=float, default=34.0,
                         help='sim time jumped BACK to (matches the real o3-loop bag wrap point)')
    parser.add_argument('--resume-margin', type=float, default=5.0,
                         help='keep publishing this far past forward-to after resuming, to confirm recovery')
    args = parser.parse_args()

    rclpy.init()
    node = Node(
        'o3_timer_freeze_probe',
        parameter_overrides=[Parameter('use_sim_time', value=True)])

    clock_pub = node.create_publisher(Clock, '/clock', 1)

    fires = []   # (wall_monotonic, sim_seconds_reported_by_now())

    def on_timer():
        fires.append((time.monotonic(), node.get_clock().now().nanoseconds / 1e9))

    node.create_timer(args.period, on_timer)

    def publish_clock(sec):
        msg = Clock()
        msg.clock = stamp(sec)
        clock_pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)

    # Phase 1: ramp forward 0 -> forward_to in steps well above the period
    # (so the timer fires roughly once per published /clock value -- this is
    # the POSITIVE CONTROL half of the same run: it proves the harness and
    # the timer both work normally under a plain forward-moving clock).
    step = max(args.period * 3.0, 0.05)
    t = 0.0
    while t < args.forward_to:
        t += step
        publish_clock(t)
    fires_before_jump = len(fires)
    last_sim_before_jump = fires[-1][1] if fires else float('nan')
    print('PHASE1_FORWARD_RAMP fires=%d last_fire_sim=%.3f (target=%.3f)' %
          (fires_before_jump, last_sim_before_jump, args.forward_to))
    if fires_before_jump == 0:
        print('FAILED_TO_MEASURE: timer never fired during the forward ramp -- harness broken')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Phase 2: JUMP BACKWARD to jump_to (matches the real bag's 282->34 wrap),
    # then resume ramping forward with the SAME step past forward_to +
    # resume_margin -- watch for the FIRST fire after the jump.
    wall_jump = time.monotonic()
    t = args.jump_to
    publish_clock(t)
    target = args.forward_to + args.resume_margin
    while t < target:
        t += step
        publish_clock(t)
    wall_end = time.monotonic()

    fires_after = fires[fires_before_jump:]
    regression_sec = args.forward_to - args.jump_to
    print('JUMP forward_to=%.3f -> jump_to=%.3f (regression=%.3fs sim)' %
          (args.forward_to, args.jump_to, regression_sec))

    if not fires_after:
        print('FIRST_FIRE_AFTER_JUMP=NEVER (timer did not fire again before reaching target=%.3f)'
              % target)
        print('FROZEN_UNTIL_CAUGHT_UP=True (no fire at all in the whole post-jump ramp)')
    else:
        first_fire_wall, first_fire_sim = fires_after[0]
        wall_elapsed = first_fire_wall - wall_jump
        sim_elapsed_since_jump = first_fire_sim - args.jump_to
        print('FIRST_FIRE_AFTER_JUMP sim=%.3f wall_elapsed_since_jump=%.3fs '
              'sim_elapsed_since_jump=%.3fs' % (first_fire_sim, wall_elapsed, sim_elapsed_since_jump))
        # "Caught up" means the first post-jump fire lands at/after roughly
        # where the timer left off before the jump (within one step) -- i.e.
        # it sat silent through the ENTIRE regression window rather than
        # firing shortly after the jump at the new (lower) sim time.
        froze = first_fire_sim >= (args.forward_to - step)
        print(
            'FROZEN_UNTIL_CAUGHT_UP=%s (first post-jump fire sim=%.3f vs pre-jump last=%.3f, step=%.3f)'
            % (froze, first_fire_sim, args.forward_to, step))
        print(
            'SILENT_WINDOW_SIM_SEC=%.3f (fraction of the %.3fs regression the timer was silent: %.1f%%)'
            % (sim_elapsed_since_jump, regression_sec,
               100.0 * min(1.0, sim_elapsed_since_jump / regression_sec) if regression_sec > 0 else 0.0))

    print('TOTAL_FIRES=%d fires_after_jump=%d wall_seconds_for_post_jump_ramp=%.3f' %
          (len(fires), len(fires_after), wall_end - wall_jump))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
