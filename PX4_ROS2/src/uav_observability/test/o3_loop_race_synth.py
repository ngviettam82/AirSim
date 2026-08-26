#!/usr/bin/env python3
"""P10.8c diagnostic (not part of the o3-* gate rounds): follow-up to
o3_timer_freeze_probe.py's finding that a sim-time timer does NOT freeze
after a backward /clock jump (it resumes within ~1 period). This script
tests the remaining hypothesis directly against the REAL, DEPLOYED
control_authority_manager_node (not just the arbiter's own gtest harness):
does a CONTINUOUS cmd_mission stream that keeps flowing straight through a
/clock regression (never going silent -- the closest fidelity to what
`ros2 bag play --loop` actually does to a bag whose ONLY populated authority
channel is MISSION) prevent clock_regressions from ever being observed,
because MISSION keeps re-proving itself live faster than any check can catch
it stale?

Drives /clock AND control/cmd_mission on the SAME schedule (mission "keeps
transmitting", exactly like a real flight's authority winner does): ramps
sim time 0 -> forward_to while publishing a valid MISSION command every
tick, then jumps /clock BACK to jump_to and keeps publishing MISSION
commands (with fresh, non-stale headers) on the SAME post-jump schedule --
never a gap. Prints whether clock_regressions ever incremented.

Usage: o3_loop_race_synth.py --uav-id uav0
Exit 0 if the run completed and the JSON-ish report line was printed
(diagnostic only, not a gate -- always exits 0 unless the harness itself
broke).

CORRECTION (found after this script ran, cross-checked against the real
bag's own authority_edges): this "continuous traffic" scenario reproduces
the SAME symptom (clock_regressions=0) as the real --loop run, but is NOT
the mechanism actually at play on Task B's flight bag. The real bag's
reference shows authority_edges=2 (arm -> MISSION, mission-complete ->
NONE) -- MISSION releases via the ORDINARY release_dwell_sec silence
timeout well before the bag ends, so active_source is already NONE at the
instant --loop wraps; there is no channel holding a stale timestamp to
catch. Two different scenarios (continuous-through-the-jump vs.
released-then-reacquired) land on the same clock_regressions=0 outcome --
keeping both isolation experiments here since the timer-freeze-refutal
half of this script's own reasoning still holds, but do not read this
script's specific "MISSION never goes silent" framing as what actually
happened on the real bag. See docs/interface-contract-v0.1.md Sec2.20
P10.8c row for the confirmed root cause.
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--period', type=float, default=0.05,
                         help='clock+cmd_mission publish step, sec (matches monitor_timer_ period)')
    parser.add_argument('--forward-to', type=float, default=282.0)
    parser.add_argument('--jump-to', type=float, default=34.0)
    parser.add_argument('--resume-margin', type=float, default=10.0)
    args = parser.parse_args()
    prefix = '/uav/%s' % args.uav_id

    rclpy.init()
    node = Node('o3_loop_race_synth')

    clock_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    clock_pub = node.create_publisher(Clock, '/clock', clock_qos)
    cmd_pub = node.create_publisher(ControlCommand, prefix + '/control/cmd_mission', 10)

    latest = {'clock_regressions': None, 'active_source': None}
    history = []   # (wall, sim_now, clock_regressions, active_source)

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

    def publish_both(sec):
        clock_msg = Clock()
        clock_msg.clock = stamp(sec)
        clock_pub.publish(clock_msg)

        cmd = ControlCommand()
        cmd.header = Header()
        cmd.header.stamp = stamp(sec)
        cmd.header.frame_id = 'odom'
        cmd.uav_id = args.uav_id
        cmd.control_mode = ControlCommand.MODE_POSITION
        cmd.position.x, cmd.position.y, cmd.position.z = 0.0, 0.0, 1.0
        cmd.yaw = 0.0
        cmd.source = ControlAuthority.SOURCE_MISSION
        cmd_pub.publish(cmd)

        rclpy.spin_once(node, timeout_sec=0.02)
        history.append(
            (time.monotonic(), sec, latest['clock_regressions'], latest['active_source']))

    # Phase 1: ramp forward with MISSION continuously live (exactly what a
    # real flight's winning channel does) -- establishes active_source=MISSION
    # and gives the diagnostics timer many chances to cross its 20-tick
    # boundary and report a baseline BEFORE the jump.
    t = 0.0
    while t < args.forward_to:
        t += args.period
        publish_both(t)
    before = latest['clock_regressions']
    source_before = latest['active_source']
    print('BEFORE_JUMP sim=%.3f clock_regressions=%s active_source=%s' % (t, before, source_before))

    # Phase 2: JUMP BACKWARD (matches the real bag's 282->34 wrap) -- but
    # UNLIKE o3_clock_regression_synth.py, MISSION does NOT go silent here.
    # It keeps publishing on the exact same schedule, with a header.stamp
    # that matches the NEW (lower) sim time -- exactly what `ros2 bag play
    # --loop` does when it restarts a bag whose cmd_mission topic has
    # traffic spanning the whole recording.
    t = args.jump_to
    publish_both(t)
    target = args.forward_to + args.resume_margin
    while t < target:
        t += args.period
        publish_both(t)

    after = latest['clock_regressions']
    source_after = latest['active_source']
    print('AFTER_JUMP_AND_RESUME sim=%.3f clock_regressions=%s active_source=%s'
          % (t, after, source_after))

    # Did clock_regressions EVER increment at ANY point in the whole run
    # (it's monotonic, so the final sample already answers this) -- and did
    # active_source ever dip away from MISSION/NONE during the transition?
    ever_regressed = any(
        (v is not None and before is not None and v > before) for _w, _s, v, _a in history)
    saw_none = any(a == ControlAuthority.SOURCE_NONE for _w, _s, _v, a in history)
    print('EVER_REGRESSED_DURING_RUN=%s SAW_ACTIVE_SOURCE_NONE_DURING_RUN=%s N_SAMPLES=%d'
          % (ever_regressed, saw_none, len(history)))

    node.destroy_node()
    rclpy.shutdown()

    print('BEFORE=%s AFTER=%s' % (before, after))
    if before is None or after is None:
        print('FAILED TO MEASURE: never received a diagnostics sample')
        sys.exit(2)
    sys.exit(0)


if __name__ == '__main__':
    main()
