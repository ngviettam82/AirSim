#!/usr/bin/env python3
"""Gate G2-B/G2-E: the mux must not add error to the source it selected.

Replaces the old G2 condition B, which injected 0.5 m into VIO and then demanded the
output land within 0.5 m of truth -- asking a mux that SELECTS to behave like one that
FILTERS. That gate could not pass by construction. This one measures the only thing the
mux actually promises, so it stays meaningful however hard the injectors are turned.

Two stub sources drive the real localization_mux_node: no Gazebo, no PX4, no RTF gate.

Three claims, each with the control that would expose it as empty:
  B1 fidelity   away from a continuity window the published pose IS the source pose
  B2 continuity the offset decays no faster than continuity_decay_rate_mps per axis
  B3 honesty    while the offset slides the pose, twist covariance may not read certain

Usage:  mux_fidelity_bench.py [--uav-id uav0] [--label name]
Exit:   0 pass  |  1 fail  |  2 FAILED TO MEASURE (never read this as a pass)
"""

import argparse
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from uav_interfaces.msg import LocalizationStatus

STUB_HZ = 50.0
MUX_HZ = 10.0
DECAY_RATE = 0.5             # continuity_decay_rate_mps, mux default
DISAGREEMENT_M = 2.0         # deliberate gap between the two stubs, on x only
VIO_STDDEV = 0.10
GPS_STDDEV = 0.90
SPEED = 0.3162               # hypot(0.30, 0.10), the stub cruise speed

FIDELITY_TOL_M = 1e-6
MAX_DECAY_MPS = math.sqrt(3.0) * DECAY_RATE
# One stub tick of travel is the floor of any offset measurement; judge well above it.
OFFSET_FLOOR_M = 8.0 * SPEED / STUB_HZ

PHASE_VIO_ONLY = 6.0         # wall seconds, the same clock every measurement uses
PHASE_GPS_ONLY = 10.0        # 2.0 m at 0.5 m/s decays in 4 s, so this holds the tail
PHASE_VIO_BACK = 8.0
HISTORY = 200


def norm(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def pose_at(history, stamp):
    """Where a source said it was at the instant the mux stamped its output."""
    if len(history) < 2:
        return None
    for i in range(len(history) - 1, 0, -1):
        t0, p0 = history[i - 1]
        t1, p1 = history[i]
        if t0 <= stamp <= t1 and t1 > t0:
            f = (stamp - t0) / (t1 - t0)
            return tuple(p0[k] + f * (p1[k] - p0[k]) for k in range(3))
    if stamp >= history[-1][0]:
        return history[-1][1]
    return None


class Bench(Node):
    def __init__(self, uav_id):
        super().__init__("mux_fidelity_bench")
        prefix = "/uav/" + uav_id
        qos = QoSProfile(depth=10)

        self.vio_odom = self.create_publisher(Odometry, prefix + "/localization/vio_odometry", qos)
        self.vio_stat = self.create_publisher(
            LocalizationStatus, prefix + "/localization/vio_status", qos)
        self.gps_odom = self.create_publisher(Odometry, prefix + "/localization/gps_odometry", qos)
        self.gps_stat = self.create_publisher(
            LocalizationStatus, prefix + "/localization/gps_status", qos)

        self.create_subscription(Odometry, prefix + "/state/odometry_fused", self._on_fused, qos)
        self.create_subscription(
            LocalizationStatus, prefix + "/state/localization_status", self._on_status, qos)

        self.uav_id = uav_id
        self.started = time.time()
        self.vio_alive = True
        self.samples = []        # (stamp, fused_xyz, twist_var, vio_hist, gps_hist)
        self.labels = []         # (stamp, position_uncertainty)
        self.vio_sent = []       # (wall stamp, pose) actually put on the wire
        self.gps_sent = []
        self.create_timer(1.0 / STUB_HZ, self._tick)

    # -- stubs, parameterized by wall seconds so every clock here is the same one ---
    def vio_pose(self, elapsed):
        return (0.30 * elapsed, 0.10 * elapsed, 2.0)

    def gps_pose(self, elapsed):
        x, y, z = self.vio_pose(elapsed)
        return (x + DISAGREEMENT_M, y, z)

    def _publish(self, odom_pub, stat_pub, name, xyz, stddev):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = xyz
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance[0] = stddev ** 2
        odom.twist.twist.linear.x = 0.30
        odom.twist.twist.linear.y = 0.10
        odom_pub.publish(odom)

        status = LocalizationStatus()
        status.header.stamp = stamp
        status.uav_id = self.uav_id
        status.active_source = (
            LocalizationStatus.SOURCE_VIO if name == "vio" else LocalizationStatus.SOURCE_GPS)
        status.quality = LocalizationStatus.QUALITY_GOOD
        status.is_valid = True
        status.position_uncertainty = stddev
        status.heading_uncertainty = 0.05
        status.detail = name
        stat_pub.publish(status)

    def _tick(self):
        now = time.time()
        elapsed = now - self.started
        if self.vio_alive:
            pose = self.vio_pose(elapsed)
            self._publish(self.vio_odom, self.vio_stat, "vio", pose, VIO_STDDEV)
            self.vio_sent.append((now, pose))
            del self.vio_sent[:-HISTORY]
        pose = self.gps_pose(elapsed)
        self._publish(self.gps_odom, self.gps_stat, "gps", pose, GPS_STDDEV)
        self.gps_sent.append((now, pose))
        del self.gps_sent[:-HISTORY]

    # -- measurement ---------------------------------------------------------------
    def _on_fused(self, msg):
        p = msg.pose.pose.position
        self.samples.append((
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            (p.x, p.y, p.z),
            msg.twist.covariance[0],
            list(self.vio_sent),
            list(self.gps_sent),
        ))

    def _on_status(self, msg):
        self.labels.append((
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            msg.position_uncertainty,
        ))


def spin_for(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def await_mux(node, timeout_sec=20.0):
    """Discovery is a precondition, not a result: measuring nothing is not a pass."""
    topic = "/uav/" + node.uav_id + "/state/localization_status"
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if node.count_publishers(topic) > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uav-id", default="uav0")
    parser.add_argument("--label", default="shipped")
    args = parser.parse_args()

    rclpy.init()
    node = Bench(args.uav_id)
    print("=== G2-B/E mux fidelity: %s ===" % args.label)

    if not await_mux(node):
        print("  RESULT: FAILED TO MEASURE")
        print("    - no localization_mux_node on ROS_DOMAIN_ID=%s after 20 s"
              % os.environ.get("ROS_DOMAIN_ID", "0"))
        rclpy.shutdown()
        return 2

    node.started = time.time()
    spin_for(node, PHASE_VIO_ONLY)
    vio_only_end = time.time()
    node.vio_alive = False              # VIO dies -> GPS wins, 2 m to absorb
    spin_for(node, PHASE_GPS_ONLY)
    gps_only_end = time.time()
    node.vio_alive = True               # VIO returns -> another 2 m to absorb
    spin_for(node, PHASE_VIO_BACK)

    samples = list(node.samples)
    labels = sorted(node.labels)
    node.destroy_node()
    rclpy.shutdown()

    failures = []
    unmeasured = []

    expected = MUX_HZ * (PHASE_VIO_ONLY + PHASE_GPS_ONLY + PHASE_VIO_BACK)
    print("  fused samples      %d (expect ~%d)" % (len(samples), expected))
    if len(samples) < 0.7 * expected:
        unmeasured.append("only %d fused samples, expected ~%d" % (len(samples), expected))

    sigmas = sorted({round(u, 3) for _, u in labels})
    print("  published sigma    %s" % sigmas)
    if round(GPS_STDDEV, 3) not in sigmas:
        unmeasured.append("gps was never the selected source: the switch never happened")
    if round(VIO_STDDEV, 3) not in sigmas:
        unmeasured.append("vio was never the selected source")

    if unmeasured:
        print("  RESULT: FAILED TO MEASURE")
        for line in unmeasured:
            print("    - " + line)
        return 2

    def sigma_near(stamp):
        best, gap = None, 0.02
        for t, u in labels:
            d = abs(t - stamp)
            if d <= gap:
                best, gap = u, d
        return best

    def offset_at(stamp, fused, vio_hist, gps_hist):
        """Gap to the source the MUX SAID it picked -- never guessed from the phase."""
        sigma = sigma_near(stamp)
        if sigma is None or sigma < 0.0:
            return None
        history = vio_hist if abs(sigma - VIO_STDDEV) < 0.01 else gps_hist
        pose = pose_at(history, stamp)
        return None if pose is None else norm(fused, pose)

    # B1 fidelity. Content matching, because a time-paired comparison measures the
    # pairing: the first run of this bench returned exactly one stub tick of travel.
    settled_head, settled_tail = [], []
    for stamp, fused, _tv, vio_hist, gps_hist in samples:
        best = float("inf")
        for history in (vio_hist, gps_hist):
            for _t, pose in history:
                best = min(best, norm(fused, pose))
        if node.started + 3.0 < stamp < vio_only_end:
            settled_head.append(best)
        elif gps_only_end - 3.0 < stamp < gps_only_end:
            settled_tail.append(best)

    if not settled_head or not settled_tail:
        print("  RESULT: FAILED TO MEASURE (no settled window on one of the sources)")
        return 2
    print("  B1 fidelity vio    max %.9f m over %d samples" % (max(settled_head), len(settled_head)))
    print("  B1 fidelity gps    max %.9f m over %d samples" % (max(settled_tail), len(settled_tail)))
    if max(settled_head) > FIDELITY_TOL_M:
        failures.append("mux altered the vio pose by %.6f m" % max(settled_head))
    if max(settled_tail) > FIDELITY_TOL_M:
        failures.append("mux altered the gps pose by %.6f m" % max(settled_tail))

    # B2 continuity + B3 honesty, over the absorb windows.
    worst_rate, decaying, silent = 0.0, 0, 0
    peak_offset = 0.0
    for i in range(1, len(samples)):
        t0, f0, _tv0, v0, g0 = samples[i - 1]
        t1, f1, tv1, v1, g1 = samples[i]
        dt = t1 - t0
        if dt < 0.5 / MUX_HZ:        # a rate needs a real interval
            continue
        off0 = offset_at(t0, f0, v0, g0)
        off1 = offset_at(t1, f1, v1, g1)
        if off0 is None or off1 is None:
            continue
        peak_offset = max(peak_offset, off0)
        if off0 > OFFSET_FLOOR_M:
            decaying += 1
            worst_rate = max(worst_rate, (off0 - off1) / dt)
            if tv1 == 0.0:
                silent += 1
    print("  B2 decay           %d samples, worst %.3f m/s (ceiling %.3f), peak offset %.3f m"
          % (decaying, worst_rate, MAX_DECAY_MPS, peak_offset))
    print("  B3 twist stated    %d of %d decaying samples said 'certain'" % (silent, decaying))

    if decaying < 5:
        print("  RESULT: FAILED TO MEASURE (the absorb window was never observed)")
        return 2
    if peak_offset < 0.5 * DISAGREEMENT_M:
        print("  RESULT: FAILED TO MEASURE (peak offset %.3f m, expected near %.1f m)"
              % (peak_offset, DISAGREEMENT_M))
        return 2
    if worst_rate > MAX_DECAY_MPS * 1.05:
        failures.append("offset collapsed at %.3f m/s, faster than the declared rate" % worst_rate)
    if silent > 0:
        failures.append("%d samples slid the pose while twist covariance read certain" % silent)

    if failures:
        print("  RESULT: FAIL")
        for line in sorted(set(failures)):
            print("    - " + line)
        return 1
    print("  RESULT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
