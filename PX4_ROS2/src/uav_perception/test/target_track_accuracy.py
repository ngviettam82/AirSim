#!/usr/bin/env python3
"""P5.5 gate: target_tracker_node id stability + velocity vs a box moving at a
known world-Z velocity (gz set_pose, paced by sim time). Drone stays parked -
this exercises the tracker's Kalman filter/association, not the control loop,
so it is not blocked by the uav0_full RTF gate (see .claude/plan/P5-perception.md
Sec 0.1/Sec 3: "drone HOVER, target moving" is measurable now; full flight is D1).
Run via verify_target_tracker.sh.
"""

import argparse
import math
import statistics
import subprocess
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from uav_interfaces.msg import TargetTrack

# sensor_camera_front / sensor_depth_front mount, relative to base_link (uav0_full/model.sdf).
CAMERA_OFFSET_BODY = (0.12, 0.03, 0.002)

# Gazebo odometry reports the model origin, not base_link (see README / P5.3/P5.4).
MODEL_ORIGIN_TO_BASE_LINK_Z = 0.240

# Box is spawned axis-aligned; a large spawn yaw would tilt the boresight math.
YAW_TOLERANCE_RAD = math.radians(5.0)

# Original P5.5 gate value; 0.15 was an undocumented drift.
VELOCITY_TOLERANCE_M_S = 0.1
MIN_TRACKING_FRACTION = 0.5
MIN_TRACKING_SAMPLES = 5


class Failure(Exception):
    """Raised when the geometry cannot be trusted; the run reports no number."""


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def converged_velocity_samples(tracking):
    # Drop each track's first sample: KF inits velocity at 0.
    seen_track_ids = set()
    samples = []
    for msg in tracking:
        if msg.track_id in seen_track_ids:
            samples.append(msg.velocity.y)
        else:
            seen_track_ids.add(msg.track_id)
    return samples


class TargetTrackAccuracy(Node):
    def __init__(self, uav_id, world):
        super().__init__(
            'target_track_accuracy',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.world = world
        self.truth = None
        self.collecting = False
        self.messages = []  # list of TargetTrack
        self.set_pose_calls = 0
        self.set_pose_failures = 0

        prefix = '/uav/' + uav_id
        self.create_subscription(
            Odometry, prefix + '/localization/vio_odometry_raw', self._on_truth, 20)
        self.create_subscription(
            TargetTrack, prefix + '/perception/target_track', self._on_track, 20)

    def _on_truth(self, msg):
        self.truth = msg

    def _on_track(self, msg):
        if self.collecting:
            self.messages.append(msg)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _prime_clock(self):
        # Priming avoids a deadline computed before clock arrives.
        tries = 0
        while rclpy.ok() and self.now_sec() <= 0.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            tries += 1
            if tries > 100:
                raise Failure('never received a /clock sample')

    def spin_for(self, seconds):
        self._prime_clock()
        end = self.now_sec() + seconds
        while rclpy.ok() and self.now_sec() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_until(self, predicate, timeout, description):
        self._prime_clock()
        end = self.now_sec() + timeout
        while rclpy.ok() and self.now_sec() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise Failure('timed out waiting for %s' % description)

    def camera_world_pose(self):
        """(x, y, z, yaw) of the front camera in world, from ground-truth odometry."""
        if self.truth is None:
            raise Failure('no ground-truth odometry yet')
        p = self.truth.pose.pose.position
        yaw = yaw_from_quaternion(self.truth.pose.pose.orientation)
        if abs(yaw) > YAW_TOLERANCE_RAD:
            raise Failure(
                'spawn yaw %.1f deg exceeds the axis-aligned test assumption (+-5 deg)'
                % math.degrees(yaw))
        base_x, base_y, base_z = p.x, p.y, p.z + MODEL_ORIGIN_TO_BASE_LINK_Z
        dx, dy, dz = CAMERA_OFFSET_BODY
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        cam_x = base_x + dx * cos_y - dy * sin_y
        cam_y = base_y + dx * sin_y + dy * cos_y
        cam_z = base_z + dz
        return cam_x, cam_y, cam_z, yaw

    def set_box_pose(self, name, x, y, z):
        """NOT exercised by the author of this script (sim access reserved for
        the verifier, see verify_target_tracker.sh header). /world/<w>/set_pose
        (gz.msgs.Pose) is the same UserCommands service family already proven
        by .../create in the sibling P5.3/P5.4 scripts, but the exact request
        schema has not been run here - if every call fails, see the WARNING
        this prints below and check `gz service -i -s /world/<w>/set_pose`."""
        request = "name: '%s', position: {x: %.4f, y: %.4f, z: %.4f}" % (name, x, y, z)
        self.set_pose_calls += 1
        result = subprocess.run(
            ['gz', 'service', '-s', '/world/%s/set_pose' % self.world,
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000', '--req', request],
            capture_output=True, text=True, check=False)
        if result.returncode != 0:
            self.set_pose_failures += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--world', default='uav_arena')
    parser.add_argument('--box-name', default='p55_moving_box')
    parser.add_argument('--standoff', type=float, default=4.0,
                         help='camera-to-box-centre distance along the boresight at t=0, m')
    parser.add_argument('--vertical-speed', type=float, default=0.3,
                         help='world +Z velocity commanded for the box, m/s')
    parser.add_argument('--duration', type=float, default=8.0, help='motion duration, sim s')
    parser.add_argument('--settle', type=float, default=3.0,
                         help='seconds to wait after spawn before moving, sim s')
    parser.add_argument('--command-period', type=float, default=0.1,
                         help='gz set_pose command period, sim s')
    parser.add_argument('--pose-only', action='store_true',
                         help='print "BOX_POSE x y z yaw" for the t=0 spawn pose and exit')
    args = parser.parse_args()

    rclpy.init()
    node = TargetTrackAccuracy(args.uav_id, args.world)

    try:
        node.wait_until(lambda: node.truth is not None, 30.0, 'ground-truth odometry')
        cam_x, cam_y, cam_z, yaw = node.camera_world_pose()
    except Failure as exc:
        print('P5.5 FAILED TO MEASURE: %s' % exc)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    box_x = cam_x + args.standoff * math.cos(yaw)
    box_y = cam_y + args.standoff * math.sin(yaw)
    # Centred on the boresight height: motion spans [-half, +half] around it,
    # comfortably inside the depth camera's vertical FOV at this standoff
    # (see verify_target_tracker.sh header for the FOV budget arithmetic).
    box_z0 = cam_z - args.vertical_speed * args.duration / 2.0

    if args.pose_only:
        print('BOX_POSE %.4f %.4f %.4f %.4f' % (box_x, box_y, box_z0, yaw))
        node.destroy_node()
        rclpy.shutdown()
        return 0

    print('=== settling %.1f s before motion starts ===' % args.settle)
    node.spin_for(args.settle)

    print('=== moving the box at %.2f m/s (world +Z) for %.1f s ===' %
          (args.vertical_speed, args.duration))
    node.collecting = True
    t0 = node.now_sec()
    end = t0 + args.duration
    while rclpy.ok() and node.now_sec() < end:
        elapsed = node.now_sec() - t0
        z = box_z0 + args.vertical_speed * elapsed
        node.set_box_pose(args.box_name, box_x, box_y, z)
        node.spin_for(args.command_period)
    node.collecting = False

    node.destroy_node()
    rclpy.shutdown()

    if node.set_pose_calls and node.set_pose_failures == node.set_pose_calls:
        print('WARNING: every gz set_pose call failed (%d/%d) - the service name or request '
              'schema may not match this Gazebo version. Run `gz service -l` and '
              '`gz service -i -s /world/%s/set_pose`, then fix set_box_pose() before trusting '
              'a FAIL below.' % (node.set_pose_failures, node.set_pose_calls, args.world))

    if not node.messages:
        print('P5.5 FAILED TO MEASURE: no TargetTrack received; is target_tracker_node running?')
        return 1

    track_ids = sorted(set(msg.track_id for msg in node.messages))
    tracking = [msg for msg in node.messages if msg.status == TargetTrack.STATUS_TRACKING]
    tracking_fraction = len(tracking) / len(node.messages)

    # Expected velocity in the OPTICAL frame: world +Z (up) maps to optical -Y
    # (down) for a level, zero-roll/pitch camera boresight (see README).
    expected_vy_optical = -args.vertical_speed
    vy_samples = converged_velocity_samples(tracking)

    print()
    print('=== P5.5 target tracker, box moving %.2f m/s (world +Z), standoff %.2f m ===' %
          (args.vertical_speed, args.standoff))
    print('  messages received: %d, distinct track_id: %s' % (len(node.messages), track_ids))
    print('  status TRACKING fraction: %.2f (%d/%d)' %
          (tracking_fraction, len(tracking), len(node.messages)))

    failures = []
    if len(track_ids) != 1:
        failures.append('track_id was not stable across the run: saw %s' % track_ids)
    if tracking_fraction < MIN_TRACKING_FRACTION:
        failures.append('TRACKING fraction %.2f below %.2f' %
                         (tracking_fraction, MIN_TRACKING_FRACTION))
    if len(vy_samples) < MIN_TRACKING_SAMPLES:
        failures.append('only %d converged TRACKING samples with velocity, too few to judge' %
                         len(vy_samples))
    else:
        vy_median = statistics.median(vy_samples)
        vy_error = vy_median - expected_vy_optical
        print('  velocity.y (optical, expect %.3f)  median %.3f (n=%d, converged)  '
              'error %+.3f m/s' % (expected_vy_optical, vy_median, len(vy_samples), vy_error))
        if abs(vy_error) > VELOCITY_TOLERANCE_M_S:
            failures.append('velocity.y error %.3f m/s exceeds %.2f' %
                             (vy_error, VELOCITY_TOLERANCE_M_S))

    if failures:
        print('  RESULT: FAIL (%s)' % '; '.join(failures))
        return 1
    print('  RESULT: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
