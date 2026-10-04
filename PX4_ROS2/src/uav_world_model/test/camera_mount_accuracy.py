#!/usr/bin/env python3
"""P5 closing gate (b): camera_front mount vs a box at a known world pose. Run via verify_camera_mount.sh."""

import argparse
import json
import math
import statistics
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from uav_interfaces.msg import LocalizationStatus, ObstacleArray

# Aiming only; the compared truth is the commanded spawn pose.
CAMERA_OFFSET_BODY = (0.12, 0.03, 0.002)

# Gazebo odometry reports the model origin; base_link sits this much higher.
MODEL_ORIGIN_TO_BASE_LINK = 0.240

# Box is spawned axis-aligned; larger yaw breaks the front-face-only assumption.
YAW_TOLERANCE_RAD = math.radians(5.0)

# A dropped mount (0.12, 0.03, 0.002) shifts the map only 0.124 m, so the
# S5-style 0.15 m total tolerance cannot catch it. Two distances split the
# error: constant term = mount translation, slope term = rotation / range scale.
CONSTANT_TOLERANCE_M = (0.08, 0.05, 0.05)

# Slope x absorbs the known ~1.2 percent range-scale residual (P5.3).
SLOPE_TOLERANCE = (0.03, 0.02, 0.02)

# Chain sanity per phase; the constant/slope split above is the real gate.
TOTAL_TOLERANCE_M = 0.15

# Beyond this the run diagnoses localization, not the mount (S5 value).
FRAME_OFFSET_TOLERANCE_M = 0.10

# Reject clutter: a match farther than this is not the known box.
MATCH_GATE_M = 1.0

MIN_SAMPLES = 5

# How much wall time a sim-time wait may consume before the run is unusable.
WALL_CLOCK_SAFETY_FACTOR = 6.0

SOURCE_NAMES = {0: 'none', 1: 'gps', 2: 'vio', 3: 'optical_flow', 4: 'fused'}


class Failure(Exception):
    """Raised when the geometry cannot be trusted; the run reports no number."""


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CameraMountAccuracy(Node):
    def __init__(self, uav_id, world):
        super().__init__(
            'camera_mount_accuracy',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        prefix = '/uav/' + uav_id
        self.world = world
        self.truth = None
        self.fused = None
        self.collecting = False
        self.expected = None
        self.centers = []           # (near AABB edge x, center y, center z) in odom
        self.sizes = []             # size.x, documents the underside depth span
        self.uncertainties = []
        self.offsets = []           # fused minus lifted truth, per fused message
        self.map_messages = 0
        self.frames = set()
        self.sources = set()

        self.create_subscription(
            Odometry, prefix + '/localization/vio_odometry_raw', self._on_truth, 20)
        self.create_subscription(
            Odometry, prefix + '/state/odometry_fused', self._on_fused, 10)
        self.create_subscription(
            LocalizationStatus, prefix + '/state/localization_status', self._on_status, 10)
        self.create_subscription(
            ObstacleArray, prefix + '/world/obstacle_map_local', self._on_map, 10)

    def _on_truth(self, message):
        self.truth = message

    def _on_fused(self, message):
        self.fused = message
        if self.truth is None:
            return
        fused = message.pose.pose.position
        truth = self.truth.pose.pose.position
        self.offsets.append((fused.x - truth.x,
                             fused.y - truth.y,
                             fused.z - (truth.z + MODEL_ORIGIN_TO_BASE_LINK)))

    def _on_status(self, message):
        self.sources.add(SOURCE_NAMES.get(message.active_source, str(message.active_source)))

    def _on_map(self, message):
        self.map_messages += 1
        self.frames.add(message.header.frame_id)
        if not self.collecting or self.expected is None:
            return
        best = None
        best_distance = None
        for obstacle in message.obstacles:
            center = obstacle.center
            distance = math.sqrt((center.x - self.expected[0]) ** 2 +
                                 (center.y - self.expected[1]) ** 2 +
                                 (center.z - self.expected[2]) ** 2)
            if best_distance is None or distance < best_distance:
                best, best_distance = obstacle, distance
        if best is not None and best_distance <= MATCH_GATE_M:
            # The floated box shows the camera its underside, whose points run
            # from the near to the far bottom edge, dragging the bbox CENTER
            # ~0.15-0.2 m deeper than the face (measured 2026-08-16). The near
            # AABB edge is the face regardless, so x compares that edge.
            self.centers.append((best.center.x - best.size.x / 2.0,
                                 best.center.y,
                                 best.center.z))
            self.sizes.append(best.size.x)
            self.uncertainties.append(best.position_uncertainty)

    def sim_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin(self, seconds):
        start_sim = self.sim_seconds()
        wall_deadline = time.time() + seconds * WALL_CLOCK_SAFETY_FACTOR
        while rclpy.ok() and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.sim_seconds() - start_sim >= seconds:
                return
        raise Failure('sim clock advanced %.1f s of %.1f s before the wall clock ran out'
                      % (self.sim_seconds() - start_sim, seconds))

    def wait_until(self, predicate, timeout, description):
        wall_deadline = time.time() + timeout * WALL_CLOCK_SAFETY_FACTOR
        while rclpy.ok() and time.time() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise Failure('timed out waiting for %s' % description)

    def camera_world_pose(self):
        if self.truth is None:
            raise Failure('no ground-truth odometry yet')
        p = self.truth.pose.pose.position
        yaw = yaw_from_quaternion(self.truth.pose.pose.orientation)
        if abs(yaw) > YAW_TOLERANCE_RAD:
            raise Failure('spawn yaw %.1f deg breaks the axis-aligned assumption'
                          % math.degrees(yaw))
        base = (p.x, p.y, p.z + MODEL_ORIGIN_TO_BASE_LINK)
        dx, dy, dz = CAMERA_OFFSET_BODY
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        return (base[0] + dx * cos_y - dy * sin_y,
                base[1] + dx * sin_y + dy * cos_y,
                base[2] + dz,
                yaw)

    def set_box_pose(self, name, x, y, z):
        request = "name: '%s', position: {x: %.4f, y: %.4f, z: %.4f}" % (name, x, y, z)
        result = subprocess.run(
            ['gz', 'service', '-s', '/world/%s/set_pose' % self.world,
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000', '--req', request],
            capture_output=True, text=True, check=False)
        # A silent no-op still fails: phase far then matches nothing.
        if result.returncode != 0:
            raise Failure('gz set_pose failed: %s %s'
                          % (result.stdout.strip(), result.stderr.strip()))

    def measure_phase(self, label, face_world, seconds):
        offset = self.median_offset()
        if offset is None:
            raise Failure('no fused/truth pair, frame offset unknown')
        self.expected = tuple(face_world[axis] + offset[axis] for axis in range(3))
        self.centers = []
        self.sizes = []
        self.uncertainties = []
        self.collecting = True
        self.spin(seconds)
        self.collecting = False
        if len(self.centers) < MIN_SAMPLES:
            raise Failure('phase %s matched only %d map entries (map messages %d)'
                          % (label, len(self.centers), self.map_messages))
        measured = tuple(statistics.median([c[axis] for c in self.centers])
                         for axis in range(3))
        error = tuple(measured[axis] - self.expected[axis] for axis in range(3))
        return {
            'label': label,
            'samples': len(self.centers),
            'expected': self.expected,
            'measured': measured,
            'error': error,
            'total': math.sqrt(sum(component ** 2 for component in error)),
            'size_x': statistics.median(self.sizes),
            'uncertainty': statistics.median(self.uncertainties),
            'frame_offset': offset,
        }

    def median_offset(self):
        if not self.offsets:
            return None
        recent = self.offsets[-50:]
        return tuple(statistics.median([o[axis] for o in recent]) for axis in range(3))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--world', default='uav_arena')
    parser.add_argument('--box-name', default='pb_mount_box')
    parser.add_argument('--box-x', type=float, required=True)
    parser.add_argument('--box-y', type=float, required=True)
    parser.add_argument('--box-z', type=float, required=True)
    parser.add_argument('--box-thickness', type=float, default=0.4)
    parser.add_argument('--standoff-near', type=float, default=3.0)
    parser.add_argument('--standoff-far', type=float, default=5.0)
    parser.add_argument('--seconds', type=float, default=20.0)
    parser.add_argument('--settle', type=float, default=8.0)
    parser.add_argument('--out', default='')
    args = parser.parse_args()

    rclpy.init()
    node = CameraMountAccuracy(args.uav_id, args.world)
    results = []
    try:
        node.wait_until(lambda: node.truth is not None, 30.0, 'ground-truth odometry')
        node.wait_until(lambda: node.fused is not None, 60.0, 'fused odometry')
        node.wait_until(lambda: node.map_messages > 0, 60.0, 'obstacle map')
        cam = node.camera_world_pose()
        yaw = cam[3]

        # x compares the near AABB edge, which is the near face.
        half = args.box_thickness / 2.0
        near_face = (args.box_x - half * math.cos(yaw),
                     args.box_y - half * math.sin(yaw),
                     args.box_z)
        node.spin(3.0)
        results.append(node.measure_phase('near', near_face, args.seconds))

        far_center = (cam[0] + args.standoff_far * math.cos(yaw),
                      cam[1] + args.standoff_far * math.sin(yaw),
                      args.box_z)
        node.set_box_pose(args.box_name, *far_center)
        far_face = (far_center[0] - half * math.cos(yaw),
                    far_center[1] - half * math.sin(yaw),
                    far_center[2])
        node.spin(args.settle)
        results.append(node.measure_phase('far', far_face, args.seconds))

        ranges = [math.sqrt((face[0] - cam[0]) ** 2 +
                            (face[1] - cam[1]) ** 2 +
                            (face[2] - cam[2]) ** 2)
                  for face in (near_face, far_face)]
    except Failure as exc:
        print('P5(b) FAILED TO MEASURE: %s' % exc)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    near, far = results
    span = ranges[1] - ranges[0]
    slope = tuple((far['error'][axis] - near['error'][axis]) / span for axis in range(3))
    constant = tuple(near['error'][axis] - slope[axis] * ranges[0] for axis in range(3))

    print()
    print('=== P5 closing gate (b): camera_front mount, box %s ===' % args.box_name)
    print('  localization     source(s) %s' % (', '.join(sorted(node.sources)) or 'unknown'))
    print('  map frame        %s' % (', '.join(sorted(node.frames)) or 'unknown'))
    for result, slant in zip(results, ranges):
        print('  phase %-5s      n=%d  slant %.3f m' % (result['label'], result['samples'], slant))
        print('    expected odom  (%.3f %.3f %.3f) m' % result['expected'])
        print('    measured odom  (%.3f %.3f %.3f) m' % result['measured'])
        print('    error          (%+.3f %+.3f %+.3f) m, total %.3f m (tolerance %.2f)'
              % (result['error'] + (result['total'], TOTAL_TOLERANCE_M)))
        print('    size.x         %.3f m (underside depth span; face alone would be ~0)'
              % result['size_x'])
        print('    frame offset   (%+.3f %+.3f %+.3f) m   uncertainty %.3f m'
              % (result['frame_offset'] + (result['uncertainty'],)))
    print('  mount residual   constant (%+.3f %+.3f %+.3f) m  (tolerance %.2f/%.2f/%.2f)'
          % (constant + CONSTANT_TOLERANCE_M))
    print('  range-dependent  slope    (%+.4f %+.4f %+.4f) m/m (tolerance %.2f/%.2f/%.2f)'
          % (slope + SLOPE_TOLERANCE))

    failures = []
    axes = 'xyz'
    for result in results:
        worst = max(abs(component) for component in result['frame_offset'])
        if worst > FRAME_OFFSET_TOLERANCE_M:
            failures.append('phase %s frame offset %.3f m: odom itself is wrong'
                            % (result['label'], worst))
        if result['total'] > TOTAL_TOLERANCE_M:
            failures.append('phase %s off by %.3f m' % (result['label'], result['total']))
        if result['uncertainty'] <= 0.0:
            failures.append('phase %s uncertainty is not positive' % result['label'])
    if node.frames - {'odom'}:
        failures.append('map arrived in %s, not odom' % sorted(node.frames))
    for axis in range(3):
        if abs(constant[axis]) > CONSTANT_TOLERANCE_M[axis]:
            failures.append('mount translation residual %s %+.3f m'
                            % (axes[axis], constant[axis]))
        if abs(slope[axis]) > SLOPE_TOLERANCE[axis]:
            failures.append('range-dependent residual %s %+.4f m/m'
                            % (axes[axis], slope[axis]))

    if args.out:
        with open(args.out, 'w') as handle:
            json.dump({
                'phases': results,
                'slant_ranges': ranges,
                'constant': constant,
                'slope': slope,
                'sources': sorted(node.sources),
                'failures': failures,
            }, handle)

    node.destroy_node()
    rclpy.shutdown()

    if failures:
        print('  RESULT: FAIL (%s)' % '; '.join(failures))
        return 1
    print('  RESULT: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
