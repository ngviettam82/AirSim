#!/usr/bin/env python3
"""P5.4 gate: obstacle distance/size vs a box at a known pose. Run via verify_obstacle_extractor.sh."""

import argparse
import math
import statistics
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from uav_interfaces.msg import ObstacleArray

# sensor_camera_front / sensor_depth_front mount, relative to base_link (uav0_full/model.sdf).
CAMERA_OFFSET_BODY = (0.12, 0.03, 0.002)

# Gazebo odometry reports the model origin, not base_link (see README / P5.3).
MODEL_ORIGIN_TO_BASE_LINK_Z = 0.240

# Box is spawned axis-aligned; a large spawn yaw would tilt it off the boresight.
YAW_TOLERANCE_RAD = math.radians(5.0)

DISTANCE_TOLERANCE_M = 0.15
SIZE_TOLERANCE_M = 0.15
MIN_CONFIDENCE = 0.3


class Failure(Exception):
    """Raised when the geometry cannot be trusted; the run reports no number."""


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ObstacleAccuracy(Node):
    def __init__(self, uav_id):
        super().__init__(
            'obstacle_accuracy',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        prefix = '/uav/' + uav_id
        self.truth = None
        self.collecting = False
        self.frames = []  # raw ObstacleArray messages

        self.create_subscription(
            Odometry, prefix + '/localization/vio_odometry_raw', self._on_truth, 20)
        self.create_subscription(
            ObstacleArray, prefix + '/perception/obstacles_local', self._on_obstacles, 20)

    def _on_truth(self, msg):
        self.truth = msg

    def _on_obstacles(self, msg):
        if self.collecting:
            self.frames.append(msg)

    def spin(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_until(self, predicate, timeout, description):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
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


def best_match(obstacles, expected_distance, expected_width, expected_height):
    """Nearest reported obstacle to the known box; the scene may also hold
    the ground plane or other clutter, so 'exactly one' is not assumed."""
    best = None
    best_score = None
    for obstacle in obstacles:
        score = (abs(obstacle.distance - expected_distance) +
                 abs(obstacle.size.x - expected_width) +
                 abs(obstacle.size.y - expected_height))
        if best_score is None or score < best_score:
            best, best_score = obstacle, score
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--standoff', type=float, default=3.0,
                         help='camera-to-box-centre distance along the boresight, m')
    parser.add_argument('--box-thickness', type=float, default=0.4)
    parser.add_argument('--box-width', type=float, default=1.0)
    parser.add_argument('--box-height', type=float, default=0.6)
    # uav_arena's ground plane is valid depth almost right up to the horizon
    # (>=~0.7 deg below level, see README pitfall); float the box clear above
    # camera height so its whole silhouette borders sky, not ground.
    parser.add_argument('--vertical-offset', type=float, default=0.6,
                         help='box centre height above the camera, m')
    parser.add_argument('--seconds', type=float, default=15.0)
    parser.add_argument('--pose-only', action='store_true',
                         help='print "BOX_POSE x y z yaw" and exit (no measurement)')
    args = parser.parse_args()

    rclpy.init()
    node = ObstacleAccuracy(args.uav_id)

    try:
        node.wait_until(lambda: node.truth is not None, 30.0, 'ground-truth odometry')
        cam_x, cam_y, cam_z, yaw = node.camera_world_pose()
    except Failure as exc:
        print('P5.4 FAILED TO MEASURE: %s' % exc)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if args.pose_only:
        box_x = cam_x + args.standoff * math.cos(yaw)
        box_y = cam_y + args.standoff * math.sin(yaw)
        box_z = cam_z + args.vertical_offset
        print('BOX_POSE %.4f %.4f %.4f %.4f' % (box_x, box_y, box_z, yaw))
        node.destroy_node()
        rclpy.shutdown()
        return 0

    node.collecting = True
    node.spin(args.seconds)
    node.collecting = False

    node.destroy_node()
    rclpy.shutdown()

    if not node.frames:
        print('P5.4 FAILED TO MEASURE: no ObstacleArray received; is the node running?')
        return 1

    expected_distance = args.standoff - args.box_thickness / 2.0
    expected_width = args.box_width
    expected_height = args.box_height

    matched = []
    counts = []
    for frame in node.frames:
        counts.append(len(frame.obstacles))
        candidate = best_match(frame.obstacles, expected_distance, expected_width, expected_height)
        if candidate is not None:
            matched.append(candidate)

    print()
    print('=== P5.4 obstacle extractor, box %.2fx%.2fx%.2f m at %.2f m ===' %
          (args.box_thickness, args.box_width, args.box_height, args.standoff))
    print('  frames received: %d, obstacles/frame min=%d max=%d median=%.1f' %
          (len(node.frames), min(counts), max(counts), statistics.median(counts)))

    if len(matched) < 5:
        print('  RESULT: FAIL (only %d frames had a matching obstacle)' % len(matched))
        return 1

    distance = statistics.median([m.distance for m in matched])
    size_x = statistics.median([m.size.x for m in matched])
    size_y = statistics.median([m.size.y for m in matched])
    size_z = statistics.median([m.size.z for m in matched])
    confidence = statistics.median([m.confidence for m in matched])

    distance_error = distance - expected_distance
    width_error = size_x - expected_width
    height_error = size_y - expected_height

    print('  n=%d  distance %.3f m (expected %.3f, error %+.3f)' %
          (len(matched), distance, expected_distance, distance_error))
    print('  size.x (width)  %.3f m (expected %.3f, error %+.3f)' %
          (size_x, expected_width, width_error))
    print('  size.y (height) %.3f m (expected %.3f, error %+.3f)' %
          (size_y, expected_height, height_error))
    print('  size.z (depth axis, single face -> not the box thickness) %.3f m' % size_z)
    print('  confidence %.2f' % confidence)

    failures = []
    if abs(distance_error) > DISTANCE_TOLERANCE_M:
        failures.append('distance error %.3f m' % distance_error)
    if abs(width_error) > SIZE_TOLERANCE_M:
        failures.append('width error %.3f m' % width_error)
    if abs(height_error) > SIZE_TOLERANCE_M:
        failures.append('height error %.3f m' % height_error)
    if confidence < MIN_CONFIDENCE:
        failures.append('confidence %.2f below %.2f' % (confidence, MIN_CONFIDENCE))

    if failures:
        print('  RESULT: FAIL (%s)' % '; '.join(failures))
        return 1
    print('  RESULT: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
