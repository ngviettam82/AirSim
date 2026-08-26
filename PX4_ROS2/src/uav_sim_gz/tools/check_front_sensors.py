#!/usr/bin/env python3
"""Verify forward sensors carry real data; checks values, not rates."""

import argparse
import statistics
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class FrontSensorCheck(Node):
    def __init__(self, uav_id):
        super().__init__('check_front_sensors')
        prefix = '/uav/' + uav_id + '/perception/front'
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.rgb = []
        self.depth = []
        self.cloud = []
        self.info = None

        self.create_subscription(Image, prefix + '/image_raw', self.rgb.append, sensor_qos)
        self.create_subscription(Image, prefix + '/depth_image', self.depth.append, sensor_qos)
        self.create_subscription(PointCloud2, prefix + '/points', self.cloud.append, sensor_qos)
        self.create_subscription(
            CameraInfo, prefix + '/camera_info', self._on_info, sensor_qos)

    def _on_info(self, msg):
        self.info = msg


def spread_across(buffer, samples=4096):
    """Stride-sample the whole buffer; lesson doc section 2.5i."""
    stride = max(1, len(buffer) // samples)
    return bytes(buffer[::stride])


def describe_image(name, frames, expect_width, expect_height):
    if not frames:
        return '%-14s NO DATA' % name, False
    first = frames[0]
    distinct = len(set(spread_across(first.data)))
    detail = '%-14s %d frames  %dx%d  encoding=%s  distinct bytes=%d' % (
        name, len(frames), first.width, first.height, first.encoding, distinct)
    ok = (first.width == expect_width and first.height == expect_height
          and distinct > 4)
    if first.width != expect_width or first.height != expect_height:
        detail += '  <- expected %dx%d' % (expect_width, expect_height)
    if distinct <= 4:
        detail += '  <- flat image, sensor is looking at nothing'
    return detail, ok


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-id', default='uav0')
    parser.add_argument('--seconds', type=float, default=12.0)
    parser.add_argument('--rgb-width', type=int, default=1280)
    parser.add_argument('--rgb-height', type=int, default=720)
    args = parser.parse_args()

    rclpy.init()
    node = FrontSensorCheck(args.uav_id)
    end = node.get_clock().now().nanoseconds * 1e-9 + args.seconds
    while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    failures = []
    line, ok = describe_image('rgb', node.rgb, args.rgb_width, args.rgb_height)
    print('  ' + line)
    if not ok:
        failures.append('rgb')

    line, ok = describe_image('depth', node.depth, 640, 480)
    print('  ' + line)
    if not ok:
        failures.append('depth')

    # Absent on purpose: cloud unbridged, see README section 3.
    if node.cloud:
        cloud = node.cloud[0]
        print('  %-14s %d clouds  %dx%d  point_step=%d  fields=%s'
              % ('points', len(node.cloud), cloud.width, cloud.height,
                 cloud.point_step, ','.join(f.name for f in cloud.fields)))
    else:
        print('  %-14s not bridged (expected - reproject from depth in ROS)' % 'points')

    if node.info is not None:
        focal_x = node.info.k[0]
        print('  %-14s %dx%d  fx=%.1f' % (
            'camera_info', node.info.width, node.info.height, focal_x))
        # Topic-name collision (README pitfall 6) shows up as wrong resolution.
        if node.info.width != args.rgb_width or focal_x <= 0.0:
            failures.append('camera_info')
            print('  %-14s <- does not match the rgb stream' % '')
    else:
        print('  %-14s NO DATA' % 'camera_info')
        failures.append('camera_info')

    rates = []
    for name, frames in (('rgb', node.rgb), ('depth', node.depth)):
        if len(frames) >= 2:
            stamps = [f.header.stamp.sec + f.header.stamp.nanosec * 1e-9 for f in frames]
            span = stamps[-1] - stamps[0]
            if span > 0:
                rates.append('%s %.1f Hz' % (name, (len(frames) - 1) / span))
    print('  %-14s %s' % ('rate (sim time)', ', '.join(rates) or 'not enough frames'))

    node.destroy_node()
    rclpy.shutdown()

    if failures:
        print('  RESULT: FAIL (%s)' % ', '.join(failures))
        return 1
    print('  RESULT: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
