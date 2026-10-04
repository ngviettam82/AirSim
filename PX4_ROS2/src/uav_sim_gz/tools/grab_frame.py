#!/usr/bin/env python3
"""Save and describe one camera frame; no cv2 (README 9)."""

import argparse
import sys

import numpy as np
import rclpy
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class FrameGrabber(Node):
    def __init__(self, topic):
        super().__init__('grab_frame')
        self.frames = []
        self.create_subscription(
            Image, topic, self.frames.append,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/uav/uav0/perception/down/image_raw')
    parser.add_argument('--out', default='/tmp/frame.png')
    parser.add_argument('--seconds', type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = FrameGrabber(args.topic)
    end = node.get_clock().now().nanoseconds * 1e-9 + args.seconds
    while rclpy.ok() and not node.frames and \
            node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not node.frames:
        print('no frame on %s' % args.topic)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    message = node.frames[-1]
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(message.encoding)
    if channels is None:
        print('unhandled encoding %s' % message.encoding)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    array = np.frombuffer(bytes(message.data), dtype=np.uint8)
    array = array.reshape(message.height, message.width, channels)
    if message.encoding == 'bgr8':
        array = array[:, :, ::-1]

    grey = array.mean(axis=2) if channels == 3 else array[:, :, 0]
    PilImage.fromarray(array.squeeze()).save(args.out)

    print('%dx%d %s -> %s' % (message.width, message.height, message.encoding, args.out))
    print('  brightness  min %3d  median %3d  max %3d'
          % (grey.min(), np.median(grey), grey.max()))
    print('  distinct grey levels: %d' % len(np.unique(grey.astype(np.uint8))))
    # A printed marker is nearly bimodal: mostly black or white.
    dark = float((grey < 60).mean())
    bright = float((grey > 195).mean())
    print('  very dark %.1f%%   very bright %.1f%%' % (100 * dark, 100 * bright))
    if dark < 0.005:
        print('  -> almost no dark pixels: no printed pattern is visible')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
