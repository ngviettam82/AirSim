import time
import unittest

import rclpy
from rclpy.parameter import Parameter

from airsim_interfaces.msg import Px4ImageSync
from airsim_px4_offboard.px4_frame_gate import Px4FrameGate
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String


class FrameGateNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    @staticmethod
    def _set_header(header, stamp_ns, frame_id='drone1/cam1_optical'):
        header.stamp.sec = stamp_ns // 1_000_000_000
        header.stamp.nanosec = stamp_ns % 1_000_000_000
        header.frame_id = frame_id

    @staticmethod
    def _header(message, stamp_ns, frame_id='drone1/cam1_optical'):
        FrameGateNodeTest._set_header(message.header, stamp_ns, frame_id)
        return message

    @staticmethod
    def _camera_info(stamp_ns, frame_id='drone1/cam1_optical', width=640, height=480):
        camera_info = CameraInfo()
        FrameGateNodeTest._set_header(camera_info.header, stamp_ns, frame_id)
        camera_info.width = width
        camera_info.height = height
        return camera_info

    @staticmethod
    def _jpeg(width=640, height=480):
        """Minimal structurally valid JPEG header for gate-parser tests."""

        return [
            0xff, 0xd8,
            0xff, 0xc0, 0x00, 0x11, 0x08,
            (height >> 8) & 0xff, height & 0xff,
            (width >> 8) & 0xff, width & 0xff,
            0x03,
            0x01, 0x11, 0x00,
            0x02, 0x11, 0x01,
            0x03, 0x11, 0x01,
            0xff, 0xda, 0x00, 0x0c, 0x03,
            0x01, 0x00, 0x02, 0x11, 0x03, 0x11,
            0x00, 0x3f, 0x00,
            0xff, 0xd9,
        ]

    @staticmethod
    def _sync(stamp_ns, *, epoch=0, image_topic='/airsim_node/drone1/cam1_Scene/image'):
        sync = Px4ImageSync()
        FrameGateNodeTest._set_header(sync.image_header, stamp_ns)
        sync.image_topic = image_topic
        sync.airsim_vehicle_name = 'drone1'
        sync.px4_topic_prefix = '/fmu'
        sync.px4_system_id = 1
        sync.px4_clock_epoch = epoch
        sync.px4_clock_timestamp_us = stamp_ns // 1_000
        sync.direct_hil_clock_verified = True
        sync.direct_hil_clock_match_count = 3
        sync.direct_hil_clock_last_matched_timestamp_us = stamp_ns // 1_000
        return sync

    @staticmethod
    def _node(*, transport='raw'):
        image_topic = '/airsim_node/drone1/cam1_Scene/image'
        if transport == 'compressed':
            image_topic += '/compressed'
        return Px4FrameGate(parameter_overrides=[
            Parameter('image_topic', value=image_topic),
            Parameter('image_transport', value=transport),
            Parameter('image_sync_topic', value='camera_sync/image_sync'),
            Parameter('sync_status_topic', value='camera_sync/status'),
            Parameter('output_prefix', value='camera_gate'),
        ])

    def test_raw_camera_sample_releases_only_after_ready_exact_sync(self):
        node = self._node()
        try:
            stamp_ns = 1_000_000_000
            image = self._header(Image(), stamp_ns)
            image.width = 640
            image.height = 480
            node._payload_cb(image)
            self.assertEqual(node.published_frames, 0)

            node._sync_status_cb(String(data='READY'))
            node._payload_cb(image)
            self.assertEqual(len(node._pending_payloads), 1)
            node._sync_cb(self._sync(stamp_ns))
            self.assertEqual(node.published_frames, 0)
            node._camera_info_cb(self._camera_info(stamp_ns))

            self.assertEqual(node.published_frames, 1)
            self.assertFalse(node._pending_payloads)
            self.assertFalse(node._pending_camera_infos)
            self.assertFalse(node._pending_syncs)
            self.assertEqual(node._last_status, 'READY')
        finally:
            node.destroy_node()

    def test_unmatched_and_nonready_payloads_are_never_released(self):
        node = self._node()
        try:
            first_stamp_ns = 1_000_000_000
            second_stamp_ns = 1_010_000_000
            node._sync_status_cb(String(data='READY'))
            image = self._header(Image(), first_stamp_ns)
            image.width = 640
            image.height = 480
            node._payload_cb(image)
            node._camera_info_cb(self._camera_info(first_stamp_ns))
            node._sync_cb(self._sync(second_stamp_ns))
            self.assertEqual(node.published_frames, 0)

            node._expire_pending(time.monotonic_ns() + node._max_frame_wait_ns + 1)
            self.assertFalse(node._pending_payloads)
            self.assertFalse(node._pending_camera_infos)
            self.assertFalse(node._pending_syncs)
            self.assertGreaterEqual(node.dropped_frames, 3)

            node._sync_status_cb(String(data='STALE_SYNC'))
            image = self._header(Image(), second_stamp_ns)
            image.width = 640
            image.height = 480
            node._payload_cb(image)
            node._camera_info_cb(self._camera_info(second_stamp_ns))
            node._sync_cb(self._sync(second_stamp_ns))
            self.assertEqual(node.published_frames, 0)
        finally:
            node.destroy_node()

    def test_epoch_fence_requires_a_new_ready_status(self):
        node = self._node()
        try:
            stamp_ns = 1_000_000_000
            node._sync_status_cb(String(data='READY'))
            image = self._header(Image(), stamp_ns)
            image.width = 640
            image.height = 480
            node._payload_cb(image)
            node._camera_info_cb(self._camera_info(stamp_ns))
            node._sync_cb(self._sync(stamp_ns + 5_000_000, epoch=0))
            self.assertEqual(len(node._pending_payloads), 1)

            node._sync_cb(self._sync(stamp_ns + 10_000_000, epoch=1))
            self.assertFalse(node._input_ready)
            self.assertFalse(node._pending_payloads)
            self.assertIsNone(node._last_released_stamp_ns)

            new_stamp_ns = stamp_ns + 20_000_000
            node._sync_status_cb(String(data='READY'))
            node._sync_cb(self._sync(new_stamp_ns, epoch=1))
            image = self._header(Image(), new_stamp_ns)
            image.width = 640
            image.height = 480
            node._payload_cb(image)
            node._camera_info_cb(self._camera_info(new_stamp_ns))
            self.assertEqual(node.published_frames, 1)
        finally:
            node.destroy_node()

    def test_compressed_payload_uses_raw_sync_topic_and_preserves_gate(self):
        node = self._node(transport='compressed')
        try:
            stamp_ns = 1_000_000_000
            node._sync_status_cb(String(data='READY'))
            node._sync_cb(self._sync(stamp_ns))
            image = self._header(CompressedImage(), stamp_ns)
            image.data = self._jpeg()
            node._payload_cb(image)
            node._camera_info_cb(self._camera_info(stamp_ns))
            self.assertEqual(node.published_frames, 1)
            self.assertEqual(
                node._expected_sync_image_topic,
                '/airsim_node/drone1/cam1_Scene/image',
            )
        finally:
            node.destroy_node()

    def test_invalid_identity_fails_closed(self):
        node = self._node()
        try:
            node._sync_status_cb(String(data='READY'))
            sync = self._sync(1_000_000_000)
            sync.px4_system_id = 2
            node._sync_cb(sync)
            self.assertIsNotNone(node.fatal_error)
            self.assertEqual(node.published_frames, 0)
        finally:
            node.destroy_node()

    def test_source_age_bound_rejects_an_exact_but_old_frame(self):
        node = self._node()
        try:
            stamp_ns = 1_000_000_000
            node._sync_status_cb(String(data='READY'))
            image = self._header(Image(), stamp_ns)
            image.width = 640
            image.height = 480
            node._payload_cb(image)
            node._camera_info_cb(self._camera_info(stamp_ns))
            sync = self._sync(stamp_ns)
            sync.px4_clock_timestamp_us += node._max_source_age_ns // 1_000 + 1
            node._sync_cb(sync)

            self.assertEqual(node.published_frames, 0)
            self.assertFalse(node._pending_syncs)
            self.assertEqual(len(node._pending_payloads), 1)
        finally:
            node.destroy_node()

    def test_mismatched_camera_info_never_releases_a_frame(self):
        node = self._node()
        try:
            stamp_ns = 1_000_000_000
            image = self._header(Image(), stamp_ns)
            image.width = 640
            image.height = 480
            node._sync_status_cb(String(data='READY'))
            node._payload_cb(image)
            node._camera_info_cb(self._camera_info(stamp_ns, width=320, height=240))
            node._sync_cb(self._sync(stamp_ns))

            self.assertEqual(node.published_frames, 0)
            self.assertGreaterEqual(node.dropped_frames, 1)
        finally:
            node.destroy_node()

    def test_invalid_compressed_payload_never_releases_a_frame(self):
        node = self._node(transport='compressed')
        try:
            stamp_ns = 1_000_000_000
            node._sync_status_cb(String(data='READY'))
            image = self._header(CompressedImage(), stamp_ns)
            image.data = [0x00, 0x01]
            node._payload_cb(image)
            node._camera_info_cb(self._camera_info(stamp_ns))
            node._sync_cb(self._sync(stamp_ns))

            self.assertEqual(node.published_frames, 0)
            self.assertGreaterEqual(node.dropped_frames, 1)
        finally:
            node.destroy_node()

    def test_invalid_camera_info_dimensions_never_release_a_compressed_frame(self):
        node = self._node(transport='compressed')
        try:
            stamp_ns = 1_000_000_000
            node._sync_status_cb(String(data='READY'))
            image = self._header(CompressedImage(), stamp_ns)
            image.data = self._jpeg()
            node._payload_cb(image)
            node._camera_info_cb(self._camera_info(stamp_ns, width=0, height=480))
            node._sync_cb(self._sync(stamp_ns))

            self.assertEqual(node.published_frames, 0)
            self.assertGreaterEqual(node.dropped_frames, 1)
        finally:
            node.destroy_node()

    def test_jpeg_dimensions_must_match_camera_info(self):
        node = self._node(transport='compressed')
        try:
            stamp_ns = 1_000_000_000
            node._sync_status_cb(String(data='READY'))
            image = self._header(CompressedImage(), stamp_ns)
            image.data = self._jpeg(width=320, height=240)
            node._payload_cb(image)
            node._camera_info_cb(self._camera_info(stamp_ns, width=640, height=480))
            node._sync_cb(self._sync(stamp_ns))

            self.assertEqual(node.published_frames, 0)
            self.assertGreaterEqual(node.dropped_frames, 1)
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
