import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.parameter import Parameter

from airsim_interfaces.msg import Px4HilSensorClock
from airsim_px4_offboard.px4_camera_sync import PendingImage, Px4CameraSync
from px4_msgs.msg import SensorCombined, TimesyncStatus, VehicleStatus
from std_msgs.msg import Header


class CameraSyncNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    @staticmethod
    def _sensor_combined(timestamp_us):
        sample = SensorCombined()
        sample.timestamp = timestamp_us
        return sample

    @staticmethod
    def _hil_history(*timestamps_us):
        history = Px4HilSensorClock()
        history.airsim_vehicle_name = 'drone1'
        history.hil_sensor_timestamps_us = list(timestamps_us)
        return history

    def test_duplicate_clock_after_receipt_gap_starts_new_session(self):
        node = Px4CameraSync(parameter_overrides=[
            Parameter('camera_topic', value='/airsim_node/drone1/cam1_Scene/image'),
            Parameter('camera_info_topic', value='/airsim_node/drone1/cam1_Scene/camera_info'),
        ])
        try:
            self.assertEqual(node._px4_session_gap_ns, 100_000_000)
            node._latest_px4_timestamp_us = 1_000_000
            node._last_px4_clock_receipt_ns = (
                time.monotonic_ns() - node._px4_session_gap_ns - 1
            )
            node._direct_hil_clock_verified = True
            node._odom_samples.append(object())
            node._attitude_samples.append(object())
            node._last_received_image_stamp_ns = 1

            duplicate_clock = SensorCombined()
            duplicate_clock.timestamp = 1_000_000
            node._sensor_combined_cb(duplicate_clock)

            self.assertEqual(node._px4_clock_epoch, 1)
            self.assertEqual(node._latest_px4_timestamp_us, 1_000_000)
            self.assertEqual(node._session_start_px4_timestamp_us, 1_000_000)
            self.assertFalse(node._odom_samples)
            self.assertFalse(node._attitude_samples)
            self.assertIsNone(node._last_received_image_stamp_ns)
            self.assertFalse(node._direct_hil_clock_verified)
            self.assertFalse(node._recent_px4_clock_timestamps_us)
            self.assertFalse(node._dds_timesync_guard_verified)
        finally:
            node.destroy_node()

    def test_exact_hil_sensor_matches_verify_the_current_px4_session(self):
        node = Px4CameraSync(parameter_overrides=[
            Parameter('camera_topic', value='/airsim_node/drone1/cam1_Scene/image'),
            Parameter('camera_info_topic', value='/airsim_node/drone1/cam1_Scene/camera_info'),
        ])
        try:
            timestamps = (1_000_000, 1_003_000, 1_006_000)
            for timestamp_us in timestamps:
                node._sensor_combined_cb(self._sensor_combined(timestamp_us))
            node._hil_clock_cb(self._hil_history(*timestamps))

            self.assertTrue(node._direct_hil_clock_verified)
            self.assertEqual(node._direct_hil_clock_match_count, 3)
            self.assertEqual(node._last_matched_hil_timestamp_us, timestamps[-1])
        finally:
            node.destroy_node()

    def test_hil_history_mismatch_cannot_verify_the_clock(self):
        node = Px4CameraSync(parameter_overrides=[
            Parameter('camera_topic', value='/airsim_node/drone1/cam1_Scene/image'),
            Parameter('camera_info_topic', value='/airsim_node/drone1/cam1_Scene/camera_info'),
        ])
        try:
            for timestamp_us in (1_000_000, 1_003_000, 1_006_000):
                node._sensor_combined_cb(self._sensor_combined(timestamp_us))
            node._hil_clock_cb(self._hil_history(1_000_000, 1_006_000))

            self.assertFalse(node._direct_hil_clock_verified)
            # The middle PX4 sample is inside the AirSim history range but is
            # absent from it, so the final match begins a new one-sample streak.
            self.assertEqual(node._direct_hil_clock_match_count, 1)
        finally:
            node.destroy_node()

    def test_px4_session_reset_clears_direct_hil_proof_state(self):
        node = Px4CameraSync(parameter_overrides=[
            Parameter('camera_topic', value='/airsim_node/drone1/cam1_Scene/image'),
            Parameter('camera_info_topic', value='/airsim_node/drone1/cam1_Scene/camera_info'),
        ])
        try:
            node._direct_hil_clock_verified = True
            node._direct_hil_clock_match_count = 3
            node._last_matched_hil_timestamp_us = 1_006_000
            node._last_evaluated_px4_clock_timestamp_us = 1_006_000
            node._hil_sensor_timestamps_us = (1_000_000, 1_003_000, 1_006_000)
            node._last_hil_clock_receipt_ns = time.monotonic_ns()
            node._last_hil_clock_timestamp_us = 1_006_000
            node._recent_px4_clock_timestamps_us.extend((1_000_000, 1_003_000, 1_006_000))
            node._dds_timesync_guard_verified = True

            node._begin_px4_session('TEST_RESET', 'test reset')

            self.assertFalse(node._direct_hil_clock_verified)
            self.assertEqual(node._direct_hil_clock_match_count, 0)
            self.assertIsNone(node._last_matched_hil_timestamp_us)
            self.assertIsNone(node._last_evaluated_px4_clock_timestamp_us)
            self.assertFalse(node._hil_sensor_timestamps_us)
            self.assertFalse(node._recent_px4_clock_timestamps_us)
            self.assertFalse(node._dds_timesync_guard_verified)
        finally:
            node.destroy_node()

    def test_previous_session_state_cannot_enter_a_new_interpolation_history(self):
        node = Px4CameraSync(parameter_overrides=[
            Parameter('camera_topic', value='/airsim_node/drone1/cam1_Scene/image'),
            Parameter('camera_info_topic', value='/airsim_node/drone1/cam1_Scene/camera_info'),
        ])
        try:
            # A receipt-gap restart can retain a monotonic HIL clock. A late
            # old state is therefore behind (not ahead of) the current clock
            # and must be rejected explicitly at the session fence.
            node._latest_px4_timestamp_us = 2_000_000
            node._session_start_px4_timestamp_us = 2_000_000

            self.assertFalse(
                node._source_time_is_current(1_999_999, 'VehicleOdometry')
            )
            self.assertTrue(
                node._source_time_is_current(2_000_000, 'VehicleOdometry')
            )
            self.assertTrue(
                node._source_time_is_current(2_000_001, 'VehicleOdometry')
            )
        finally:
            node.destroy_node()

    def test_stale_hil_history_revokes_a_previously_verified_clock(self):
        node = Px4CameraSync(parameter_overrides=[
            Parameter('camera_topic', value='/airsim_node/drone1/cam1_Scene/image'),
            Parameter('camera_info_topic', value='/airsim_node/drone1/cam1_Scene/camera_info'),
        ])
        try:
            now_ns = time.monotonic_ns()
            node._direct_hil_clock_check_started_ns = now_ns - 1
            node._direct_hil_clock_verified = True
            node._direct_hil_clock_match_count = 3
            node._last_matched_hil_timestamp_us = 1_006_000
            node._latest_px4_timestamp_us = 1_006_000
            node._last_px4_clock_receipt_ns = now_ns
            node._last_hil_clock_timestamp_us = 1_006_000
            node._last_hil_clock_receipt_ns = now_ns - node._hil_clock_max_age_ns - 1
            node._dds_timesync_guard_verified = True

            self.assertFalse(node._direct_hil_clock_ready(now_ns))
            self.assertFalse(node._direct_hil_clock_verified)
            self.assertIsNone(node.fatal_error)
        finally:
            node.destroy_node()

    def test_dds_timestamp_synchronization_is_fatal_even_with_zero_estimate(self):
        node = Px4CameraSync(parameter_overrides=[
            Parameter('camera_topic', value='/airsim_node/drone1/cam1_Scene/image'),
            Parameter('camera_info_topic', value='/airsim_node/drone1/cam1_Scene/camera_info'),
        ])
        try:
            timesync = TimesyncStatus()
            timesync.source_protocol = TimesyncStatus.SOURCE_PROTOCOL_DDS
            timesync.observed_offset = 1
            timesync.estimated_offset = 0
            node._timesync_status_cb(timesync)

            self.assertIsNotNone(node.fatal_error)
            self.assertIn('UXRCE_DDS_SYNCT=0', node.fatal_error)
            self.assertFalse(node._dds_timesync_guard_verified)
        finally:
            node.destroy_node()

    def test_normal_sitl_hil_off_status_still_binds_vehicle_identity(self):
        node = Px4CameraSync(parameter_overrides=[
            Parameter('camera_topic', value='/airsim_node/drone1/cam1_Scene/image'),
            Parameter('camera_info_topic', value='/airsim_node/drone1/cam1_Scene/camera_info'),
        ])
        try:
            self.assertEqual(node._px4_vehicle_status_topic, '/fmu/out/vehicle_status_v1')
            vehicle_status = VehicleStatus()
            vehicle_status.system_id = 1
            vehicle_status.hil_state = VehicleStatus.HIL_STATE_OFF
            node._vehicle_status_cb(vehicle_status)

            self.assertEqual(node._validated_px4_system_id, 1)
            self.assertIsNone(node.fatal_error)
        finally:
            node.destroy_node()

    def test_custom_vehicle_status_topic_override_is_preserved(self):
        node = Px4CameraSync(parameter_overrides=[
            Parameter('camera_topic', value='/airsim_node/drone1/cam1_Scene/image'),
            Parameter('camera_info_topic', value='/airsim_node/drone1/cam1_Scene/camera_info'),
            Parameter('px4_vehicle_status_topic', value='/custom/vehicle_status'),
        ])
        try:
            self.assertEqual(node._px4_vehicle_status_topic, '/custom/vehicle_status')
        finally:
            node.destroy_node()

    def test_history_timestamp_index_preserves_order_and_reset_brackets(self):
        node = Px4CameraSync(parameter_overrides=[
            Parameter('camera_topic', value='/airsim_node/drone1/cam1_Scene/image'),
            Parameter('camera_info_topic', value='/airsim_node/drone1/cam1_Scene/camera_info'),
        ])
        try:
            def sample(timestamp_ns, reset_counter):
                return SimpleNamespace(
                    remote_time_ns=timestamp_ns,
                    reset_counter=reset_counter,
                )

            for timestamp_ns in (300, 100, 200):
                node._insert_sample(
                    node._odom_samples,
                    node._odom_sample_timestamps_ns,
                    sample(timestamp_ns, 0),
                    'odometry',
                )

            self.assertEqual(node._odom_sample_timestamps_ns, [100, 200, 300])
            bracket, reason = node._bracket(
                node._odom_samples,
                node._odom_sample_timestamps_ns,
                150,
                200,
                'reset_counter',
            )
            self.assertEqual(reason, 'ready')
            self.assertEqual(bracket.before.remote_time_ns, 100)
            self.assertEqual(bracket.after.remote_time_ns, 200)
            self.assertAlmostEqual(bracket.ratio, 0.5)

            node._insert_sample(
                node._odom_samples,
                node._odom_sample_timestamps_ns,
                sample(200, 1),
                'odometry',
            )
            bracket, reason = node._bracket(
                node._odom_samples,
                node._odom_sample_timestamps_ns,
                150,
                200,
                'reset_counter',
            )
            self.assertIsNone(bracket)
            self.assertEqual(reason, 'state_reset')
        finally:
            node.destroy_node()

    def test_pending_image_retries_only_when_awaited_history_advances(self):
        node = Px4CameraSync(parameter_overrides=[
            Parameter('camera_topic', value='/airsim_node/drone1/cam1_Scene/image'),
            Parameter('camera_info_topic', value='/airsim_node/drone1/cam1_Scene/camera_info'),
        ])
        try:
            node._validated_px4_system_id = 1
            node._last_vehicle_status_receipt_ns = time.monotonic_ns()
            node._direct_hil_clock_ready = lambda _now_ns: True
            node._pending_images.append(PendingImage(Header(), 1_000, time.monotonic_ns()))
            node._make_pair = Mock(return_value=(None, 'odometry_awaiting_future_sample'))

            node._process_pending_images()
            self.assertEqual(node._make_pair.call_count, 1)
            self.assertEqual(node._pending_images[0].retry_history, 'odometry')

            node._history_versions['gyro'] += 1
            node._process_pending_images()
            self.assertEqual(node._make_pair.call_count, 1)

            node._history_versions['odometry'] += 1
            node._process_pending_images()
            self.assertEqual(node._make_pair.call_count, 2)
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
