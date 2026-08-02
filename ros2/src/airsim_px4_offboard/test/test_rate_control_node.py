import time
import unittest

import rclpy
from rclpy.parameter import Parameter

from airsim_interfaces.msg import Px4ImageSync, Px4RateSetpoint
from airsim_px4_offboard.control_safety import RateCommand
from airsim_px4_offboard.px4_rate_control import Px4RateControl
from px4_msgs.msg import SensorCombined, TimesyncStatus, VehicleStatus
from std_msgs.msg import String


class RateControlNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    @staticmethod
    def _node(*overrides):
        parameters = [
            Parameter(
                'expected_sync_image_topic',
                value='/airsim_node/drone1/cam1_Scene/image',
            ),
        ]
        parameters.extend(overrides)
        return Px4RateControl(parameter_overrides=parameters)

    def test_old_image_sync_cannot_cross_px4_clock_reset(self):
        node = self._node()
        try:
            node._control_mode_cb(String(data='PX4'))

            vehicle_status = VehicleStatus()
            vehicle_status.system_id = 1
            vehicle_status.hil_state = VehicleStatus.HIL_STATE_ON
            node._vehicle_status_cb(vehicle_status)

            old_clock = SensorCombined()
            old_clock.timestamp = 1_000_000
            node._px4_clock_cb(old_clock)

            reset_clock = SensorCombined()
            reset_clock.timestamp = 1_000
            node._px4_clock_cb(reset_clock)
            self.assertEqual(node._px4_clock_epoch, 1)

            queued_old_sync = Px4ImageSync()
            queued_old_sync.airsim_vehicle_name = 'drone1'
            queued_old_sync.px4_topic_prefix = '/fmu'
            queued_old_sync.px4_system_id = 1
            queued_old_sync.px4_clock_epoch = 0
            queued_old_sync.px4_clock_timestamp_us = 1_000_000
            queued_old_sync.image_topic = '/airsim_node/drone1/cam1_Scene/image'
            queued_old_sync.direct_hil_clock_verified = True
            queued_old_sync.direct_hil_clock_match_count = 3
            queued_old_sync.direct_hil_clock_last_matched_timestamp_us = 1_000_000
            queued_old_sync.image_header.stamp.sec = 1
            node._dds_timesync_guard_started_ns = (
                time.monotonic_ns() - node._px4_dds_timesync_guard_ns - 1
            )
            node._image_sync_cb(queued_old_sync)
            self.assertIsNone(node._latest_sync_stamp_ns)

            # A PX4 clock reset starts a new session, so a fresh vehicle-status
            # sample is also required before the controller can become ready.
            vehicle_status.timestamp = reset_clock.timestamp
            node._vehicle_status_cb(vehicle_status)

            command = Px4RateSetpoint()
            command.header.stamp.nanosec = 1_000_000
            command.thrust = 0.5
            node._setpoint_cb(command)
            node._publish_timer_cb()
            self.assertFalse(node._was_publishing)
            self.assertEqual(node._last_status, 'WAITING_FOR_IMAGE_SYNC')
        finally:
            node.destroy_node()

    def test_duplicate_clock_after_receipt_gap_starts_new_session(self):
        node = self._node()
        try:
            self.assertEqual(node._px4_session_gap_ns, 100_000_000)
            node._latest_px4_timestamp_us = 1_000_000
            node._last_px4_clock_receipt_ns = (
                time.monotonic_ns() - node._px4_session_gap_ns - 1
            )
            node._setpoint = RateCommand(0.0, 0.0, 0.0, 0.5, 1, 1)
            node._accepted_sync_stamps.append(1)

            duplicate_clock = SensorCombined()
            duplicate_clock.timestamp = 1_000_000
            node._px4_clock_cb(duplicate_clock)

            self.assertEqual(node._px4_clock_epoch, 1)
            self.assertEqual(node._latest_px4_timestamp_us, 1_000_000)
            self.assertEqual(node._session_start_px4_timestamp_us, 1_000_000)
            self.assertIsNone(node._setpoint)
            self.assertFalse(node._accepted_sync_stamps)
        finally:
            node.destroy_node()

    def test_dds_timestamp_synchronization_is_fatal_even_with_zero_estimate(self):
        node = self._node()
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
        node = self._node()
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
        node = self._node(
            Parameter('px4_vehicle_status_topic', value='/custom/vehicle_status')
        )
        try:
            self.assertEqual(node._px4_vehicle_status_topic, '/custom/vehicle_status')
        finally:
            node.destroy_node()

    def test_non_dds_timesync_status_does_not_trip_the_dds_guard(self):
        node = self._node()
        try:
            timesync = TimesyncStatus()
            timesync.source_protocol = TimesyncStatus.SOURCE_PROTOCOL_MAVLINK
            timesync.observed_offset = 1
            timesync.estimated_offset = 1
            node._timesync_status_cb(timesync)

            self.assertIsNone(node.fatal_error)
            self.assertFalse(node._dds_timesync_guard_verified)
        finally:
            node.destroy_node()

    def test_unproven_image_sync_cannot_activate_rate_control(self):
        node = self._node()
        try:
            node._control_mode_cb(String(data='PX4'))
            vehicle_status = VehicleStatus()
            vehicle_status.system_id = 1
            node._vehicle_status_cb(vehicle_status)

            px4_clock = SensorCombined()
            px4_clock.timestamp = 1_000_000
            node._px4_clock_cb(px4_clock)
            node._dds_timesync_guard_verified = True

            image_sync = Px4ImageSync()
            image_sync.airsim_vehicle_name = 'drone1'
            image_sync.px4_topic_prefix = '/fmu'
            image_sync.px4_system_id = 1
            image_sync.image_topic = '/airsim_node/drone1/cam1_Scene/image'
            image_sync.px4_clock_epoch = 0
            image_sync.px4_clock_timestamp_us = 1_000_000
            image_sync.image_header.stamp.nanosec = 900_000_000

            node._image_sync_cb(image_sync)
            self.assertIsNone(node._latest_sync_stamp_ns)

            image_sync.direct_hil_clock_verified = True
            image_sync.direct_hil_clock_match_count = 3
            image_sync.direct_hil_clock_last_matched_timestamp_us = 1_000_000
            node._image_sync_cb(image_sync)
            self.assertEqual(node._latest_sync_stamp_ns, 900_000_000)
        finally:
            node.destroy_node()

    def test_wrong_camera_sync_fails_closed(self):
        node = self._node()
        try:
            node._dds_timesync_guard_verified = True
            image_sync = Px4ImageSync()
            image_sync.image_topic = '/airsim_node/drone1/other_Scene/image'
            node._image_sync_cb(image_sync)

            self.assertIsNotNone(node.fatal_error)
            self.assertIn('camera does not match', node.fatal_error)
        finally:
            node.destroy_node()

    def test_px4_failsafe_fences_active_camera_authorized_control(self):
        node = self._node()
        try:
            node._setpoint = RateCommand(0.0, 0.0, 0.0, 0.5, 1, 1)
            node._latest_sync_stamp_ns = 1
            node._latest_sync_receipt_ns = time.monotonic_ns()
            node._accepted_sync_stamps.append(1)
            node._last_accepted_source_stamp_ns = 1

            failsafe = VehicleStatus()
            failsafe.system_id = 1
            failsafe.failsafe = True
            node._vehicle_status_cb(failsafe)

            self.assertTrue(node._px4_failsafe_active)
            self.assertIsNone(node._setpoint)
            self.assertIsNone(node._latest_sync_stamp_ns)
            self.assertFalse(node._accepted_sync_stamps)
            self.assertEqual(node._last_status, 'PX4_FAILSAFE')

            recovered = VehicleStatus()
            recovered.system_id = 1
            recovered.failsafe = False
            node._vehicle_status_cb(recovered)

            self.assertFalse(node._px4_failsafe_active)
            self.assertEqual(node._last_status, 'PX4_FAILSAFE_RECOVERED')
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
