import unittest

from airsim_px4_offboard.control_safety import (
    DEFAULT_PX4_SESSION_GAP_SEC,
    RateCommand,
    command_readiness,
    receipt_gap_exceeds,
    seconds_to_positive_nanoseconds,
    timestamp_progress,
    validate_rate_values,
    validate_source_timestamp,
)


class ControlSafetyTest(unittest.TestCase):
    def test_session_gap_default_and_duration_conversion(self):
        self.assertEqual(DEFAULT_PX4_SESSION_GAP_SEC, 0.1)
        self.assertEqual(
            seconds_to_positive_nanoseconds(
                DEFAULT_PX4_SESSION_GAP_SEC, 'px4_session_gap_sec'
            ),
            100_000_000,
        )
        with self.assertRaisesRegex(ValueError, 'at least one nanosecond'):
            seconds_to_positive_nanoseconds(0.0000000001, 'px4_session_gap_sec')

    def test_rate_validation_rejects_non_finite_and_out_of_bounds(self):
        self.assertEqual(
            validate_rate_values(1.0, -2.0, 0.5, 0.4, 6.0, 6.0, 3.0),
            (1.0, -2.0, 0.5, 0.4),
        )
        with self.assertRaisesRegex(ValueError, 'non_finite_setpoint'):
            validate_rate_values(float('nan'), 0.0, 0.0, 0.5, 6.0, 6.0, 3.0)
        with self.assertRaisesRegex(ValueError, 'setpoint_out_of_bounds'):
            validate_rate_values(7.0, 0.0, 0.0, 0.5, 6.0, 6.0, 3.0)

    def test_source_timestamp_rejects_missing_replay_and_future(self):
        validate_source_timestamp(1_000, None, 1_000, 100)
        with self.assertRaisesRegex(ValueError, 'missing_source_timestamp'):
            validate_source_timestamp(0, None, 1_000, 100)
        with self.assertRaisesRegex(ValueError, 'replayed_source_timestamp'):
            validate_source_timestamp(1_000, 1_000, 1_100, 100)
        with self.assertRaisesRegex(ValueError, 'source_timestamp_in_future'):
            validate_source_timestamp(1_201, 1_000, 1_100, 100)

    def test_timestamp_progress_does_not_refresh_duplicates(self):
        self.assertEqual(timestamp_progress(None, 10), 'first')
        self.assertEqual(timestamp_progress(10, 11), 'advanced')
        self.assertEqual(timestamp_progress(10, 10), 'duplicate')
        self.assertEqual(timestamp_progress(10, 9), 'regressed')

    def test_command_requires_fresh_transport_source_and_sync(self):
        command = RateCommand(0.0, 0.0, 0.0, 0.5, source_time_ns=900, receipt_ns=900)
        common = dict(
            command=command,
            now_receipt_ns=1_000,
            latest_px4_time_ns=1_000,
            latest_sync_time_ns=900,
            latest_sync_receipt_ns=950,
            accepted_sync_stamps=(900,),
            receipt_timeout_ns=200,
            source_timeout_ns=200,
            max_future_ns=20,
            sync_timeout_ns=200,
            require_image_sync=True,
        )
        self.assertEqual(command_readiness(**common), (True, 'ready'))

        stale_transport = dict(common, now_receipt_ns=1_101)
        self.assertEqual(
            command_readiness(**stale_transport),
            (False, 'stale_setpoint_transport'),
        )
        stale_source = dict(common, latest_px4_time_ns=1_101)
        self.assertEqual(command_readiness(**stale_source), (False, 'stale_setpoint_source'))
        stale_sync = dict(common, latest_sync_receipt_ns=799)
        self.assertEqual(command_readiness(**stale_sync), (False, 'stale_image_sync'))
        ahead = dict(common, latest_sync_time_ns=899)
        self.assertEqual(command_readiness(**ahead), (False, 'setpoint_ahead_of_image_sync'))
        unrelated = dict(common, accepted_sync_stamps=(899,))
        self.assertEqual(
            command_readiness(**unrelated),
            (False, 'setpoint_not_bound_to_image_sync'),
        )

    def test_receipt_gap_fences_a_new_px4_session(self):
        self.assertFalse(receipt_gap_exceeds(None, 1_000, 100))
        self.assertFalse(receipt_gap_exceeds(1_000, 1_100, 100))
        self.assertTrue(receipt_gap_exceeds(1_000, 1_101, 100))
        with self.assertRaisesRegex(ValueError, 'positive'):
            receipt_gap_exceeds(1_000, 1_100, 0)


if __name__ == '__main__':
    unittest.main()
