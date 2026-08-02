import math
import unittest

from airsim_px4_offboard.sync_math import (
    FRD_TO_FLU,
    NED_TO_ENU,
    interpolation_bracket,
    latest_at_or_before_index,
    frd_to_flu,
    lerp_vector,
    matrix_multiply,
    matrix_to_quaternion,
    ned_to_enu,
    ned_velocity_covariance_to_flu_body,
    ned_velocity_to_flu_body,
    px4_us_to_airsim_ns,
    px4_attitude_ned_frd_to_enu_flu,
    quaternion_slerp,
    quaternion_to_matrix,
)


class SyncMathTest(unittest.TestCase):
    def assertMatrixAlmostEqual(self, actual, expected):
        for actual_row, expected_row in zip(actual, expected):
            for actual_value, expected_value in zip(actual_row, expected_row):
                self.assertAlmostEqual(actual_value, expected_value, places=7)

    def test_ned_and_frd_axes_convert_to_ros(self):
        self.assertEqual(ned_to_enu((1.0, 2.0, 3.0)), (2.0, 1.0, -3.0))
        self.assertEqual(frd_to_flu((1.0, 2.0, 3.0)), (1.0, -2.0, -3.0))

    def test_identity_px4_attitude_uses_both_world_and_body_frame_changes(self):
        converted = px4_attitude_ned_frd_to_enu_flu((1.0, 0.0, 0.0, 0.0))
        expected_rotation = matrix_multiply(NED_TO_ENU, FRD_TO_FLU)
        self.assertMatrixAlmostEqual(quaternion_to_matrix(converted), expected_rotation)

    def test_slerp_and_linear_interpolation_are_bounded(self):
        half_turn_about_x = (0.0, 1.0, 0.0, 0.0)
        midpoint = quaternion_slerp((1.0, 0.0, 0.0, 0.0), half_turn_about_x, 0.5)
        self.assertAlmostEqual(abs(midpoint[0]), math.sqrt(0.5), places=7)
        self.assertAlmostEqual(abs(midpoint[1]), math.sqrt(0.5), places=7)
        self.assertEqual(lerp_vector((0.0, 1.0, 2.0), (2.0, 3.0, 4.0), 0.5), (1.0, 2.0, 3.0))

    def test_airsim_hil_timestamp_maps_directly_to_image_clock(self):
        self.assertEqual(
            px4_us_to_airsim_ns(1_700_000_000_000_123),
            1_700_000_000_000_123_000,
        )

    def test_world_velocity_is_rotated_into_odometry_child_frame(self):
        self.assertEqual(
            ned_velocity_to_flu_body((1.0, 2.0, 3.0), (1.0, 0.0, 0.0, 0.0)),
            (1.0, -2.0, -3.0),
        )
        covariance = ned_velocity_covariance_to_flu_body(
            (1.0, 4.0, 9.0), (1.0, 0.0, 0.0, 0.0)
        )
        self.assertMatrixAlmostEqual(
            covariance,
            ((1.0, 0.0, 0.0), (0.0, 4.0, 0.0), (0.0, 0.0, 9.0)),
        )

    def test_interpolation_rejects_reset_boundaries(self):
        bracket, reason = interpolation_bracket(
            (1_000, 2_000), 1_500, 2_000, (3, 4)
        )
        self.assertIsNone(bracket)
        self.assertEqual(reason, 'state_reset')

        bracket, reason = interpolation_bracket(
            (1_000, 2_000), 1_500, 2_000, (4, 4)
        )
        self.assertEqual(reason, 'ready')
        self.assertEqual(bracket.before_index, 0)
        self.assertEqual(bracket.after_index, 1)
        self.assertAlmostEqual(bracket.ratio, 0.5)

    def test_causal_sample_selection_never_uses_future_data(self):
        timestamps = (1_000, 2_000, 3_000)
        self.assertIsNone(latest_at_or_before_index(timestamps, 999))
        self.assertEqual(latest_at_or_before_index(timestamps, 2_500), 1)
        self.assertEqual(latest_at_or_before_index(timestamps, 3_000), 2)


if __name__ == '__main__':
    unittest.main()
