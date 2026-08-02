import unittest

from airsim_px4_offboard.px4_schema import validate_px4_message_contract


def _message(name, fields, version_marker=False, version=None):
    message_type = type(name, (), {
        'get_fields_and_field_types': lambda self: fields,
    })
    message = message_type()
    if version_marker:
        message.MESSAGE_VERSION = version
    return message


class Px4SchemaTest(unittest.TestCase):
    def test_exact_unversioned_layout_is_accepted(self):
        validate_px4_message_contract(_message('OffboardControlMode', {
            'timestamp': 'uint64',
            'position': 'boolean',
            'velocity': 'boolean',
            'acceleration': 'boolean',
            'attitude': 'boolean',
            'body_rate': 'boolean',
            'thrust_and_torque': 'boolean',
            'direct_actuator': 'boolean',
        }))

    def test_changed_field_type_and_version_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'field layout is unsupported'):
            validate_px4_message_contract(_message('VehicleRatesSetpoint', {
                'timestamp': 'uint64',
                'roll': 'double',
                'pitch': 'float',
                'yaw': 'float',
                'thrust_body': 'float[3]',
                'reset_integral': 'boolean',
            }, True, 0))
        with self.assertRaisesRegex(RuntimeError, 'MESSAGE_VERSION'):
            validate_px4_message_contract(_message('VehicleRatesSetpoint', {
                'timestamp': 'uint64',
                'roll': 'float',
                'pitch': 'float',
                'yaw': 'float',
                'thrust_body': 'float[3]',
                'reset_integral': 'boolean',
            }, True, 2))

    def test_timesync_status_layout_is_exactly_checked(self):
        fields = {
            'timestamp': 'uint64',
            'source_protocol': 'uint8',
            'remote_timestamp': 'uint64',
            'observed_offset': 'int64',
            'estimated_offset': 'int64',
            'round_trip_time': 'uint32',
        }
        validate_px4_message_contract(_message('TimesyncStatus', fields))

        incompatible_fields = dict(fields)
        incompatible_fields['observed_offset'] = 'uint64'
        with self.assertRaisesRegex(RuntimeError, 'field layout is unsupported'):
            validate_px4_message_contract(_message('TimesyncStatus', incompatible_fields))


if __name__ == '__main__':
    unittest.main()
