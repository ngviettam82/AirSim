import unittest

from airsim_px4_offboard.launch_helpers import (
    camera_info_topic,
    camera_pipelines,
    camera_vehicle_name,
    join_topic_prefix,
    parse_bool,
    primary_camera_topic,
    primary_gated_sync_topic,
    primary_sync_topic,
    resolve_px4_vehicle_status_topic,
    validate_frame_gate_transport,
    validate_live_control_topology,
    validate_camera_vehicle,
)


class LaunchHelpersTest(unittest.TestCase):
    def test_multiple_cameras_receive_unique_default_prefixes(self):
        pipelines = camera_pipelines('/camera/front;/camera/down', '')
        self.assertEqual(
            pipelines,
            [('/camera/front', 'camera_0_sync'), ('/camera/down', 'camera_1_sync')],
        )
        self.assertEqual(primary_sync_topic(pipelines, '1'), 'camera_1_sync/image_sync')
        self.assertEqual(primary_camera_topic(pipelines, '1'), '/camera/down')
        self.assertEqual(
            primary_gated_sync_topic(pipelines, '1'), 'camera_1_sync_gated/image_sync'
        )

    def test_invalid_camera_prefix_topology_is_rejected(self):
        with self.assertRaises(ValueError):
            camera_pipelines('/camera/front;/camera/down', 'same;same')
        with self.assertRaises(ValueError):
            camera_pipelines('/camera/front;/camera/down', 'only_one')
        with self.assertRaisesRegex(ValueError, 'trailing-slash normalization'):
            camera_pipelines('/camera/front;/camera/down', 'front;front/')
        with self.assertRaisesRegex(ValueError, 'generated frame-gate prefix'):
            camera_pipelines(
                '/camera/front;/camera/down',
                'front;front_gated',
                reserve_gated_prefixes=True,
            )
        with self.assertRaisesRegex(ValueError, 'must be relative'):
            camera_pipelines(
                '/camera/front;/camera/down',
                'front;/drone1/front',
                reserve_gated_prefixes=True,
            )

    def test_camera_authorized_rate_control_requires_a_frame_gate(self):
        with self.assertRaisesRegex(ValueError, 'start_frame_gates must be true'):
            validate_live_control_topology(
                start_rate_control=True,
                require_image_sync=True,
                start_frame_gates=False,
            )

        validate_live_control_topology(
            start_rate_control=False,
            require_image_sync=True,
            start_frame_gates=False,
        )
        validate_live_control_topology(
            start_rate_control=True,
            require_image_sync=False,
            start_frame_gates=False,
        )
        validate_live_control_topology(
            start_rate_control=True,
            require_image_sync=True,
            start_frame_gates=True,
        )

    def test_topic_prefix_and_boolean_parsing(self):
        self.assertEqual(join_topic_prefix('/drone1/fmu/', '/out/sensor_combined'), '/drone1/fmu/out/sensor_combined')
        self.assertEqual(
            resolve_px4_vehicle_status_topic('/drone1/fmu/', ''),
            '/drone1/fmu/out/vehicle_status_v1',
        )
        self.assertEqual(
            resolve_px4_vehicle_status_topic('/drone1/fmu', '', '_v12'),
            '/drone1/fmu/out/vehicle_status_v12',
        )
        self.assertEqual(
            resolve_px4_vehicle_status_topic('/drone1/fmu', '', ''),
            '/drone1/fmu/out/vehicle_status',
        )
        self.assertEqual(
            resolve_px4_vehicle_status_topic(
                '/drone1/fmu', '/custom/vehicle_status', '_v12'
            ),
            '/custom/vehicle_status',
        )
        with self.assertRaisesRegex(ValueError, 'px4_vehicle_status_suffix'):
            resolve_px4_vehicle_status_topic('/drone1/fmu', '', 'vehicle_status_v1')
        self.assertEqual(
            camera_info_topic('/airsim_node/drone1/front_Scene/image'),
            '/airsim_node/drone1/front_Scene/camera_info',
        )
        self.assertEqual(
            camera_info_topic('/airsim_node/drone1/front_Scene/image/compressed'),
            '/airsim_node/drone1/front_Scene/camera_info',
        )
        self.assertEqual(
            camera_vehicle_name('/airsim_node/drone1/front_Scene/image/compressed'),
            'drone1',
        )
        validate_camera_vehicle('/airsim_node/drone1/front_Scene/image', 'drone1')
        with self.assertRaisesRegex(ValueError, 'belongs to AirSim vehicle'):
            validate_camera_vehicle('/airsim_node/drone2/front_Scene/image', 'drone1')
        with self.assertRaises(ValueError):
            camera_info_topic('/airsim_node/drone1/front_Scene')
        self.assertTrue(parse_bool('true'))
        self.assertFalse(parse_bool('Off'))
        with self.assertRaises(ValueError):
            parse_bool('maybe')
        with self.assertRaisesRegex(ValueError, 'direct JPEG only'):
            validate_frame_gate_transport(
                image_response_compress=True,
                start_frame_gates=True,
                frame_gate_image_transport='raw',
            )
        validate_frame_gate_transport(
            image_response_compress=True,
            start_frame_gates=True,
            frame_gate_image_transport='compressed',
        )


if __name__ == '__main__':
    unittest.main()
