"""Launch the single AirSim ROS 2 wrapper used by all PX4 pipelines."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _argument(name, default_value, description):
    return DeclareLaunchArgument(name, default_value=default_value, description=description)


def generate_launch_description():
    airsim_launch = os.path.join(
        get_package_share_directory('airsim_ros_pkgs'), 'launch', 'airsim_node.launch.py'
    )
    arguments = [
        _argument('output', 'screen', 'Launch output destination.'),
        _argument('host_ip', 'localhost', 'AirSim RPC host.'),
        _argument('host_port', '41451', 'AirSim RPC port.'),
        _argument('rpc_timeout_sec', '5.0', 'Per-request AirSim RPC timeout in seconds.'),
        _argument(
            'update_airsim_img_response_every_n_sec',
            '0.05',
            'AirSim camera poll period in seconds; lower values request a higher frame rate.',
        ),
        _argument(
            'update_airsim_control_every_n_sec',
            '0.01',
            'AirSim state/HIL-history poll period in seconds.',
        ),
        _argument(
            'image_response_compress',
            'False',
            'Request and publish direct JPEG camera messages instead of raw Image messages.',
        ),
        _argument('enable_object_transforms_list', 'True', 'Publish AirSim object transforms.'),
        _argument(
            'world_frame_id',
            'world_enu',
            'Shared AirSim/PX4 world frame for camera TF and synchronized odometry.',
        ),
    ]
    wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(airsim_launch),
        launch_arguments={
            'output': LaunchConfiguration('output'),
            'host_ip': LaunchConfiguration('host_ip'),
            'host_port': LaunchConfiguration('host_port'),
            'rpc_timeout_sec': LaunchConfiguration('rpc_timeout_sec'),
            'update_airsim_img_response_every_n_sec': LaunchConfiguration(
                'update_airsim_img_response_every_n_sec'
            ),
            'update_airsim_control_every_n_sec': LaunchConfiguration(
                'update_airsim_control_every_n_sec'
            ),
            'image_response_compress': LaunchConfiguration('image_response_compress'),
            'publish_clock': 'False',
            'enable_api_control': 'False',
            'enable_object_transforms_list': LaunchConfiguration('enable_object_transforms_list'),
            'world_frame_id': LaunchConfiguration('world_frame_id'),
        }.items(),
    )
    return LaunchDescription(arguments + [wrapper])
