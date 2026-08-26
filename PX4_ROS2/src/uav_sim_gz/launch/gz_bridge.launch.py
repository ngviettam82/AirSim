"""Bridge Gazebo sensor data and the simulation clock into ROS2.

Sim-world layer only, no real-drone counterpart; start after Gazebo is up.
New variant: add MODEL_IMAGE_TOPICS entry + matching config/bridge_<model>.yaml.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

# gz topic -> ROS name a real driver uses (R7).
MODEL_IMAGE_TOPICS = {
    'uav0': [],
    'uav0_nav': [
        ('/uav0/camera_down/image', '/uav/uav0/perception/down/image_raw'),
    ],
    'uav0_nav_indoor': [
        ('/uav0/camera_down/image', '/uav/uav0/perception/down/image_raw'),
    ],
    'uav0_full': [
        ('/uav0/camera_down/image', '/uav/uav0/perception/down/image_raw'),
        ('/uav0/camera_front/image', '/uav/uav0/perception/front/image_raw'),
        ('/uav0/depth_front/image', '/uav/uav0/perception/front/depth_image'),
    ],
    # S10: uav0_full with the front pair merged into one rgbd_camera. The ROS names
    # are identical to uav0_full -- above the bridge nothing may notice (R7).
    'uav0_full_rgbd': [
        ('/uav0/camera_down/image', '/uav/uav0/perception/down/image_raw'),
        ('/uav0/rgbd_front/image', '/uav/uav0/perception/front/image_raw'),
        ('/uav0/rgbd_front/depth_image', '/uav/uav0/perception/front/depth_image'),
    ],
    # P5.5 tracker gate: front only, keeps render sensor count at 2 (RTF).
    'uav0_track': [
        ('/uav0/camera_front/image', '/uav/uav0/perception/front/image_raw'),
        ('/uav0/depth_front/image', '/uav/uav0/perception/front/depth_image'),
    ],
}


def _bridge_nodes(context):
    model = LaunchConfiguration('model').perform(context)
    config = LaunchConfiguration('config').perform(context)

    if model not in MODEL_IMAGE_TOPICS:
        raise RuntimeError(
            f"unknown model '{model}'; known: {sorted(MODEL_IMAGE_TOPICS)}"
        )

    if not config:
        config = os.path.join(
            get_package_share_directory('uav_sim_gz'),
            'config', f'bridge_{model}.yaml',
        )
    if not os.path.exists(config):
        raise RuntimeError(f'bridge config not found: {config}')

    nodes = [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{
            'config_file': config,
            'use_sim_time': False,  # publishes /clock, cannot wait for it
        }],
    )]

    image_topics = MODEL_IMAGE_TOPICS[model]
    if LaunchConfiguration('images').perform(context).lower() in ('false', '0'):
        image_topics = []
    # One image_bridge per stream; shared bridge dropped frames (Debt #10).
    for index, (gz_topic, ros_topic) in enumerate(image_topics):
        nodes.append(Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='gz_image_bridge_%d' % index,
            output='screen',
            arguments=[gz_topic],
            remappings=[(gz_topic, ros_topic)],
            parameters=[{'use_sim_time': False}],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='uav0_nav',
            description='Model variant; selects image topics and bridge config.',
        ),
        DeclareLaunchArgument(
            'config',
            default_value='',
            description='Override the bridge config path. Empty picks it from model.',
        ),
        DeclareLaunchArgument(
            'images',
            default_value='true',
            description='Bridge image topics. Turn off for tests that do not read them.',
        ),
        OpaqueFunction(function=_bridge_nodes),
    ])
