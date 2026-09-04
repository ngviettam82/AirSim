from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='8080',
            description='Web GCS HTTP and WebSocket server port'
        ),
        DeclareLaunchArgument(
            'host',
            default_value='0.0.0.0',
            description='Web GCS server bind host IP'
        ),
        DeclareLaunchArgument(
            'vehicle_name',
            default_value='drone1',
            description='Vehicle name in AirSim / ROS 2'
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='cam1',
            description='Camera name in AirSim'
        ),
        DeclareLaunchArgument(
            'camera_host_port',
            default_value='8000',
            description='AirSim CameraHost streaming port'
        ),
        DeclareLaunchArgument(
            'airsim_ip',
            default_value='127.0.0.1',
            description='AirSim host IP for camera stream'
        ),
        Node(
            package='px4_airsim_gcs',
            executable='web_gcs_node',
            name='web_gcs_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'host': LaunchConfiguration('host'),
                'vehicle_name': LaunchConfiguration('vehicle_name'),
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_host_port': LaunchConfiguration('camera_host_port'),
                'airsim_ip': LaunchConfiguration('airsim_ip'),
            }]
        )
    ])

