import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('px4_airsim_autonomy')
    default_params_file = os.path.join(pkg_share, 'config', 'autonomy_params.yaml')

    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='drone1',
        description='Vehicle name token in AirSim'
    )

    algorithm_arg = DeclareLaunchArgument(
        'algorithm',
        default_value='obstacle_avoidance',
        description='Active algorithm: obstacle_avoidance, scanning_patrol, target_guiding, area_search'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to YAML parameter file'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='cam1',
        description='Camera name token in AirSim (matches settings.json)'
    )

    enable_fmu_registration_arg = DeclareLaunchArgument(
        'enable_fmu_registration',
        default_value='true',
        description='Register mode with PX4 FMU (set to false for standalone simulation)'
    )

    autonomy_node = Node(
        package='px4_airsim_autonomy',
        executable='autonomous_flight_mode_node',
        name='autonomous_flight_mode_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'vehicle_name': LaunchConfiguration('vehicle_name'),
                'camera_name': LaunchConfiguration('camera_name'),
                'algorithm': LaunchConfiguration('algorithm'),
                'enable_fmu_registration': LaunchConfiguration('enable_fmu_registration'),
            }
        ]
    )

    return LaunchDescription([
        vehicle_name_arg,
        camera_name_arg,
        algorithm_arg,
        enable_fmu_registration_arg,
        params_file_arg,
        autonomy_node
    ])

