from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'airsim_px4_offboard'


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wouter Jansen',
    maintainer_email='wouter.jansen@uantwerpen.be',
    description='PX4-owned ROS 2 live control and AirSim camera synchronization.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_camera_sync = airsim_px4_offboard.px4_camera_sync:main',
            'px4_frame_gate = airsim_px4_offboard.px4_frame_gate:main',
            'px4_rate_control = airsim_px4_offboard.px4_rate_control:main',
        ],
    },
)
