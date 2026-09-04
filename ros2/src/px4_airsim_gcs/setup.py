import os
from glob import glob
from setuptools import setup

package_name = 'px4_airsim_gcs'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

extra_files = [(os.path.join('share', package_name, os.path.dirname(p)), [p]) 
               for p in package_files('static')]

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
] + extra_files

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tam Nguyen',
    maintainer_email='ngviettam82@users.noreply.github.com',
    description='Web Companion Ground Control Station for PX4 AirSim Autonomy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_gcs_node = px4_airsim_gcs.web_gcs_node:main',
        ],
    },
)

