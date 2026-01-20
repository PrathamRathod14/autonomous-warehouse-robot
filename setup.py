from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_warehouse_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Required by ROS 2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pratham',
    maintainer_email='pratham@example.com',
    description='Autonomous warehouse delivery robot using ROS 2 Nav2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'warehouse_delivery = autonomous_warehouse_mission.warehouse_delivery_mission:main',
            'station_recorder = autonomous_warehouse_mission.station_recorder:main',
        ],
    },
)
