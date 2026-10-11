import os
from glob import glob

from setuptools import setup

package_name = 'drone_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    description='Drone camera car detection and following',

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'car_detector = drone_tracker.car_detector:main',
            'drone_follow_controller = drone_tracker.drone_follow_controller:main',
            'gimbal_controller = drone_tracker.gimbal_controller:main',
            'autonomous_gimbal_controller = drone_tracker.autonomous_gimbal_controller:main',
            'gimbal_car_detector = drone_tracker.gimbal_car_detector:main',
        ],
    },
)