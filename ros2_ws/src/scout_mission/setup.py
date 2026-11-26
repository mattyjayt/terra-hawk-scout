from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'scout_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jaime Tellie',
    maintainer_email='jaime.tellie@gmail.com',
    description='Delivery mission state machine & Nav2 helpers',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'save_waypoint = scout_mission.save_waypoint:main',
            'go_to = scout_mission.go_to:main',
        ],
    },
)
