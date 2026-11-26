"""SLAM: slam_toolbox online async mapping.

(or localization) with the Scout's settings.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Include slam_toolbox's online_async launch with config/slam.yaml."""
    bringup = get_package_share_directory('scout_bringup')
    slam_pkg = get_package_share_directory('slam_toolbox')
    params = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': params,
                          'use_sim_time': use_sim_time}.items())

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(bringup, 'config', 'slam.yaml'),
            description='slam_toolbox parameters'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        slam,
    ])
