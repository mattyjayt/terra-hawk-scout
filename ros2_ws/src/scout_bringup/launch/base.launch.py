"""Bring up the Scout base: bridge + robot model (+ Foxglove)."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Start base_bridge, robot_state_publisher and optionally.

    foxglove_bridge.
    """
    bringup = get_package_share_directory('scout_bringup')
    description = get_package_share_directory('scout_description')
    perception = get_package_share_directory('scout_perception')
    params = os.path.join(bringup, 'config', 'base.yaml')

    impl = LaunchConfiguration('impl')
    port = LaunchConfiguration('port')
    foxglove = LaunchConfiguration('foxglove')

    bridge = Node(package=['scout_base_', impl],
                  executable='base_bridge',
                  parameters=[params, {'port': port}])

    urdf = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(description, 'launch', 'description.launch.py')))

    vision = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(perception, 'launch', 'perception.launch.py')))

    fox = Node(package='foxglove_bridge',
               executable='foxglove_bridge',
               parameters=[{'port': LaunchConfiguration('foxglove_port')}],
               condition=IfCondition(foxglove))

    return LaunchDescription([
        DeclareLaunchArgument('impl', default_value='py',
                              description='bridge implementation: py or cpp'),
        DeclareLaunchArgument('port', default_value='/dev/scout_base',
                              description='serial port of the R4'),
        DeclareLaunchArgument('foxglove', default_value='true',
                              description='start foxglove_bridge'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        bridge, urdf, fox, vision
    ])
