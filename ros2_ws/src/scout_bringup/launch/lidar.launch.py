"""Start the RPLIDAR C1 driver (sllidar_ros2) and the safety stop."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start sllidar_node (/scan in base_laser).

    and safety_stop (cmd_vel_in -> cmd_vel).
    """
    bringup = get_package_share_directory('scout_bringup')
    port = LaunchConfiguration('port')
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/scout_lidar',
            description='serial port of the C1 (CP2102 USB adapter)'),
        DeclareLaunchArgument(
            'safety',
            default_value='true',
            description='start safety_stop (teleop must publish /cmd_vel_in)'),
        Node(package='sllidar_ros2',
             executable='sllidar_node',
             name='sllidar_node',
             parameters=[{'channel_type': 'serial',
                          'serial_port': port,
                          'serial_baudrate': 460800,
                          'frame_id': 'base_laser',
                          'inverted': False,
                          'angle_compensate': True,
                          'scan_mode': 'Standard'}],
             output='screen'),
        Node(package='scout_localization',
             executable='safety_stop',
             name='safety_stop',
             parameters=[os.path.join(bringup, 'config', 'safety_stop.yaml')],
             condition=IfCondition(LaunchConfiguration('safety')),
             output='screen'),
    ])
