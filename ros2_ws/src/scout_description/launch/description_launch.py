"""Publish the Scout's URDF and TF (robot_state_publisher)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start robot_state_publisher with the xacro-expanded URDF."""
    urdf_path = os.path.join(get_package_share_directory('scout_description'),
                             'urdf', 'scout.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]),
                                       value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}],
             output='screen'),
    ])
