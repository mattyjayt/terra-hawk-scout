"""Bring up the Scout base: bridge + robot model (+ Foxglove)."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Start base_bridge, robot_state_publisher and optionally.

    foxglove_bridge.
    """
    perception = get_package_share_directory('scout_perception')
    params = os.path.join(perception, 'config', 'perception.yaml')

    model_path = LaunchConfiguration('model_path')
    imgsz = LaunchConfiguration('imgsz')
    debug_image = LaunchConfiguration('debug_image')

    tracker = Node(package='scout_perception',
                   executable='pan_tracker')

    detector = Node(package='scout_perception',
                    executable='person_detector',
                    parameters=[params, {
                        'model_path': model_path,
                        'imgsz': imgsz,
                        'debug_image': debug_image}])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/kleya/models/yolo26n_ncnn_model',
            description='detection model path'),
        DeclareLaunchArgument('imgsz', default_value='320',
                              description='image size'),
        DeclareLaunchArgument('debug_image', default_value='true',
                              description='display compressed image'),
        tracker, detector
    ])
