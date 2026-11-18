"""Bring up the Scout perception: camera, tracker and detector."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """
    Start perception, camera, detector and.

    tracker.
    """
    perception = get_package_share_directory('scout_perception')
    params = os.path.join(perception, 'config', 'perception.yaml')

    model_path = LaunchConfiguration('model_path')
    imgsz = LaunchConfiguration('imgsz')
    debug_image = LaunchConfiguration('debug_image')
    use_tracking = LaunchConfiguration('use_tracking')

    camera = Node(package='camera_ros',
                  executable='camera_node',
                  parameters=[{
                      'camera': 0,
                      'width': 640,
                      'height': 480,
                      'format': 'RGB888',
                      'frame_id': 'camera_optical_frame'
                  }])

    tracker = Node(package='scout_perception',
                   executable='pan_tracker')

    detector = Node(package='scout_perception',
                    executable='person_detector',
                    parameters=[params, {
                        'model_path': model_path,
                        'imgsz': imgsz,
                        'debug_image': debug_image,
                        'use_tracking': use_tracking}])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/kleya/models/yolo26n_ncnn_model',
            description='detection model path'),
        DeclareLaunchArgument('imgsz', default_value='320',
                              description='image size'),
        DeclareLaunchArgument('debug_image', default_value='true',
                              description='display compressed image'),
        DeclareLaunchArgument('use_tracking', default_value='true',
                              description='enable tracking'),
        camera, tracker, detector
    ])
