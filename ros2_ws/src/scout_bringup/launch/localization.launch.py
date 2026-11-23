"""Odometry without encoders: rf2o laser odometry + gyro, fused by an EKF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start rf2o, the covariance relay and the robot_localization EKF."""
    bringup = get_package_share_directory('scout_bringup')
    ekf_yaml = os.path.join(bringup, 'config', 'ekf.yaml')
    relay_yaml = os.path.join(bringup, 'config', 'covariance_relay.yaml')
    imu_topic = LaunchConfiguration('imu_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{'laser_scan_topic': '/scan',
                     'odom_topic': '/odom/rf2o',
                     'publish_tf': False,
                     'base_frame_id': 'base_link',
                     'odom_frame_id': 'odom',
                     'init_pose_from_topic': '',
                     'freq': 20.0,
                     'use_sim_time': use_sim_time}],
        ros_arguments=['--log-level', 'error'])

    relay = Node(
        package='scout_localization',
        executable='covariance_relay',
        name='covariance_relay',
        parameters=[relay_yaml, {'use_sim_time': use_sim_time}])

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_yaml, {'imu0': imu_topic,
                               'use_sim_time': use_sim_time}])

    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='gyro source: /imu/data_raw (robot) or /imu/fake'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        rf2o, relay, ekf,
    ])
