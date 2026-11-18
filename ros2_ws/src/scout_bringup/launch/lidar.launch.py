"""Start the RPLIDAR C1 driver (sllidar_ros2) with the Scout's settings."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start sllidar_node publishing /scan in frame base_laser."""
    port = LaunchConfiguration('port')
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/scout_lidar',
            description='serial port of the C1 (CP2102 USB adapter)'),
        Node(package='sllidar_ros2',
             executable='sllidar_node',
             name='sllidar_node',
             parameters=[{'channel_type': 'serial',
                          'serial_port': port,
                          'serial_baudrate': 460800,
                          'frame_id': 'base_laser',
                          'inverted': False,
                          'angle_compensate': True,
                          'scan_mode': 'Standard'}]),
    ])
