"""Nav2 for the Scout: map_server + AMCL + the navigation servers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start localization (map_server, amcl) and navigation.

    each with a lifecycle manager.
    """
    pkg = get_package_share_directory('scout_navigation')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg, 'maps', 'scout_map.yaml')

    params = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    common = [params, {'use_sim_time': use_sim_time}]
    args = ['--ros-args', '--log-level', log_level]
    tf_remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    localization_nodes = ['map_server', 'amcl']
    navigation_nodes = ['controller_server',
                        'planner_server', 'behavior_server',
                        'bt_navigator', 'waypoint_follower',
                        'velocity_smoother', 'collision_monitor']

    def nav2_node(package, executable, remappings=()):
        return Node(package=package, executable=executable, name=executable,
                    output='screen', parameters=common, arguments=args,
                    remappings=tf_remap + list(remappings))

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='saved map yaml (Lab 10)'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),

        # --- localization: the saved map and AMCL (map -> odom) ---
        Node(package='nav2_map_server', executable='map_server',
             name='map_server',
             output='screen', arguments=args, remappings=tf_remap,
             parameters=common + [{'yaml_filename': map_yaml}]),
        nav2_node('nav2_amcl', 'amcl'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', arguments=args,
             parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                          'node_names': localization_nodes}]),

        nav2_node('nav2_controller', 'controller_server',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower'),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_collision_monitor', 'collision_monitor'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', arguments=args,
             parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                          'node_names': navigation_nodes}]),
    ])
