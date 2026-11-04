"""
Start the three WayPilot nodes: navigator, mission_manager and web_bridge.

Usage:
  ros2 launch waypilot waypilot.launch.py [port:=8080] [use_sim_time:=false]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('waypilot'), 'config', 'waypilot.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    port = LaunchConfiguration('port')

    common = [params_file, {'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML file with node parameters'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the simulation clock'),
        DeclareLaunchArgument('port', default_value='8080',
                              description='HTTP port of the web UI'),

        Node(package='waypilot', executable='navigator', name='navigator',
             output='screen', parameters=common),
        Node(package='waypilot', executable='mission_manager', name='mission_manager',
             output='screen', parameters=common),
        Node(package='waypilot', executable='web_bridge', name='web_bridge',
             output='screen', parameters=common + [{'port': port}]),
    ])
