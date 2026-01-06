#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='robot_config.yaml',
        description='Name of config file to load'
    )
    
    config_path = [
        get_package_share_directory('params_practice'),
        '/config/',
        LaunchConfiguration('config')
    ]
    
    return LaunchDescription([
        config_arg,
        Node(
            package='params_practice',
            executable='basic_param_node',
            name='basic_param_node',
            output='screen',
            parameters=[config_path]
        )
    ])