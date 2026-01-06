#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('params_practice'),
        'config',
        'robot_config.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='params_practice',
            executable='basic_param_node',
            name='basic_param_node',
            output='screen',
            parameters=[config]
        )
    ])