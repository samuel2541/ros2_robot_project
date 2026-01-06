#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Lance le sensor manager avec configuration YAML.
    """
    
    # Argument pour le namespace (multi-robot)
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace du robot'
    )
    
    # Chemin vers le config
    config = os.path.join(
        get_package_share_directory('params_practice'),
        'config',
        'sensor_config.yaml'
    )
    
    return LaunchDescription([
        namespace_arg,
        
        Node(
            package='params_practice',
            executable='sensor_manager',
            name='sensor_manager',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[config]
        ),
    ])
