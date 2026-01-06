#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Lance plusieurs robots avec des configurations différentes.
    """
    
    # Chemin vers le fichier config
    config = os.path.join(
        get_package_share_directory('params_practice'),
        'config',
        'multi_robot_config.yaml'
    )
    
    return LaunchDescription([
        # Robot Alpha
        Node(
            package='params_practice',
            executable='basic_param_node',
            name='basic_param_node',
            namespace='robot_alpha',
            output='screen',
            parameters=[config]
        ),
        
        # Robot Beta
        Node(
            package='params_practice',
            executable='basic_param_node',
            name='basic_param_node',
            namespace='robot_beta',
            output='screen',
            parameters=[config]
        ),
        
        # Robot Gamma
        Node(
            package='params_practice',
            executable='basic_param_node',
            name='basic_param_node',
            namespace='robot_gamma',
            output='screen',
            parameters=[config]
        ),
    ])