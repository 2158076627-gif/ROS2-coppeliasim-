#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('assembly_trajectory_planner')
    config = os.path.join(pkg, 'config', 'trajectory_planner_params.yaml')
    return LaunchDescription([
        Node(
            package='assembly_trajectory_planner',
            executable='trajectory_planner_node',
            name='trajectory_planner_node',
            output='screen',
            parameters=[config],
        ),
    ])
