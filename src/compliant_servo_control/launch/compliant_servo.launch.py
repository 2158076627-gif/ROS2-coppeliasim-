#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_mock_force = LaunchConfiguration('use_mock_force')
    use_mock_force_arg = DeclareLaunchArgument(
        'use_mock_force',
        default_value='true',
        description='Publish synthetic force/torque samples for demos',
    )

    pkg = get_package_share_directory('compliant_servo_control')
    config = os.path.join(pkg, 'config', 'compliant_servo_params.yaml')

    return LaunchDescription([
        use_mock_force_arg,
        Node(
            package='compliant_servo_control',
            executable='mock_force_sensor_node',
            name='mock_force_sensor_node',
            output='screen',
            parameters=[config],
            condition=IfCondition(use_mock_force),
        ),
        Node(
            package='compliant_servo_control',
            executable='compliant_servo_node',
            name='compliant_servo_node',
            output='screen',
            parameters=[config],
        ),
    ])
