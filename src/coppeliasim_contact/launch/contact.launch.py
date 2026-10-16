"""Separate physical backend. No mock plant or mock force source is launched."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration('params_file')
    default = os.path.join(get_package_share_directory('coppeliasim_contact'), 'config', 'physical.yaml')
    nodes = [Node(package=pkg, executable=name, name=name, output='screen', parameters=[config])
             for pkg, name in [('coppeliasim_contact','coppeliasim_bridge'),
                               ('axis_hole_bringup','mock_perception_node'),
                               ('assembly_trajectory_planner','trajectory_planner_node'),
                               ('compliant_servo_control','compliant_servo_node'),
                               ('axis_hole_bringup','demo_executor_node')]]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason='Required physical demo node exited'))])) for node in nodes]
    return LaunchDescription([DeclareLaunchArgument('params_file',default_value=default), *handlers, *nodes])
