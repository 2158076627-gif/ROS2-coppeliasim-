"""Six-node synthetic feedback loop. No absent perception or hardware packages."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration('params_file')
    default = os.path.join(get_package_share_directory('axis_hole_bringup'), 'config', 'demo.yaml')
    definitions = [
        ('axis_hole_bringup', 'mock_perception_node'),
        ('assembly_trajectory_planner', 'trajectory_planner_node'),
        ('axis_hole_bringup', 'mock_plant_node'),
        ('compliant_servo_control', 'mock_force_sensor_node'),
        ('compliant_servo_control', 'compliant_servo_node'),
        ('axis_hole_bringup', 'demo_executor_node')]
    nodes = [Node(package=pkg, executable=name, name=name, output='screen',
                  parameters=[config]) for pkg, name in definitions]
    # Subscribers wait for fresh inputs rather than relying on timer-based startup.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[EmitEvent(event=Shutdown(reason='Required demo node exited'))]))
        for node in nodes]
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        *handlers, *nodes,
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('use_rviz')))])
