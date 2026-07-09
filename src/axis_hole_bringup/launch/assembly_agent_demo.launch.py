#!/usr/bin/env python3
"""
assembly_agent_demo.launch.py

End-to-end demo for "ros2机械臂装配智能体":
1. stereo vision and laser point cloud mock publishers
2. 2D hole detection and 2D/3D fusion
3. assembly trajectory planning
4. force tactile feedback and compliant servo correction
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_mock_force = LaunchConfiguration('use_mock_force')

    pkg_bringup = get_package_share_directory('axis_hole_bringup')
    pkg_fusion = get_package_share_directory('axis_hole_fusion')
    pkg_traj = get_package_share_directory('assembly_trajectory_planner')
    pkg_servo = get_package_share_directory('compliant_servo_control')

    tf_config = os.path.join(pkg_fusion, 'config', 'tf_frames.yaml')
    fusion_config = os.path.join(pkg_fusion, 'config', 'fusion_params.yaml')
    trajectory_config = os.path.join(pkg_traj, 'config', 'trajectory_planner_params.yaml')
    servo_config = os.path.join(pkg_servo, 'config', 'compliant_servo_params.yaml')
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'axis_hole_perception.rviz')

    tf_broadcaster = Node(
        package='axis_hole_fusion',
        executable='tf_broadcaster_node',
        name='tf_broadcaster_node',
        output='screen',
        parameters=[{'config_path': tf_config}],
    )

    mock_image = Node(
        package='demo_data',
        executable='mock_image_publisher',
        name='mock_image_publisher',
        output='screen',
        parameters=[{'width': 1280, 'height': 720, 'rate_hz': 10.0, 'baseline_px': 120.0}],
    )

    mock_cloud = Node(
        package='demo_data',
        executable='mock_pointcloud_publisher',
        name='mock_pointcloud_publisher',
        output='screen',
        parameters=[{
            'rate_hz': 10.0,
            'num_points': 50000,
            'plane_width_m': 0.4,
            'plane_height_m': 0.3,
            'hole_radius_m': 0.015,
            'frame_id': 'laser_frame',
            'topic': '/laser/pointcloud_raw',
        }],
    )

    detector = Node(
        package='stereo_hole_detector',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        parameters=[{'config_path': ''}],
    )

    fusion = Node(
        package='axis_hole_fusion',
        executable='axis_hole_fusion_node',
        name='axis_hole_fusion_node',
        output='screen',
        parameters=[{'config_path': fusion_config}],
    )

    trajectory = Node(
        package='assembly_trajectory_planner',
        executable='trajectory_planner_node',
        name='trajectory_planner_node',
        output='screen',
        parameters=[trajectory_config],
    )

    mock_force = Node(
        package='compliant_servo_control',
        executable='mock_force_sensor_node',
        name='mock_force_sensor_node',
        output='screen',
        parameters=[servo_config],
        condition=IfCondition(use_mock_force),
    )

    servo = Node(
        package='compliant_servo_control',
        executable='compliant_servo_node',
        name='compliant_servo_node',
        output='screen',
        parameters=[servo_config],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false', description='Launch RViz2'),
        DeclareLaunchArgument('use_mock_force', default_value='true', description='Use synthetic force feedback'),
        SetParameter(name='use_sim_time', value=False),
        LogInfo(msg=''),
        LogInfo(msg='==============================================='),
        LogInfo(msg='ros2机械臂装配智能体 - Assembly Agent Demo'),
        LogInfo(msg='Modules: vision recognition | trajectory planning | compliant control'),
        LogInfo(msg='==============================================='),
        LogInfo(msg=''),
        tf_broadcaster,
        mock_image,
        mock_cloud,
        TimerAction(period=1.5, actions=[detector]),
        TimerAction(period=2.0, actions=[fusion]),
        TimerAction(period=2.5, actions=[trajectory]),
        TimerAction(period=3.0, actions=[mock_force, servo]),
        TimerAction(period=3.5, actions=[rviz]),
        LogInfo(msg='Monitor topics:'),
        LogInfo(msg='  /perception/axis_hole_pose'),
        LogInfo(msg='  /assembly/trajectory_plan'),
        LogInfo(msg='  /assembly/compliant_twist'),
        LogInfo(msg='  /assembly/contact_state'),
    ])
