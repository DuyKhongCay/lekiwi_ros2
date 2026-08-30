# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch LeKiwi control orchestration and optional operator-facing helpers."""
    joint_config_file = LaunchConfiguration('joint_config_file')
    orchestrator = Node(
        package='lekiwi_control',
        executable='task_orchestrator',
        name='task_orchestrator',
        parameters=[LaunchConfiguration('orchestrator_params_file')],
        output='screen',
    )
    arm_bridge = Node(
        package='lekiwi_control',
        executable='lerobot_arm_bridge',
        name='lerobot_arm_bridge',
        parameters=[{'joint_config_file': joint_config_file}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_lerobot_bridge')),
    )
    camera_demo = Node(
        package='lekiwi_control',
        executable='camera_mode_demo',
        name='camera_mode_demo_client',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_camera_demo')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'joint_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lekiwi_bringup'), 'config', 'hardware', 'lekiwi_joints.yaml'
            ]),
        ),
        DeclareLaunchArgument(
            'orchestrator_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lekiwi_bringup'), 'config', 'control', 'orchestrator.yaml'
            ]),
        ),
        DeclareLaunchArgument('start_lerobot_bridge', default_value='false'),
        DeclareLaunchArgument('run_camera_demo', default_value='false'),
        orchestrator,
        arm_bridge,
        camera_demo,
    ])
