# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the task orchestrator and optional mode demo client."""
    params_file = LaunchConfiguration('orchestrator_params_file')
    run_demo = LaunchConfiguration('run_demo')
    orchestrator = Node(
        package='lekiwi_control',
        executable='task_orchestrator',
        name='task_orchestrator',
        parameters=[params_file],
        output='screen',
    )
    demo = Node(
        package='lekiwi_control',
        executable='camera_mode_demo',
        name='camera_mode_demo_client',
        output='screen',
        condition=IfCondition(run_demo),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'orchestrator_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lekiwi_bringup'), 'config', 'control', 'orchestrator.yaml'
            ]),
        ),
        DeclareLaunchArgument('run_demo', default_value='false'),
        orchestrator,
        demo,
    ])
