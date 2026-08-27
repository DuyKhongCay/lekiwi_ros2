# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch all currently implemented LeKiwi subsystems."""
    container_name = LaunchConfiguration('data_plane_container')
    use_test_sources = LaunchConfiguration('use_test_sources')
    run_demo = LaunchConfiguration('run_camera_demo')

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lekiwi_description'), 'launch', 'description.launch.py'
            ])
        ),
    )

    data_plane = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[],
        output='screen',
    )
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lekiwi_bringup'), 'launch', 'cameras.launch.py'
            ])
        ),
        launch_arguments={
            'container_name': container_name,
            'create_container': 'false',
            'cam_params_file': PathJoinSubstitution([
                FindPackageShare('lekiwi_bringup'), 'config', 'perception', 'cameras.yaml'
            ]),
            'use_test_sources': use_test_sources,
        }.items(),
    )
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lekiwi_bringup'), 'launch', 'control.launch.py'
            ])
        ),
        launch_arguments={
            'orchestrator_params_file': PathJoinSubstitution([
                FindPackageShare('lekiwi_bringup'), 'config', 'control', 'orchestrator.yaml'
            ]),
            'run_demo': run_demo,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'data_plane_container', default_value='lekiwi_perception_container'
        ),
        DeclareLaunchArgument('use_test_sources', default_value='false'),
        DeclareLaunchArgument('run_camera_demo', default_value='false'),
        description,
        data_plane,
        cameras,
        control,
    ])
