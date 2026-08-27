# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the camera hub in a new or existing component container."""
    container_name = LaunchConfiguration('container_name')
    create_container = LaunchConfiguration('create_container')
    params_file = LaunchConfiguration('cam_params_file')
    use_test_sources = LaunchConfiguration('use_test_sources')

    camera_component = ComposableNode(
        package='lekiwi_perception',
        plugin='lekiwi_perception::MultiCameraHubComponent',
        name='multi_camera_hub',
        parameters=[params_file, {'use_test_sources': use_test_sources}],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[camera_component],
        output='screen',
        condition=IfCondition(create_container),
    )
    loader = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[camera_component],
        condition=UnlessCondition(create_container),
    )
    return LaunchDescription([
        DeclareLaunchArgument('container_name', default_value='lekiwi_perception_container'),
        DeclareLaunchArgument('create_container', default_value='true'),
        DeclareLaunchArgument(
            'cam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lekiwi_bringup'), 'config', 'perception', 'cameras.yaml'
            ]),
        ),
        DeclareLaunchArgument('use_test_sources', default_value='false'),
        container,
        loader,
    ])
