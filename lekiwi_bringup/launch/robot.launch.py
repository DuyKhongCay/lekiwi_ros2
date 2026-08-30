# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Compose LeKiwi description, perception, and control subsystem launches."""
    bringup_share = FindPackageShare('lekiwi_bringup')
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'description.launch.py'])
        ),
        launch_arguments={
            'hardware_type': LaunchConfiguration('hardware_type'),
            'usb_port': LaunchConfiguration('usb_port'),
            'joint_config_file': LaunchConfiguration('joint_config_file'),
            'use_ros2_control': LaunchConfiguration('use_ros2_control'),
            'controllers_file': LaunchConfiguration('controllers_file'),
            'start_controller_manager': LaunchConfiguration('start_controller_manager'),
            'activate_controllers': LaunchConfiguration('activate_controllers'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_prefix': LaunchConfiguration('frame_prefix'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'ignore_timestamp': LaunchConfiguration('ignore_timestamp'),
        }.items(),
    )
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'cameras.launch.py'])
        ),
        launch_arguments={
            'container_name': LaunchConfiguration('data_plane_container'),
            'create_container': 'true',
            'cam_params_file': LaunchConfiguration('camera_params_file'),
            'use_test_sources': LaunchConfiguration('use_test_sources'),
        }.items(),
    )
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'control.launch.py'])
        ),
        launch_arguments={
            'joint_config_file': LaunchConfiguration('joint_config_file'),
            'orchestrator_params_file': LaunchConfiguration('orchestrator_params_file'),
            'start_lerobot_bridge': LaunchConfiguration('start_lerobot_bridge'),
            'run_camera_demo': LaunchConfiguration('run_camera_demo'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('hardware_type', default_value='mock'),
        DeclareLaunchArgument('usb_port', default_value='/dev/lekiwi_serial'),
        DeclareLaunchArgument(
            'joint_config_file',
            default_value=PathJoinSubstitution([
                bringup_share, 'config', 'hardware', 'lekiwi_joints.yaml'
            ]),
        ),
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution([
                bringup_share, 'config', 'controllers', 'lekiwi_controllers.yaml'
            ]),
        ),
        DeclareLaunchArgument('start_controller_manager', default_value='false'),
        DeclareLaunchArgument('activate_controllers', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('ignore_timestamp', default_value='false'),
        DeclareLaunchArgument('data_plane_container', default_value='lekiwi_perception_container'),
        DeclareLaunchArgument(
            'camera_params_file',
            default_value=PathJoinSubstitution([
                bringup_share, 'config', 'perception', 'cameras.yaml'
            ]),
        ),
        DeclareLaunchArgument('use_test_sources', default_value='false'),
        DeclareLaunchArgument(
            'orchestrator_params_file',
            default_value=PathJoinSubstitution([
                bringup_share, 'config', 'control', 'orchestrator.yaml'
            ]),
        ),
        DeclareLaunchArgument('start_lerobot_bridge', default_value='false'),
        DeclareLaunchArgument('run_camera_demo', default_value='false'),
        description,
        cameras,
        control,
    ])
