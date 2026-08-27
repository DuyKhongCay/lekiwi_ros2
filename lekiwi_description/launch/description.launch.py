# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Publish the LeKiwi URDF and TF tree."""
    package_share = Path(get_package_share_directory('lekiwi_description'))
    urdf_path = package_share / 'urdf' / 'duykhongcay_lekiwi.urdf'
    robot_description = urdf_path.read_text()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('ignore_timestamp', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'frame_prefix': ParameterValue(
                    LaunchConfiguration('frame_prefix'), value_type=str),
                'ignore_timestamp': ParameterValue(
                    LaunchConfiguration('ignore_timestamp'), value_type=bool),
                'publish_frequency': 50.0,
            }],
            remappings=[('joint_states', LaunchConfiguration('joint_states_topic'))],
        ),
    ])
