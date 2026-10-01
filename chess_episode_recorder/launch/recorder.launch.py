# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Launch file for LeKiwi Episode Recorder with Gamepad D-Pad controls."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("chess_episode_recorder")

    default_config_path = PathJoinSubstitution(
        [pkg_share, "config", "episode_recorder_params.yaml"]
    )

    # Launch Arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_config_path,
        description="Path to episode recorder YAML configuration file",
    )
    root_dir_arg = DeclareLaunchArgument(
        "root_dir",
        default_value="",
        description="Optional override for root storage directory",
    )
    task_arg = DeclareLaunchArgument(
        "task",
        default_value="",
        description="Optional override for task name",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Node parameter list
    node_params = [
        params_file,
        {"use_sim_time": use_sim_time},
    ]

    # Optional overrides from command line
    root_dir = LaunchConfiguration("root_dir")
    task = LaunchConfiguration("task")

    recorder_node = Node(
        package="chess_episode_recorder",
        executable="episode_recorder_node",
        name="episode_recorder_node",
        parameters=node_params,
        output="screen",
    )

    return LaunchDescription(
        [
            params_file_arg,
            root_dir_arg,
            task_arg,
            use_sim_time_arg,
            recorder_node,
        ]
    )
