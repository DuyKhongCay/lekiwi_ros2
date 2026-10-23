# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Launch file for LeKiwi Episode Recorder with Master Robot Bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    recorder_share = FindPackageShare("chess_episode_recorder")
    bringup_share = FindPackageShare("lekiwi_bringup")

    default_config_path = PathJoinSubstitution(
        [recorder_share, "config", "episode_recorder_params.yaml"]
    )
    default_collection_config = PathJoinSubstitution(
        [recorder_share, "config", "collection_config.yaml"]
    )

    # 1. Declare Recorder & Bringup Launch Arguments
    params_file_arg = DeclareLaunchArgument(
        "recorder_params_file",
        default_value=default_config_path,
        description="Path to episode recorder YAML configuration file",
    )
    collection_config_arg = DeclareLaunchArgument(
        "collection_config_file",
        default_value=default_collection_config,
        description="Path to SmolVLA collection targets YAML configuration file",
    )
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="Global logging severity threshold",
    )

    # 2. Include Top-Level Robot Bringup
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "robot.launch.py"])
        ),
        launch_arguments={
            "hardware_type": "real",
            "use_sim_time": "false",
            "orchestrator": "true",
            "manipulation": "kinematics",
            "cameras": "true",
            "record": "true",
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    # 3. Define Episode Recorder Node
    recorder_node = Node(
        package="chess_episode_recorder",
        executable="episode_recorder_node",
        name="episode_recorder_node",
        parameters=[
            LaunchConfiguration("recorder_params_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "collection_config_path": LaunchConfiguration("collection_config_file"),
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            params_file_arg,
            collection_config_arg,
            use_sim_time,
            log_level_arg,
            robot_bringup,
            recorder_node,
        ]
    )
