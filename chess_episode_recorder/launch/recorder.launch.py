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

    # 1. Declare Recorder & Bringup Launch Arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_config_path,
        description="Path to episode recorder YAML configuration file",
    )
    hardware_type_arg = DeclareLaunchArgument(
        "hardware_type",
        default_value="real",
        description="Hardware interface type: real or mock",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )
    uarm_teleop_arg = DeclareLaunchArgument(
        "uarm_teleop",
        default_value="true",
        description="Start uArm leader teleoperation node for follower arm recording",
    )
    orchestrator_arg = DeclareLaunchArgument(
        "orchestrator",
        default_value="true",
        description="Start LeKiwi orchestration subsystem",
    )
    manipulation_arg = DeclareLaunchArgument(
        "manipulation",
        default_value="false",
        description="Manipulation subsystem mode (kinematics, policy, or false; default false for recorder teleop)",
    )
    cameras_arg = DeclareLaunchArgument(
        "cameras",
        default_value="true",
        description="Start camera perception streams",
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
            "hardware_type": LaunchConfiguration("hardware_type"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "uarm_teleop": LaunchConfiguration("uarm_teleop"),
            "orchestrator": LaunchConfiguration("orchestrator"),
            "manipulation": LaunchConfiguration("manipulation"),
            "cameras": LaunchConfiguration("cameras"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    # 3. Define Episode Recorder Node
    recorder_node = Node(
        package="chess_episode_recorder",
        executable="episode_recorder_node",
        name="episode_recorder_node",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            params_file_arg,
            hardware_type_arg,
            use_sim_time_arg,
            uarm_teleop_arg,
            orchestrator_arg,
            manipulation_arg,
            cameras_arg,
            log_level_arg,
            robot_bringup,
            recorder_node,
        ]
    )
