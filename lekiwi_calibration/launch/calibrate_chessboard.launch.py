# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Launch file for running the chessboard AprilTag calibrator node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Configures launch arguments and registers the chessboard tag calibrator node."""
    pkg_share = FindPackageShare("lekiwi_calibration")
    default_config_path = PathJoinSubstitution(
        [pkg_share, "config", "chessboard_calib_params.yaml"]
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_path,
        description="Path to calibrator YAML config file.",
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run node in headless mode without GUI window.",
    )

    calib_node = Node(
        package="lekiwi_calibration",
        executable="calibrate_chessboard",
        name="chessboard_tag_calibrator",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"headless": LaunchConfiguration("headless")},
        ],
    )

    return LaunchDescription(
        [
            config_file_arg,
            headless_arg,
            calib_node,
        ]
    )
