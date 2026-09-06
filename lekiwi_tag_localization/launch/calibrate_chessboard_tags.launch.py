# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Generates launch description for running chessboard tag calibration node.
def generate_launch_description():
    # Configures launch arguments and registers the chessboard tag calibrator node.
    pkg_share = get_package_share_directory("lekiwi_tag_localization")
    default_config_path = os.path.join(
        pkg_share, "config", "calibrate_chessboard_tags.yaml"
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_path,
        description="Path to calibrator YAML config file.",
    )

    calib_node = Node(
        package="lekiwi_tag_localization",
        executable="calibrate_chessboard_tags.py",
        name="chessboard_tag_calibrator",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription(
        [
            config_file_arg,
            calib_node,
        ]
    )
