# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("handeye_calibration")

    default_params_file = PathJoinSubstitution(
        [pkg_share, "config", "handeye_params.yaml"]
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to YAML parameters file for handeye_calibration_node",
    )

    calib_node = Node(
        package="handeye_calibration",
        executable="handeye_calibration_node",
        name="handeye_calibration_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription(
        [
            params_file_arg,
            calib_node,
        ]
    )
