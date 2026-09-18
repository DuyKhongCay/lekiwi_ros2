# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Launch file for LeKiwi control (workspace checker and reachability services)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch LeKiwi kinematics and workspace checking services."""
    start_workspace_checker_arg = DeclareLaunchArgument(
        "start_workspace_checker",
        default_value="true",
        description="Start URDF-based Workspace Checker node",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    workspace_checker = Node(
        package="lekiwi_control",
        executable="workspace_checker",
        name="workspace_checker",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_workspace_checker")),
    )

    return LaunchDescription(
        [
            start_workspace_checker_arg,
            use_sim_time_arg,
            workspace_checker,
        ]
    )
