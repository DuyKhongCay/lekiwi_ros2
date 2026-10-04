# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unified launch script for LeKiwi manipulation subsystem.

Supports switching between Real Hardware Manipulation Action Server and Mock Policy Server.
Always launches Cartesian & Named Pose Service Node for deterministic arm control.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("lekiwi_manipulation")

    default_params_file = PathJoinSubstitution(
        [pkg_share, "config", "manipulation_params.yaml"]
    )

    # Launch Arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to manipulation parameters YAML configuration file",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    use_mock_arg = DeclareLaunchArgument(
        "use_mock",
        default_value="true",
        description="If true, runs mock_policy_server instead of real hardware manipulation_action_server",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock = LaunchConfiguration("use_mock")

    # 1. Cartesian & Named Pose Service Node (Always active for home, stow, ready, etc.)
    cartesian_service_node = Node(
        package="lekiwi_manipulation",
        executable="cartesian_service_node",
        name="cartesian_service_node",
        condition=UnlessCondition(use_mock),
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # 2. Real Hardware Manipulation Action Server (Default: use_mock=false)
    real_action_server_node = Node(
        package="lekiwi_manipulation",
        executable="manipulation_action_server",
        name="manipulation_action_server",
        condition=UnlessCondition(use_mock),
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # 3. Mock Policy Server (Active when use_mock=true)
    mock_server_node = Node(
        package="lekiwi_manipulation",
        executable="mock_policy_server",
        name="mock_policy_server",
        condition=IfCondition(use_mock),
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            params_file_arg,
            use_sim_time_arg,
            use_mock_arg,
            cartesian_service_node,
            real_action_server_node,
            mock_server_node,
        ]
    )
