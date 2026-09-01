# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch diagnostic aggregator and system resource monitors (CPU, RAM, Disk)."""
    bringup_share = FindPackageShare("lekiwi_bringup")

    analyzers_config_arg = DeclareLaunchArgument(
        "analyzers_config_file",
        default_value=PathJoinSubstitution(
            [bringup_share, "config", "diagnostics", "lekiwi_analyzers.yaml"]
        ),
        description="Path to diagnostics aggregator configuration YAML.",
    )

    enable_system_monitors_arg = DeclareLaunchArgument(
        "enable_system_monitors",
        default_value="true",
        description="Whether to run system monitors (CPU, RAM, Disk).",
    )

    cpu_warning_percentage_arg = DeclareLaunchArgument(
        "cpu_warning_percentage",
        default_value="90",
        description="CPU load percentage threshold to trigger diagnostic warning.",
    )

    ram_warning_percentage_arg = DeclareLaunchArgument(
        "ram_warning_percentage",
        default_value="90",
        description="RAM usage percentage threshold to trigger diagnostic warning.",
    )

    hd_path_arg = DeclareLaunchArgument(
        "hd_path",
        default_value="/",
        description="Mount path for hard drive usage monitoring.",
    )

    # Aggregator node
    aggregator_node = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="diagnostic_aggregator",
        output="screen",
        parameters=[LaunchConfiguration("analyzers_config_file")],
    )

    # System monitors from diagnostic_common_diagnostics
    system_condition = IfCondition(LaunchConfiguration("enable_system_monitors"))

    cpu_monitor_node = Node(
        package="diagnostic_common_diagnostics",
        executable="cpu_monitor.py",
        name="cpu_monitor",
        output="screen",
        parameters=[
            {
                "warning_percentage": LaunchConfiguration("cpu_warning_percentage"),
                "window": 1,
            }
        ],
        condition=system_condition,
    )

    ram_monitor_node = Node(
        package="diagnostic_common_diagnostics",
        executable="ram_monitor.py",
        name="ram_monitor",
        output="screen",
        parameters=[
            {
                "warning_percentage": LaunchConfiguration("ram_warning_percentage"),
                "window": 1,
            }
        ],
        condition=system_condition,
    )

    hd_monitor_node = Node(
        package="diagnostic_common_diagnostics",
        executable="hd_monitor.py",
        name="hd_monitor",
        output="screen",
        parameters=[
            {
                "path": LaunchConfiguration("hd_path"),
            }
        ],
        condition=system_condition,
    )

    return LaunchDescription(
        [
            analyzers_config_arg,
            enable_system_monitors_arg,
            cpu_warning_percentage_arg,
            ram_warning_percentage_arg,
            hd_path_arg,
            aggregator_node,
            cpu_monitor_node,
            ram_monitor_node,
            hd_monitor_node,
        ]
    )
