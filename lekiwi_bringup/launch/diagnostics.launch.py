# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""System diagnostics aggregation and resource monitoring bringup launch file.

Launches diagnostic_aggregator with LeKiwi analyzers, and optional CPU,
RAM, and hard disk resource monitors from diagnostic_common_diagnostics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Configure diagnostic aggregator and hardware resource monitor nodes.

    Returns:
        LaunchDescription containing diagnostic aggregator and system monitors.
    """
    # 1. Resolve analyzer configuration file path
    bringup_share = FindPackageShare("lekiwi_bringup")

    analyzers_config = PathJoinSubstitution(
        [bringup_share, "config", "diagnostics", "lekiwi_analyzers.yaml"]
    )

    # 2. Declare launch arguments
    enable_system_monitors_arg = DeclareLaunchArgument(
        "enable_system_monitors",
        default_value="true",
        description="Whether to run system monitors (CPU, RAM, Disk).",
    )

    # 3. Define diagnostic aggregator node
    aggregator_node = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="diagnostic_aggregator",
        output="screen",
        parameters=[analyzers_config],
    )

    # 4. Define conditional system resource monitor nodes (CPU, RAM, Disk)
    system_condition = IfCondition(LaunchConfiguration("enable_system_monitors"))

    cpu_monitor_node = Node(
        package="diagnostic_common_diagnostics",
        executable="cpu_monitor.py",
        name="cpu_monitor",
        output="screen",
        parameters=[analyzers_config],
        condition=system_condition,
    )

    ram_monitor_node = Node(
        package="diagnostic_common_diagnostics",
        executable="ram_monitor.py",
        name="ram_monitor",
        output="screen",
        parameters=[analyzers_config],
        condition=system_condition,
    )

    hd_monitor_node = Node(
        package="diagnostic_common_diagnostics",
        executable="hd_monitor.py",
        name="hd_monitor",
        output="screen",
        parameters=[analyzers_config],
        condition=system_condition,
    )

    # 5. Assemble LaunchDescription
    return LaunchDescription(
        [
            enable_system_monitors_arg,
            aggregator_node,
            cpu_monitor_node,
            ram_monitor_node,
            hd_monitor_node,
        ]
    )
