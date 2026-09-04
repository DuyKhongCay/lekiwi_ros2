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

    analyzers_config = PathJoinSubstitution(
        [bringup_share, "config", "diagnostics", "lekiwi_analyzers.yaml"]
    )

    enable_system_monitors_arg = DeclareLaunchArgument(
        "enable_system_monitors",
        default_value="true",
        description="Whether to run system monitors (CPU, RAM, Disk).",
    )

    aggregator_node = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="diagnostic_aggregator",
        output="screen",
        parameters=[analyzers_config],
    )

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

    return LaunchDescription(
        [
            enable_system_monitors_arg,
            aggregator_node,
            cpu_monitor_node,
            ram_monitor_node,
            hd_monitor_node,
        ]
    )
