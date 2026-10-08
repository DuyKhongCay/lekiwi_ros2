# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Gamepad teleoperation bringup launch file for LeKiwi mobile base.

Starts joy_linux driver node for Linux joystick devices and joy_teleop
to map raw gamepad axes and buttons to cmd_vel velocity commands.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Configure joy_linux driver and joy_teleop velocity translation node.

    Returns:
        LaunchDescription containing gamepad teleoperation nodes.
    """
    # 1. Resolve package paths and configuration files
    bringup_share = FindPackageShare("lekiwi_bringup")
    teleop_config_file = PathJoinSubstitution(
        [bringup_share, "config", "control", "gamepad_base_teleop.yaml"]
    )

    # 2. Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock"
    )

    # 3. Define joy driver and joy_teleop nodes
    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        parameters=[
            teleop_config_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    joy_teleop_node = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        parameters=[
            teleop_config_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    # 4. Assemble LaunchDescription
    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_node,
            joy_teleop_node,
        ]
    )
