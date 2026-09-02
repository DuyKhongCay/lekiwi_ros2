# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch Gamepad Teleoperation subsystem for LeKiwi mobile base."""
    bringup_share = FindPackageShare("lekiwi_bringup")

    teleop_config_file = LaunchConfiguration("teleop_config_file")
    device_name = LaunchConfiguration("device_name")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declared_args_spec = [
        (
            "teleop_config_file",
            PathJoinSubstitution(
                [bringup_share, "config", "control", "gamepad_base_teleop.yaml"]
            ),
            "Path to gamepad teleoperation YAML configuration",
        ),
        ("device_name", "/dev/gamepad", "Linux joystick device path"),
        (
            "cmd_vel_topic",
            "/omni_base_controller/cmd_vel",
            "Target topic for base velocity commands",
        ),
        ("use_sim_time", "false", "Use simulation clock if true"),
    ]

    all_declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in declared_args_spec
    ]

    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        parameters=[
            teleop_config_file,
            {
                "dev": device_name,
                "use_sim_time": use_sim_time,
            },
        ],
        output="screen",
    )

    joy_teleop_node = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        parameters=[
            teleop_config_file,
            {
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=[
            ("/omni_base_controller/cmd_vel", cmd_vel_topic),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            *all_declared_arguments,
            joy_node,
            joy_teleop_node,
        ]
    )
