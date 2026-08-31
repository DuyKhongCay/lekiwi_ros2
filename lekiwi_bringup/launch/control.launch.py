# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch LeKiwi control orchestration and optional operator-facing helpers."""
    joint_config_file = LaunchConfiguration("joint_config_file")
    orchestrator = Node(
        package="lekiwi_control",
        executable="task_orchestrator",
        name="task_orchestrator",
        parameters=[LaunchConfiguration("orchestrator_params_file")],
        output="screen",
    )
    arm_bridge = Node(
        package="lekiwi_control",
        executable="lerobot_arm_bridge",
        name="lerobot_arm_bridge",
        parameters=[{"joint_config_file": joint_config_file}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_lerobot_bridge")),
    )
    camera_demo = Node(
        package="lekiwi_control",
        executable="camera_mode_demo",
        name="camera_mode_demo_client",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_camera_demo")),
    )

    bringup_share = FindPackageShare("lekiwi_bringup")

    declared_args_spec = [
        (
            "joint_config_file",
            PathJoinSubstitution(
                [bringup_share, "config", "hardware", "lekiwi_joints.yaml"]
            ),
        ),
        (
            "orchestrator_params_file",
            PathJoinSubstitution(
                [bringup_share, "config", "control", "orchestrator.yaml"]
            ),
        ),
        ("start_lerobot_bridge", "false"),
        ("run_camera_demo", "false"),
    ]

    all_declared_arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in declared_args_spec
    ]

    return LaunchDescription(
        [
            *all_declared_arguments,
            orchestrator,
            arm_bridge,
            camera_demo,
        ]
    )
