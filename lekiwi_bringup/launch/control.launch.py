# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch LeKiwi control orchestration and optional LeRobot bridge helper."""
    bringup_share = FindPackageShare("lekiwi_bringup")

    joint_config_file = PathJoinSubstitution(
        [bringup_share, "config", "servos", "lekiwi_arm_calib.yaml"]
    )
    orchestrator_params_file = PathJoinSubstitution(
        [bringup_share, "config", "control", "orchestrator.yaml"]
    )

    start_lerobot_bridge_arg = DeclareLaunchArgument(
        "start_lerobot_bridge",
        default_value="false",
        description="Start LeRobot arm trajectory bridge node",
    )

    orchestrator = Node(
        package="lekiwi_control",
        executable="task_orchestrator",
        name="task_orchestrator",
        parameters=[orchestrator_params_file],
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

    return LaunchDescription(
        [
            start_lerobot_bridge_arg,
            orchestrator,
            arm_bridge,
        ]
    )
