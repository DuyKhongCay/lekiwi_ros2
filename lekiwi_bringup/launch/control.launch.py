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

    start_tf_gatekeeper_arg = DeclareLaunchArgument(
        "start_tf_gatekeeper",
        default_value="true",
        description="Start TF Tree Readiness Gatekeeper node",
    )

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

    orchestrator = Node(
        package="lekiwi_orchestrator",
        executable="task_orchestrator",
        name="task_orchestrator",
        parameters=[
            orchestrator_params_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    arm_bridge = Node(
        package="lekiwi_manipulation",
        executable="lerobot_arm_bridge",
        name="lerobot_arm_bridge",
        parameters=[
            {
                "joint_config_file": joint_config_file,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_lerobot_bridge")),
    )

    tf_gatekeeper = Node(
        package="lekiwi_orchestrator",
        executable="tf_gatekeeper",
        name="tf_readiness_gatekeeper",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_tf_gatekeeper")),
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
            start_lerobot_bridge_arg,
            start_tf_gatekeeper_arg,
            start_workspace_checker_arg,
            use_sim_time_arg,
            orchestrator,
            arm_bridge,
            tf_gatekeeper,
            workspace_checker,
        ]
    )
