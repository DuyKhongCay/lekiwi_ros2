# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Launch LeKiwi orchestration and readiness subsystem (TF gatekeeper, workspace checker, orchestrators)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("lekiwi_orchestrator")
    default_params_file = PathJoinSubstitution(
        [pkg_share, "config", "orchestrator_params.yaml"]
    )

    # Minimal CLI Arguments - specific node settings are loaded from orchestrator_params.yaml
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to orchestrator configuration parameters YAML file",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    start_mission_arg = DeclareLaunchArgument(
        "start_mission",
        default_value="true",
        description="Whether to start the autonomous chess mission orchestrator",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 1. System Readiness Gatekeeper (TF tree, arm joints freshness, standstill, EKF convergence)
    tf_gatekeeper_node = Node(
        package="lekiwi_motion",
        executable="tf_gatekeeper_node",
        name="tf_gatekeeper_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("odometry/filtered", "/odometry/local"),
        ],
    )

    # 2. Workspace Kinematics Feasibility & Base Standoff Planner
    workspace_checker_node = Node(
        package="lekiwi_motion",
        executable="workspace_checker_node",
        name="workspace_checker",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # 3. Camera Mode & Lifecycle Task Orchestrator
    task_orchestrator_node = Node(
        package="lekiwi_orchestrator",
        executable="task_orchestrator",
        name="task_orchestrator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # 4. Autonomous Chess Mission Conductor
    chess_mission_node = Node(
        package="lekiwi_orchestrator",
        executable="chess_mission_orchestrator",
        name="chess_mission_orchestrator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("start_mission")),
    )

    return LaunchDescription(
        [
            params_file_arg,
            use_sim_time_arg,
            start_mission_arg,
            tf_gatekeeper_node,
            workspace_checker_node,
            task_orchestrator_node,
            chess_mission_node,
        ]
    )
