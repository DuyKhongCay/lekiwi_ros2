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
    start_readiness_arg = DeclareLaunchArgument(
        "start_readiness_manager",
        default_value="true",
        description="Whether to start system_readiness_node in orchestrator launch",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 1. System Readiness Manager (Nav & Grasp readiness, EKF convergence, arm joints)
    system_readiness_node = Node(
        package="lekiwi_motion",
        executable="system_readiness_node",
        name="system_readiness_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("start_readiness_manager")),
    )

    # 2. Workspace Kinematics Feasibility & Base Standoff Planner
    workspace_checker_node = Node(
        package="lekiwi_motion",
        executable="workspace_checker_node",
        name="workspace_checker",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # 3. Autonomous Chess Mission Conductor
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
            start_readiness_arg,
            system_readiness_node,
            workspace_checker_node,
            chess_mission_node,
        ]
    )
