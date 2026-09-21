# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Launch high-level LeKiwi autonomous chess mission orchestrator."""

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

    # Launch Arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to orchestrator configuration parameters YAML file",
    )
    start_mission_arg = DeclareLaunchArgument(
        "start_mission_orchestrator",
        default_value="true",
        description="Whether to start the autonomous chess mission orchestrator",
    )
    start_task_orch_arg = DeclareLaunchArgument(
        "start_task_orchestrator",
        default_value="true",
        description="Whether to start the camera mode and lifecycle task orchestrator",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Nodes
    chess_mission_node = Node(
        package="lekiwi_orchestrator",
        executable="chess_mission_orchestrator",
        name="chess_mission_orchestrator",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_mission_orchestrator")),
    )

    task_orchestrator_node = Node(
        package="lekiwi_orchestrator",
        executable="task_orchestrator",
        name="task_orchestrator",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_task_orchestrator")),
    )

    return LaunchDescription(
        [
            params_file_arg,
            start_mission_arg,
            start_task_orch_arg,
            use_sim_time_arg,
            chess_mission_node,
            task_orchestrator_node,
        ]
    )
