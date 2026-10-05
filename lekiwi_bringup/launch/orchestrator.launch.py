# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Launch file for LeKiwi mission orchestration and readiness management.

Starts system readiness monitoring, workspace kinematics feasibility
checks, and the central autonomous chess mission conductor node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Configure and launch orchestration, readiness supervisor, and mission conductor.

    Returns:
        LaunchDescription containing orchestrator subsystem nodes.
    """
    # 1. Resolve configuration parameter paths
    bringup_share = FindPackageShare("lekiwi_bringup")
    default_params_file = PathJoinSubstitution(
        [bringup_share, "config", "control", "orchestrator.yaml"]
    )

    # 2. Declare launch arguments
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
    navigation_arg = DeclareLaunchArgument(
        "navigation",
        default_value="true",
        description="Whether mobile base navigation (Nav2) is active",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 3. Define system readiness, workspace, and mission conductor nodes
    system_readiness_node = Node(
        package="lekiwi_motion",
        executable="system_readiness_node",
        name="system_readiness_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("start_readiness_manager")),
    )

    workspace_checker_node = Node(
        package="lekiwi_motion",
        executable="workspace_checker_node",
        name="workspace_checker",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    chess_mission_node = Node(
        package="lekiwi_orchestrator",
        executable="chess_mission_orchestrator",
        name="chess_mission_orchestrator",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "navigation": LaunchConfiguration("navigation"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_mission")),
    )

    # 4. Assemble LaunchDescription
    return LaunchDescription(
        [
            params_file_arg,
            use_sim_time_arg,
            start_mission_arg,
            start_readiness_arg,
            navigation_arg,
            system_readiness_node,
            workspace_checker_node,
            chess_mission_node,
        ]
    )
