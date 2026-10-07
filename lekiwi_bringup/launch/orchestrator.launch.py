# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Launch file for LeKiwi mission orchestration and readiness management.

Starts system readiness monitoring, workspace kinematics feasibility
checks, central autonomous chess mission conductor node, and arm manipulation subsystem
(manipulation_action_server or mock_policy_server).
"""

from launch.conditions import UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Configure and launch orchestration, readiness supervisor, mission conductor, and manipulation.

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
    navigation_arg = DeclareLaunchArgument(
        "navigation",
        default_value="true",
        description="Whether mobile base navigation (Nav2) is active",
    )
    
    manipulation_arg = DeclareLaunchArgument(
        "manipulation",
        default_value="mock",
        choices=["mock", "kinematics", "policy", "false"],
        description="Manipulation subsystem mode: 'mock' (simulated manipulation_action_server, default), "
        "'kinematics' (hardware trajectory controller), "
        "'policy' (LeRobot ACT/SmolVLA, accel=50), or 'false' (disabled)",
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
    )

    # 4. Define arm manipulation subsystem node
    manipulation_action_server_node = Node(
        package="lekiwi_motion",
        executable="manipulation_action_server",
        name="manipulation_action_server",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "mock_manipulation": PythonExpression(
                    ["'", LaunchConfiguration("manipulation"), "' == 'mock'"]
                ),
            },
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'", LaunchConfiguration("manipulation"), "' in ['mock', 'kinematics']"
                ]
            )
        ),
    )

    # 5. Assemble LaunchDescription
    return LaunchDescription(
        [
            params_file_arg,
            use_sim_time_arg,
            navigation_arg,
            manipulation_arg,
            system_readiness_node,
            workspace_checker_node,
            chess_mission_node,
            manipulation_action_server_node,
        ]
    )
