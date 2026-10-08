# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Launch file for LeKiwi mission orchestration and readiness management.

Starts system readiness monitoring, workspace kinematics feasibility checks,
and arm manipulation action server inside a multi-threaded composable container,
alongside the central autonomous chess mission conductor node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Configure and launch orchestration container and mission conductor.

    Returns:
        LaunchDescription containing orchestrator container and subsystem nodes.
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
        default_value="kinematics",
        choices=["kinematics", "policy", "false"],
        description="Manipulation subsystem mode: "
        "'kinematics' (hardware trajectory controller), "
        "'policy' (LeRobot ACT/SmolVLA, accel=50), or 'false' (disabled)",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 3. Define multi-threaded composable container for C++ motion/supervision nodes
    orchestrator_container = ComposableNodeContainer(
        name="lekiwi_orchestrator_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="lekiwi_motion",
                plugin="lekiwi_motion::SystemReadinessNode",
                name="system_readiness_node",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="lekiwi_motion",
                plugin="lekiwi_motion::WorkspaceCheckerNode",
                name="workspace_checker",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    # 4. Conditionally load manipulation action server component
    load_manipulation_action = LoadComposableNodes(
        target_container=orchestrator_container,
        composable_node_descriptions=[
            ComposableNode(
                package="lekiwi_motion",
                plugin="lekiwi_motion::ManipulationActionServer",
                name="manipulation_action_server",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'", LaunchConfiguration("manipulation"), "' == 'kinematics'"
                ]
            )
        ),
    )

    # 5. Define Python chess mission conductor node (independent process)
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

    # 6. Assemble LaunchDescription
    return LaunchDescription(
        [
            params_file_arg,
            use_sim_time_arg,
            navigation_arg,
            manipulation_arg,
            orchestrator_container,
            load_manipulation_action,
            chess_mission_node,
        ]
    )
