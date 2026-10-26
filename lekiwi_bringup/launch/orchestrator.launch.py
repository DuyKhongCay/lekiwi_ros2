# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Launch file for LeKiwi mission orchestration and readiness management.

Starts system readiness monitoring, workspace kinematics feasibility checks,
arm manipulation action server, and episode recorder inside a unified
multi-threaded composable container, alongside the central autonomous chess
mission conductor node and policy inference client.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
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
    inference_client_params_file = PathJoinSubstitution(
        [bringup_share, "config", "control", "async_inference_cilent.yaml"]
    )
    default_recorder_params_file = PathJoinSubstitution(
        [bringup_share, "config", "control", "episode_recorder.yaml"]
    )

    # 2. Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to orchestrator configuration parameters YAML file",
    )
    recorder_params_file_arg = DeclareLaunchArgument(
        "recorder_params_file",
        default_value=default_recorder_params_file,
        description="Path to episode recorder configuration parameters YAML file",
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
        default_value="policy",
        choices=["kinematics", "policy", "false"],
        description="Manipulation subsystem mode: "
        "'kinematics' (hardware trajectory controller), "
        "'policy' (LeRobot ACT/SmolVLA, accel=50), or 'false' (disabled)",
    )
    record_arg = DeclareLaunchArgument(
        "record",
        default_value="false",
        description="Trigger record mode of manipulation action server and episode recorder.",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 3. Configure composable motion, supervision, and recorder components
    system_readiness_component = ComposableNode(
        package="lekiwi_motion",
        plugin="lekiwi_motion::SystemReadinessNode",
        name="system_readiness_node",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    workspace_checker_component = ComposableNode(
        package="lekiwi_motion",
        plugin="lekiwi_motion::WorkspaceCheckerNode",
        name="workspace_checker",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    manipulation_action_component = ComposableNode(
        package="lekiwi_motion",
        plugin="lekiwi_motion::ManipulationActionServer",
        name="manipulation_action_server",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "record": LaunchConfiguration("record"),
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
        condition=IfCondition(
            PythonExpression(
                [
                    "'", LaunchConfiguration("manipulation"), "' == 'kinematics'"
                ]
            )
        ),
    )

    episode_recorder_component = ComposableNode(
        package="chess_episode_recorder",
        plugin="chess_episode_recorder::EpisodeRecorderNode",
        name="episode_recorder_node",
        parameters=[
            LaunchConfiguration("recorder_params_file"),
            {
                "use_sim_time": use_sim_time,
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
        condition=IfCondition(
            PythonExpression(
                [
                    "'", LaunchConfiguration("record"), "'.lower() in ['true', '1']"
                ]
            )
        ),
    )

    all_orchestrator_components = [
        system_readiness_component,
        workspace_checker_component,
        manipulation_action_component,
        episode_recorder_component,
    ]

    # 4. Assemble multi-threaded composable component container
    orchestrator_container = ComposableNodeContainer(
        name="lekiwi_orchestrator_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=all_orchestrator_components,
        output="screen",
    )

    # 5. Conditionally start async policy inference client node (Phase 1: Async ZeroMQ)
    async_inference_client_node = Node(
        package="lekiwi_inference",
        executable="async_ros2_client_node",
        name="async_ros2_inference_client",
        output="screen",
        parameters=[
            params_file,
            inference_client_params_file,
            {
                "use_sim_time": use_sim_time,
            },
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'", LaunchConfiguration("manipulation"), "' == 'policy'"
                ]
            )
        ),
    )

    # 6. Define Python chess mission conductor node (independent process)
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
                "record": LaunchConfiguration("record"),
            },
        ],
    )

    # 7. Assemble LaunchDescription
    return LaunchDescription(
        [
            params_file_arg,
            recorder_params_file_arg,
            use_sim_time_arg,
            navigation_arg,
            manipulation_arg,
            record_arg,
            orchestrator_container,
            async_inference_client_node,
            chess_mission_node,
        ]
    )
