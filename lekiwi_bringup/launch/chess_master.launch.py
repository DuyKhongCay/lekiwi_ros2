# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch LeKiwi Chess Master subsystem as ROS 2 composable nodes in a multi-threaded container."""
    bringup_share = FindPackageShare("lekiwi_bringup")

    default_config = PathJoinSubstitution(
        [bringup_share, "config", "chess", "chess_master_params.yaml"]
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to chess master config YAML file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    container = ComposableNodeContainer(
        name="chess_master_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="lekiwi_chess_master",
                plugin="lekiwi_chess_master::ChessGameStateTrackerComponent",
                name="chess_game_state_tracker",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="lekiwi_chess_master",
                plugin="lekiwi_chess_master::ChessEngineActionComponent",
                name="chess_engine_action",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="lekiwi_chess_master",
                plugin="lekiwi_chess_master::ChessboardVisualizerComponent",
                name="chessboard_visualizer",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            config_file_arg,
            use_sim_time_arg,
            container,
        ]
    )
