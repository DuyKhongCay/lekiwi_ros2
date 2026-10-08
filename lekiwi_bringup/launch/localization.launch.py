# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unified IMU pre-processing and dual-stage EKF localization launch file.

Sets up Madgwick orientation filter, IMU frame transformer, magnetometer
bias pipeline, and Dual EKF state estimation (Local EKF for odom -> base_footprint
and Global EKF for map -> odom) inside a single multi-threaded composable container.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Configure unified IMU filter, transformer, and dual EKF localization container.

    Returns:
        LaunchDescription containing static composable container and observer node.
    """
    # 1. Resolve configuration parameter paths
    bringup_share = FindPackageShare("lekiwi_bringup")

    default_imu_params = PathJoinSubstitution(
        [bringup_share, "config", "localization", "imu.yaml"]
    )
    ekf_local_params = PathJoinSubstitution(
        [bringup_share, "config", "localization", "ekf_local.yaml"]
    )
    ekf_global_params = PathJoinSubstitution(
        [bringup_share, "config", "localization", "ekf_global.yaml"]
    )

    # 2. Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    enable_ekf_arg = DeclareLaunchArgument(
        "enable_ekf",
        default_value="true",
        description="Enable Local and Global robot_localization EKF nodes",
    )
    use_mag_arg = DeclareLaunchArgument(
        "use_mag",
        default_value="false",
        description="Enable magnetometer pipeline and fusion in Local EKF",
    )
    
    # 3. Define IMU pre-processing components
    imu_filter_component = ComposableNode(
        package="imu_filter_madgwick",
        plugin="ImuFilterMadgwickRos",
        name="imu_filter",
        parameters=[
            default_imu_params,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "use_mag": LaunchConfiguration("use_mag"),
            },
        ],
        remappings=[
            ("imu/data_raw", "/lekiwi_imu_broadcaster/imu"),
            ("imu/mag", "/magnetic_field/calibrated"),
            ("imu/data", "/imu/data"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    imu_transformer_component = ComposableNode(
        package="imu_transformer",
        plugin="imu_transformer::ImuTransformer",
        name="imu_transformer",
        parameters=[
            default_imu_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("imu_in", "/imu/data"),
            ("imu_out", "/imu/data_transformed"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # 4. Define Dual EKF components
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        parameters=[
            ekf_local_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("odometry/filtered", "/odometry/local"),
            ("set_pose", "/ekf_filter_node_odom/set_pose"),
            ("enable", "/ekf_filter_node_odom/enable"),
            ("reset", "/ekf_filter_node_odom/reset"),
            ("toggle", "/ekf_filter_node_odom/toggle"),
        ],
    )

    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        parameters=[
            ekf_global_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("odometry/filtered", "/odometry/global"),
            ("set_pose", "/ekf_filter_node_map/set_pose"),
            ("enable", "/ekf_filter_node_map/enable"),
            ("reset", "/ekf_filter_node_map/reset"),
            ("toggle", "/ekf_filter_node_map/toggle"),
        ],
    )

    # 5. Magnetometer bias remover component
    mag_bias_remover_component = ComposableNode(
        package="magnetometer_pipeline",
        plugin="magnetometer_pipeline::MagnetometerBiasRemoverNodelet",
        name="magnetometer_bias_remover",
        parameters=[
            default_imu_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("imu/mag", "/lekiwi_magnetometer_broadcaster/magnetic_field"),
            ("imu/mag_bias", "/imu/mag_bias"),
            ("imu/mag_unbiased", "/magnetic_field/calibrated"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # 6. Assemble static composable components list (cameras.launch.py pattern)
    all_localization_components = [
        imu_filter_component,
        imu_transformer_component,
        mag_bias_remover_component,
    ]

    localization_container = ComposableNodeContainer(
        name="lekiwi_localization_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=all_localization_components,
        output="screen",
    )

    # 7. Magnetometer bias observer node (Python node delayed until container runs)
    mag_bias_observer_node = Node(
        package="magnetometer_pipeline",
        executable="magnetometer_bias_observer.py",
        name="mag_bias_observer",
        output="screen",
        parameters=[
            default_imu_params,
            {
                "2d_mode": False,
                "measuring_time": 30.0,
                "load_from_params": True,
                "load_from_file": False,
                "save_to_file": False,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[
            ("imu/mag", "/lekiwi_magnetometer_broadcaster/magnetic_field"),
            ("imu/mag_bias", "/imu/mag_bias"),
        ],
        condition=IfCondition(LaunchConfiguration("use_mag")),
    )

    delayed_mag_bias_observer = RegisterEventHandler(
        OnProcessStart(
            target_action=localization_container,
            on_start=[
                TimerAction(
                    period=4.0,
                    actions=[mag_bias_observer_node],
                )
            ],
        ),
        condition=IfCondition(LaunchConfiguration("use_mag")),
    )

    # 8. Assemble LaunchDescription
    return LaunchDescription(
        [
            use_sim_time_arg,
            enable_ekf_arg,
            use_mag_arg,
            localization_container,
            ekf_local_node,
            ekf_global_node,
            delayed_mag_bias_observer,
        ]
    )
