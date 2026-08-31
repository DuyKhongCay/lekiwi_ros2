# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch LeKiwi IMU pre-processing & filtering pipeline (Layer 2)."""
    bringup_share = FindPackageShare("lekiwi_bringup")

    default_imu_params = PathJoinSubstitution(
        [bringup_share, "config", "sensors", "imu_filter.yaml"]
    )

    declared_args_spec = [
        ("imu_params_file", default_imu_params),
        ("use_mag", "true"),
        ("use_sim_time", "false"),
        ("enable_transformer", "true"),
        ("target_frame", "base_footprint"),
        ("raw_imu_topic", "/lekiwi_imu_broadcaster/imu"),
        ("raw_mag_topic", "/lekiwi_magnetometer_broadcaster/magnetic_field"),
        ("filtered_imu_topic", "/imu/data"),
        ("transformed_imu_topic", "/imu/data_transformed"),
    ]

    all_declared_arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in declared_args_spec
    ]

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        output="screen",
        parameters=[
            LaunchConfiguration("imu_params_file"),
            {
                "use_mag": ParameterValue(
                    LaunchConfiguration("use_mag"), value_type=bool
                ),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            },
        ],
        remappings=[
            ("imu/data_raw", LaunchConfiguration("raw_imu_topic")),
            ("imu/mag", LaunchConfiguration("raw_mag_topic")),
            ("imu/data", LaunchConfiguration("filtered_imu_topic")),
        ],
    )

    imu_transformer_node = Node(
        package="imu_transformer",
        executable="imu_transformer_node",
        name="imu_transformer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_transformer")),
        parameters=[
            LaunchConfiguration("imu_params_file"),
            {
                "target_frame": LaunchConfiguration("target_frame"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            },
        ],
        remappings=[
            ("imu_in", LaunchConfiguration("filtered_imu_topic")),
            ("imu_out", LaunchConfiguration("transformed_imu_topic")),
        ],
    )

    return LaunchDescription(
        [
            *all_declared_arguments,
            imu_filter_node,
            imu_transformer_node,
        ]
    )
