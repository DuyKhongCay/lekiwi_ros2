# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch LeKiwi IMU pre-processing & filtering pipeline (Layer 2)."""
    bringup_share = FindPackageShare("lekiwi_bringup")

    default_imu_params = PathJoinSubstitution(
        [bringup_share, "config", "imu", "imu_filter.yaml"]
    )
    default_mag_calib = PathJoinSubstitution(
        [bringup_share, "config", "imu", "icm20948_magnetometer_calib.yaml"]
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock"
    )
    use_mag_arg = DeclareLaunchArgument(
        "use_mag",
        default_value="false",
        description="Enable magnetometer pipeline and fusion",
    )

    use_sim_time_param = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    use_mag_param = ParameterValue(LaunchConfiguration("use_mag"), value_type=bool)

    # 1. Magnetometer Bias Observer (Python node: calibrate via /calibrate_magnetometer service & load/save YAML)
    mag_bias_observer_node = Node(
        package="magnetometer_pipeline",
        executable="magnetometer_bias_observer.py",
        name="mag_bias_observer",
        output="screen",
        parameters=[
            {
                "calibration_file_path": default_mag_calib,
                "2d_mode": False,
                "measuring_time": 30.0,
                "load_from_params": False,
                "load_from_file": True,
                "save_to_file": True,
                "use_sim_time": use_sim_time_param,
            }
        ],
        remappings=[
            ("imu/mag", "/lekiwi_magnetometer_broadcaster/magnetic_field"),
            ("imu/mag_bias", "/imu/mag_bias"),
        ],
        condition=IfCondition(LaunchConfiguration("use_mag")),
    )

    # 2. Composable Components for C++ High-frequency IMU pipeline
    mag_bias_remover_component = ComposableNode(
        package="magnetometer_pipeline",
        plugin="magnetometer_pipeline::MagnetometerBiasRemoverNodelet",
        name="magnetometer_bias_remover",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
            },
        ],
        remappings=[
            ("imu/mag", "/lekiwi_magnetometer_broadcaster/magnetic_field"),
            ("imu/mag_bias", "/imu/mag_bias"),
            ("imu/mag_unbiased", "/magnetic_field/calibrated"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    imu_filter_component = ComposableNode(
        package="imu_filter_madgwick",
        plugin="ImuFilterMadgwickRos",
        name="imu_filter",
        parameters=[
            default_imu_params,
            {
                "use_sim_time": use_sim_time_param,
                "use_mag": use_mag_param,
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
            {"use_sim_time": use_sim_time_param},
        ],
        remappings=[
            ("imu_in", "/imu/data"),
            ("imu_out", "/imu/data_transformed"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    imu_container = ComposableNodeContainer(
        name="lekiwi_imu_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            imu_filter_component,
            imu_transformer_component,
        ],
        output="screen",
    )

    load_mag_bias_remover = LoadComposableNodes(
        target_container=imu_container,
        composable_node_descriptions=[mag_bias_remover_component],
        condition=IfCondition(LaunchConfiguration("use_mag")),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_mag_arg,
            imu_container,
            load_mag_bias_remover,
            mag_bias_observer_node,
        ]
    )
