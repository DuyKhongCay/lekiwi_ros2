# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    default_mag_calib = PathJoinSubstitution(
        [bringup_share, "config", "sensors", "icm20948_magnetometer_calib.yaml"]
    )

    declared_args_spec = [
        ("imu_params_file", default_imu_params),
        ("mag_calib_file", default_mag_calib),
        ("use_sim_time", "false"),
        ("target_frame", "base_footprint"),
        ("raw_imu_topic", "/lekiwi_imu_broadcaster/imu"),
        ("raw_mag_topic", "/lekiwi_magnetometer_broadcaster/magnetic_field"),
        ("calibrated_mag_topic", "/magnetic_field/calibrated"),
        ("filtered_imu_topic", "/imu/data"),
        ("transformed_imu_topic", "/imu/data_transformed"),
    ]

    all_declared_arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in declared_args_spec
    ]

    use_sim_time_param = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )

    # 1. Magnetometer Bias Observer (Calibrate via /calibrate_magnetometer service & load/save YAML)
    mag_bias_observer_node = Node(
        package="magnetometer_pipeline",
        executable="magnetometer_bias_observer.py",
        name="mag_bias_observer",
        output="screen",
        parameters=[
            {
                "calibration_file_path": LaunchConfiguration("mag_calib_file"),
                "2d_mode": False,
                "measuring_time": 30.0,
                "load_from_params": False,
                "load_from_file": True,
                "save_to_file": True,
                "use_sim_time": use_sim_time_param,
            }
        ],
        remappings=[
            ("imu/mag", LaunchConfiguration("raw_mag_topic")),
            ("imu/mag_bias", "/imu/mag_bias"),
        ],
    )

    # 2. Magnetometer Bias Remover (Substracts bias & scales ellipsoid to unit sphere)
    mag_bias_remover_node = Node(
        package="magnetometer_pipeline",
        executable="magnetometer_bias_remover_node",
        name="magnetometer_bias_remover",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
            }
        ],
        remappings=[
            ("imu/mag", LaunchConfiguration("raw_mag_topic")),
            ("imu/mag_bias", "/imu/mag_bias"),
            ("imu/mag_unbiased", LaunchConfiguration("calibrated_mag_topic")),
        ],
    )

    # 3. Madgwick AHRS Filter (Fuses linear accel, calibrated mag and angular vel into absolute orientation)
    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        output="screen",
        parameters=[
            LaunchConfiguration("imu_params_file"),
            {
                "use_mag": True,
                "use_sim_time": use_sim_time_param,
            },
        ],
        remappings=[
            ("imu/data_raw", LaunchConfiguration("raw_imu_topic")),
            ("imu/mag", LaunchConfiguration("calibrated_mag_topic")),
            ("imu/data", LaunchConfiguration("filtered_imu_topic")),
        ],
    )

    # 4. IMU Transformer (Transforms IMU tensors from sensor frame to robot base_footprint frame)
    imu_transformer_node = Node(
        package="imu_transformer",
        executable="imu_transformer_node",
        name="imu_transformer",
        output="screen",
        parameters=[
            LaunchConfiguration("imu_params_file"),
            {
                "target_frame": LaunchConfiguration("target_frame"),
                "use_sim_time": use_sim_time_param,
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
            mag_bias_observer_node,
            mag_bias_remover_node,
            imu_filter_node,
            imu_transformer_node,
        ]
    )
