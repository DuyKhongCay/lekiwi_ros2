# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Compose LeKiwi description, perception, and control subsystem launches."""
    bringup_share = FindPackageShare("lekiwi_bringup")

    # Description subsystem arguments
    description_args_spec = [
        ("hardware_type", "mock"),
        ("usb_port", "/dev/lekiwi_serial"),
        (
            "joint_config_file",
            PathJoinSubstitution(
                [bringup_share, "config", "hardware", "lekiwi_joints.yaml"]
            ),
        ),
        ("use_ros2_control", "true"),
        ("enable_imu", "true"),
        ("imu_hardware_type", "mock"),
        ("imu_i2c_bus", "1"),
        ("imu_i2c_address", "0x68"),
        ("imu_auto_calibrate_gyro", "true"),
        ("imu_gyro_calib_samples", "500"),
        (
            "controllers_file",
            PathJoinSubstitution(
                [bringup_share, "config", "controllers", "lekiwi_controllers.yaml"]
            ),
        ),
        ("start_controller_manager", "false"),
        ("activate_controllers", "false"),
        ("use_sim_time", "false"),
        ("frame_prefix", ""),
        ("joint_states_topic", "/joint_states"),
        ("ignore_timestamp", "false"),
    ]

    # Perception subsystem arguments
    perception_args_spec = [
        ("data_plane_container", "lekiwi_perception_container"),
        (
            "camera_params_file",
            PathJoinSubstitution(
                [bringup_share, "config", "perception", "cameras.yaml"]
            ),
        ),
        ("use_test_sources", "false"),
    ]

    # Control subsystem arguments
    control_args_spec = [
        (
            "orchestrator_params_file",
            PathJoinSubstitution(
                [bringup_share, "config", "control", "orchestrator.yaml"]
            ),
        ),
        ("start_lerobot_bridge", "false"),
        ("run_camera_demo", "false"),
    ]

    # IMU Filtering pipeline arguments (Layer 2)
    imu_args_spec = [
        ("enable_imu_pipeline", "true"),
        ("enable_imu_transformer", "true"),
        ("imu_use_mag", "true"),
        ("imu_target_frame", "base_footprint"),
        (
            "imu_params_file",
            PathJoinSubstitution(
                [bringup_share, "config", "sensors", "imu_filter.yaml"]
            ),
        ),
    ]

    # Subsystem include definitions
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "description.launch.py"])
        ),
        launch_arguments={
            name: LaunchConfiguration(name) for name, _ in description_args_spec
        }.items(),
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "cameras.launch.py"])
        ),
        launch_arguments={
            "container_name": LaunchConfiguration("data_plane_container"),
            "create_container": "true",
            "cam_params_file": LaunchConfiguration("camera_params_file"),
            "use_test_sources": LaunchConfiguration("use_test_sources"),
        }.items(),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "control.launch.py"])
        ),
        launch_arguments={
            "joint_config_file": LaunchConfiguration("joint_config_file"),
            "orchestrator_params_file": LaunchConfiguration("orchestrator_params_file"),
            "start_lerobot_bridge": LaunchConfiguration("start_lerobot_bridge"),
            "run_camera_demo": LaunchConfiguration("run_camera_demo"),
        }.items(),
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "imu.launch.py"])
        ),
        launch_arguments={
            "imu_params_file": LaunchConfiguration("imu_params_file"),
            "use_mag": LaunchConfiguration("imu_use_mag"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "enable_transformer": LaunchConfiguration("enable_imu_transformer"),
            "target_frame": LaunchConfiguration("imu_target_frame"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_imu_pipeline")),
    )

    # Automatic declaration of all arguments
    all_declared_arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in (
            description_args_spec
            + perception_args_spec
            + control_args_spec
            + imu_args_spec
        )
    ]

    return LaunchDescription(
        [
            *all_declared_arguments,
            description,
            cameras,
            control,
            imu,
        ]
    )
