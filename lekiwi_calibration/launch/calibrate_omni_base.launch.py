# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Single Entry Point Launch: Bring up necessary robot stack and Omni Base Calibrator."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Configure robot bringup via robot.launch.py, maintain perception context, and run calibrator."""
    calib_share = FindPackageShare("lekiwi_calibration")
    bringup_share = FindPackageShare("lekiwi_bringup")

    default_config_path = PathJoinSubstitution(
        [calib_share, "config", "omni_base_calib_params.yaml"]
    )

    # 1. Launch arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_path,
        description="Path to omni base calibrator YAML config file.",
    )
    hardware_type_arg = DeclareLaunchArgument(
        "hardware_type",
        default_value="real",
        description="Hardware interface type: real or mock.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true.",
    )

    config_file = LaunchConfiguration("config_file")
    hardware_type = LaunchConfiguration("hardware_type")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 2. Main Robot Stack Bringup (Description, Controllers, IMU, EKF, Cameras, Teleop Gamepad)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "robot.launch.py"])
        ),
        launch_arguments={
            "hardware_type": hardware_type,
            "use_sim_time": use_sim_time,
            "arm_controller": "false",
            "base_controller": "true",
            "imu_broadcaster": "true",
            "enable_ekf": "true",
            "cameras": "true",
            "teleop_gamepad": "true",
            "enable_orchestrator": "true",
            "enable_readiness_checks": "false",
            "start_mission": "false",
            "navigation": "false",
            "chess_master": "false",
            "manipulation": "false",
        }.items(),
    )

    # 3. Action: Maintain continuous PerceptionContext at 1 Hz (TF_TRACKING_AND_NAV = 1)
    # Ensures all camera nodes (current and late joiners) receive context 1 and open GStreamer valve
    # set_perception_context = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "topic",
    #         "pub",
    #         "-r",
    #         "1",
    #         "--qos-durability",
    #         "transient_local",
    #         "--qos-reliability",
    #         "reliable",
    #         "/perception_context",
    #         "lekiwi_interfaces/msg/PerceptionContext",
    #         "{value: 1}",
    #     ],
    #     output="screen",
    # )

    # 4. Omni Base Kinematic Calibrator Node (automatically waits for omni_base_controller service)
    calib_node = Node(
        package="lekiwi_calibration",
        executable="calibrate_omni_base",
        name="omni_base_calibrator",
        output="screen",
        parameters=[
            config_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            config_file_arg,
            hardware_type_arg,
            use_sim_time_arg,
            robot_bringup,
            # set_perception_context,
            calib_node,
        ]
    )
