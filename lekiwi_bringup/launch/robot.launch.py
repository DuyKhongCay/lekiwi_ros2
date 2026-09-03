# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Compose LeKiwi system with master switches for each subsystem."""
    bringup_share = FindPackageShare("lekiwi_bringup")

    # =========================================================================
    # 1. Master Subsystem Toggle Arguments (Biến quan trọng nhất cho từng launch)
    # =========================================================================
    master_toggles_spec = [
        (
            "enable_description",
            "true",
            "Master toggle: publish robot model (URDF/xacro) & optional ros2_control manager",
        ),
        (
            "enable_perception",
            "false",
            "Master toggle: start GStreamer camera pipelines and Hailo NPU chess perception",
        ),
        (
            "enable_control",
            "true",
            "Master toggle: start high-level control orchestration, arm controllers & bridges",
        ),
        (
            "enable_imu_pipeline",
            "true",
            "Master toggle: start Madgwick IMU filter & base_footprint TF2 transformer",
        ),
        (
            "start_gamepad_teleop",
            "false",
            "Master toggle: start joystick gamepad teleoperation for mobile base",
        ),
        (
            "start_uarm_teleop",
            "false",
            "Master toggle: start uArm leader teleoperation node for follower arm",
        ),
        (
            "enable_diagnostics",
            "true",
            "Master toggle: start diagnostic_aggregator tree & hardware/system monitors",
        ),
        (
            "enable_navigation",
            "false",
            "Master toggle: start Nav2 stack & twist_mux arbitration around chessboard arena",
        ),
    ]

    # =========================================================================
    # 2. Description Subsystem Arguments
    # =========================================================================
    description_args_spec = [
        (
            "hardware_type",
            "real",
            "Hardware interface type: 'mock' (safe development) or 'real' (physical robot)",
        ),
        (
            "usb_port",
            "/dev/lekiwi_serial",
            "Serial port connected to Feetech STS3215 servo bus",
        ),
        (
            "joint_config_file",
            PathJoinSubstitution(
                [bringup_share, "config", "servos", "lekiwi_arm_calib.yaml"]
            ),
            "Path to arm joint calibration YAML configuration (lekiwi_arm_calib.yaml)",
        ),
        (
            "use_ros2_control",
            "true",
            "Enable ros2_control hardware tags in robot URDF model",
        ),
        (
            "enable_imu",
            "true",
            "Enable ICM-20948 sensor component in ros2_control URDF model",
        ),
        (
            "imu_hardware_type",
            "real",
            "IMU hardware interface type: 'mock' or 'real'",
        ),
        ("imu_i2c_bus", "1", "I2C bus number for ICM-20948 IMU"),
        ("imu_i2c_address", "0x68", "I2C address for ICM-20948 IMU"),
        ("imu_auto_calibrate_gyro", "true", "Perform gyroscope zero-rate calibration"),
        ("imu_gyro_calib_samples", "500", "Sample count for gyroscope calibration"),
        (
            "controllers_file",
            PathJoinSubstitution(
                [bringup_share, "config", "controllers", "lekiwi_controllers.yaml"]
            ),
            "Path to controller_manager controllers YAML configuration",
        ),
        (
            "start_controller_manager",
            "true",
            "Start ros2_control controller_manager node",
        ),
        (
            "activate_controllers",
            "true",
            "Spawn and activate all default controllers (arm, base, imu, mag)",
        ),
        ("use_sim_time", "false", "Use simulation clock if true"),
        ("frame_prefix", "", "Prefix for robot TF frame names"),
        ("joint_states_topic", "/joint_states", "Target topic for joint states"),
        ("ignore_timestamp", "false", "Ignore URDF timestamps in publisher"),
    ]

    # =========================================================================
    # 3. Perception Subsystem Arguments
    # =========================================================================
    perception_args_spec = [
        (
            "data_plane_container",
            "lekiwi_perception_container",
            "Name of ComposableNodeContainer hosting cameras & NPU",
        ),
        (
            "publish_debug_image",
            "false",
            "Publish visual debug overlay image from chess perception",
        ),
        ("use_test_sources", "false", "Use videotestsrc instead of real cameras"),
    ]

    # =========================================================================
    # 4. Control Subsystem Arguments
    # =========================================================================
    control_args_spec = [
        (
            "orchestrator_params_file",
            PathJoinSubstitution(
                [bringup_share, "config", "control", "orchestrator.yaml"]
            ),
            "Path to orchestrator FSM parameters YAML configuration",
        ),
        (
            "start_arm_controller",
            "false",
            "Spawn and activate arm_controller (independent of LeRobot bridge)",
        ),
        (
            "start_lerobot_bridge",
            "false",
            "Start LeRobot arm trajectory bridge node",
        ),
        ("run_camera_demo", "false", "Run camera mode demo client node"),
    ]

    # =========================================================================
    # 5. IMU Subsystem Arguments
    # =========================================================================
    imu_args_spec = [
        (
            "imu_mag_calib_file",
            PathJoinSubstitution(
                [bringup_share, "config", "sensors", "icm20948_magnetometer_calib.yaml"]
            ),
            "Path to magnetometer calibration YAML configuration",
        ),
        (
            "imu_target_frame",
            "base_footprint",
            "Target TF frame for transformed IMU data",
        ),
        (
            "imu_params_file",
            PathJoinSubstitution(
                [bringup_share, "config", "sensors", "imu_filter.yaml"]
            ),
            "Path to IMU Madgwick filter parameters YAML configuration",
        ),
    ]

    # =========================================================================
    # 6. Teleop Gamepad Arguments
    # =========================================================================
    gamepad_args_spec = [
        (
            "gamepad_config_file",
            PathJoinSubstitution(
                [bringup_share, "config", "control", "gamepad_base_teleop.yaml"]
            ),
            "Path to gamepad teleoperation YAML configuration",
        ),
        ("gamepad_device", "/dev/gamepad", "Linux joystick device path"),
        (
            "teleop_cmd_vel_topic",
            "/omni_base_controller/cmd_vel",
            "Target topic for base velocity commands",
        ),
    ]

    # =========================================================================
    # 7. Teleop uArm Leader Arguments
    # =========================================================================
    uarm_args_spec = [
        (
            "uarm_config_file",
            PathJoinSubstitution(
                [bringup_share, "config", "control", "uarm_teleop.yaml"]
            ),
            "Path to uArm leader teleoperation node parameters YAML",
        ),
        (
            "uarm_calib_file",
            PathJoinSubstitution(
                [bringup_share, "config", "servos", "uarm_teleop_calib.yaml"]
            ),
            "Path to unified physical calibration & kinematic mapping YAML",
        ),
        ("uarm_port", "/dev/uarm_leader", "Serial port for uArm leader arm"),
        (
            "uarm_arm_mode",
            "joint_trajectory",
            "Arm dispatch mode: joint_trajectory, forward_position, or joint_states_only",
        ),
        (
            "uarm_publish_rate_hz",
            "50.0",
            "Publish rate in Hz for /leader/joint_states",
        ),
        (
            "uarm_leader_topic",
            "/leader/joint_states",
            "Topic for published leader joint states",
        ),
    ]

    # =========================================================================
    # 8. Diagnostics Subsystem Arguments
    # =========================================================================
    diagnostics_args_spec = [
        (
            "analyzers_config_file",
            PathJoinSubstitution(
                [bringup_share, "config", "diagnostics", "lekiwi_analyzers.yaml"]
            ),
            "Path to diagnostic analyzer grouping hierarchy YAML",
        ),
        (
            "enable_system_monitors",
            "true",
            "Enable host system CPU, RAM, and disk utilization monitors",
        ),
        ("cpu_warning_percentage", "90", "CPU usage percentage warning threshold"),
        ("ram_warning_percentage", "90", "RAM usage percentage warning threshold"),
        ("hd_path", "/", "Root filesystem path for disk space monitoring"),
    ]

    # =========================================================================
    # 9. Navigation Subsystem Arguments
    # =========================================================================
    nav_share = FindPackageShare("lekiwi_navigation")
    navigation_args_spec = [
        (
            "nav_map_file",
            PathJoinSubstitution([nav_share, "maps", "chessboard_arena.yaml"]),
            "Path to static chessboard arena map YAML",
        ),
        (
            "nav_params_file",
            PathJoinSubstitution([nav_share, "config", "nav2_params.yaml"]),
            "Path to Nav2 parameters YAML",
        ),
        (
            "nav_autostart",
            "true",
            "Automatically start Nav2 lifecycle nodes",
        ),
        (
            "nav_twist_mux_config",
            PathJoinSubstitution([nav_share, "config", "twist_mux.yaml"]),
            "Path to twist_mux YAML configuration",
        ),
    ]

    # =========================================================================
    # Include Subsystem Launch Descriptions
    # =========================================================================

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "description.launch.py"])
        ),
        launch_arguments={
            name: LaunchConfiguration(name) for name, _, _ in description_args_spec
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_description")),
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "cameras.launch.py"])
        ),
        launch_arguments={
            "container_name": LaunchConfiguration("data_plane_container"),
            "create_container": "true",
            "publish_debug_image": LaunchConfiguration("publish_debug_image"),
            "use_test_sources": LaunchConfiguration("use_test_sources"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_perception")),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "control.launch.py"])
        ),
        launch_arguments={
            "joint_config_file": LaunchConfiguration("joint_config_file"),
            "orchestrator_params_file": LaunchConfiguration("orchestrator_params_file"),
            "start_arm_controller": LaunchConfiguration("start_arm_controller"),
            "start_lerobot_bridge": LaunchConfiguration("start_lerobot_bridge"),
            "run_camera_demo": LaunchConfiguration("run_camera_demo"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_control")),
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "imu.launch.py"])
        ),
        launch_arguments={
            "imu_params_file": LaunchConfiguration("imu_params_file"),
            "mag_calib_file": LaunchConfiguration("imu_mag_calib_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "target_frame": LaunchConfiguration("imu_target_frame"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_imu_pipeline")),
    )

    teleop_gamepad = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "teleop_gamepad.launch.py"])
        ),
        launch_arguments={
            "teleop_config_file": LaunchConfiguration("gamepad_config_file"),
            "device_name": LaunchConfiguration("gamepad_device"),
            "cmd_vel_topic": LaunchConfiguration("teleop_cmd_vel_topic"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_gamepad_teleop")),
    )

    teleop_uarm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "teleop_uarm.launch.py"])
        ),
        launch_arguments={
            "teleop_config_file": LaunchConfiguration("uarm_config_file"),
            "calibration_file": LaunchConfiguration("uarm_calib_file"),
            "port": LaunchConfiguration("uarm_port"),
            "publish_rate_hz": LaunchConfiguration("uarm_publish_rate_hz"),
            "leader_topic": LaunchConfiguration("uarm_leader_topic"),
            "arm_mode": LaunchConfiguration("uarm_arm_mode"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_uarm_teleop")),
    )

    diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "diagnostics.launch.py"])
        ),
        launch_arguments={
            "analyzers_config_file": LaunchConfiguration("analyzers_config_file"),
            "enable_system_monitors": LaunchConfiguration("enable_system_monitors"),
            "cpu_warning_percentage": LaunchConfiguration("cpu_warning_percentage"),
            "ram_warning_percentage": LaunchConfiguration("ram_warning_percentage"),
            "hd_path": LaunchConfiguration("hd_path"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_diagnostics")),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav_share, "launch", "navigation.launch.py"])
        ),
        launch_arguments={
            "map": LaunchConfiguration("nav_map_file"),
            "params_file": LaunchConfiguration("nav_params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": LaunchConfiguration("nav_autostart"),
            "twist_mux_config": LaunchConfiguration("nav_twist_mux_config"),
            "cmd_vel_out": LaunchConfiguration("teleop_cmd_vel_topic"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_navigation")),
    )

    # Automatic declaration of all arguments with descriptions
    all_declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in (
            master_toggles_spec
            + description_args_spec
            + perception_args_spec
            + control_args_spec
            + imu_args_spec
            + gamepad_args_spec
            + uarm_args_spec
            + diagnostics_args_spec
            + navigation_args_spec
        )
    ]

    return LaunchDescription(
        [
            *all_declared_arguments,
            description,
            cameras,
            control,
            imu,
            teleop_gamepad,
            teleop_uarm,
            diagnostics,
            navigation,
        ]
    )
