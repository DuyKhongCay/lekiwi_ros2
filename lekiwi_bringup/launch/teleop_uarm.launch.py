# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch Teleoperation Leader driver node for uArm (Zhongli servos)."""
    bringup_share = FindPackageShare("lekiwi_bringup")

    teleop_config_file = LaunchConfiguration("teleop_config_file")
    calibration_file = LaunchConfiguration("calibration_file")
    port = LaunchConfiguration("port")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    leader_topic = LaunchConfiguration("leader_topic")
    arm_mode = LaunchConfiguration("arm_mode")
    follower_jtc_topic = LaunchConfiguration("follower_jtc_topic")
    follower_fwd_topic = LaunchConfiguration("follower_fwd_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declared_args_spec = [
        (
            "teleop_config_file",
            PathJoinSubstitution(
                [bringup_share, "config", "control", "uarm_teleop.yaml"]
            ),
            "Path to the uArm teleop runtime configuration YAML file",
        ),
        (
            "calibration_file",
            PathJoinSubstitution(
                [bringup_share, "config", "servos", "uarm_teleop_calib.yaml"]
            ),
            "Path to the unified physical calibration and kinematic mapping YAML file",
        ),
        ("port", "/dev/uarm_leader", "Serial port connected to uArm leader arm"),
        ("publish_rate_hz", "50.0", "Publish rate in Hz for /leader/joint_states"),
        (
            "leader_topic",
            "/leader/joint_states",
            "Topic for published leader joint states",
        ),
        (
            "arm_mode",
            "joint_trajectory",
            "Arm command dispatch mode: joint_trajectory, forward_position, or joint_states_only",
        ),
        (
            "follower_jtc_topic",
            "/arm_trajectory_controller/joint_trajectory",
            "Topic for follower JointTrajectoryController",
        ),
        (
            "follower_fwd_topic",
            "/arm_forward_controller/commands",
            "Topic for follower ForwardCommandController",
        ),
        ("use_sim_time", "false", "Use simulation clock if true"),
    ]

    all_declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in declared_args_spec
    ]

    teleop_leader_node = Node(
        package="teleop_zhongli_servo_hw",
        executable="teleop_uarm_node",
        name="teleop_uarm_node",
        output="screen",
        parameters=[
            teleop_config_file,
            {
                "port": port,
                "calibration_file": calibration_file,
                "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                "leader_topic": leader_topic,
                "arm_mode": arm_mode,
                "follower_jtc_topic": follower_jtc_topic,
                "follower_fwd_topic": follower_fwd_topic,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            *all_declared_arguments,
            teleop_leader_node,
        ]
    )
