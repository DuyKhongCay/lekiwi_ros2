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
    teleop_share = FindPackageShare("teleop_zhongli_servo_hw")

    teleop_config_file = PathJoinSubstitution(
        [teleop_share, "config", "uarm_teleop.yaml"]
    )
    calibration_file = PathJoinSubstitution(
        [teleop_share, "config", "uarm_teleop_calib.yaml"]
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    teleop_leader_node = Node(
        package="teleop_zhongli_servo_hw",
        executable="teleop_uarm_node",
        name="teleop_uarm_node",
        output="screen",
        parameters=[
            teleop_config_file,
            {
                "calibration_file": calibration_file,
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            teleop_leader_node,
        ]
    )
