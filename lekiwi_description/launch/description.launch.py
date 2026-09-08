# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Publish the robot model (URDF/xacro) and robot_state_publisher."""
    description_share = FindPackageShare("lekiwi_description")
    xacro_file = PathJoinSubstitution(
        [description_share, "urdf", "lekiwi_robot.urdf.xacro"]
    )
    default_joint_config = PathJoinSubstitution(
        [description_share, "config", "calibration", "sts3215_servos_calib.yaml"]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "hardware_type",
            default_value="true",
            description="Hardware type: real or mock",
        ),
        DeclareLaunchArgument(
            "imu_hardware_type",
            default_value="true",
            description="IMU hardware type: real or mock",
        ),
        DeclareLaunchArgument(
            "joint_config_file",
            default_value=default_joint_config,
            description="Path to joint calibration config file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
    ]

    robot_description_content = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " hardware_type:=",
                LaunchConfiguration("hardware_type"),
                " imu_hardware_type:=",
                LaunchConfiguration("imu_hardware_type"),
                " joint_config_file:=",
                LaunchConfiguration("joint_config_file"),
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "publish_frequency": 50.0,
            },
        ],
    )

    return LaunchDescription(
        [
            *declared_arguments,
            rsp_node,
        ]
    )

