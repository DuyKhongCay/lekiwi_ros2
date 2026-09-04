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
    bringup_share = FindPackageShare("lekiwi_bringup")
    description_share = FindPackageShare("lekiwi_description")

    xacro_file = PathJoinSubstitution(
        [description_share, "urdf", "lekiwi_robot.urdf.xacro"]
    )

    joint_config_file = PathJoinSubstitution(
        [bringup_share, "config", "servos", "lekiwi_arm_calib.yaml"]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "hardware_type",
            default_value="real",
            description="Hardware type: real or mock",
        ),
        DeclareLaunchArgument(
            "imu_hardware_type",
            default_value="real",
            description="IMU hardware type: real or mock",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
    ]

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    xacro_file,
                    " hardware_type:=",
                    LaunchConfiguration("hardware_type"),
                    " imu_hardware_type:=",
                    LaunchConfiguration("imu_hardware_type"),
                    " joint_config_file:=",
                    joint_config_file,
                ]
            ),
            value_type=str,
        )
    }

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
