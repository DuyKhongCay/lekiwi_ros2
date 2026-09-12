# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Generates launch description for running LeKiwi omni base calibration routines.
def generate_launch_description():
    # Configures launch arguments and registers the omni base calibrator node.
    calib_mode_arg = DeclareLaunchArgument(
        "calib_mode",
        default_value="spin",
        description="Calibration routine mode: spin (IMU Ground Truth), rollout, or square.",
    )
    test_dist_arg = DeclareLaunchArgument(
        "test_dist",
        default_value="1.0",
        description="Target linear distance for rollout or edge length for square test in meters.",
    )
    rot_cnt_arg = DeclareLaunchArgument(
        "rot_cnt",
        default_value="5",
        description="Number of complete rotations to perform during in-place spin test.",
    )
    linear_vel_arg = DeclareLaunchArgument(
        "linear_vel",
        default_value="0.15",
        description="Linear velocity in m/s during test executions.",
    )
    angular_vel_arg = DeclareLaunchArgument(
        "angular_vel",
        default_value="0.5",
        description="Angular velocity in rad/s during rotation maneuvers.",
    )
    controller_name_arg = DeclareLaunchArgument(
        "controller_name",
        default_value="omni_base_controller",
        description="Name of the running omni base controller node to query parameters from.",
    )
    actual_measured_dist_arg = DeclareLaunchArgument(
        "actual_measured_dist",
        default_value="0.0",
        description="Ground truth measured distance for rollout mode (0.0 uses target distance).",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odometry/filtered",
        description="Odometry topic for state feedback (e.g. /odometry/filtered or /omni_base_controller/odom).",
    )

    calib_node = Node(
        package="lekiwi_control",
        executable="calibrate_omni_base",
        name="omni_base_calibrator",
        output="screen",
        parameters=[
            {
                "calib_mode": LaunchConfiguration("calib_mode"),
                "test_dist": LaunchConfiguration("test_dist"),
                "rot_cnt": LaunchConfiguration("rot_cnt"),
                "linear_vel": LaunchConfiguration("linear_vel"),
                "angular_vel": LaunchConfiguration("angular_vel"),
                "controller_name": LaunchConfiguration("controller_name"),
                "actual_measured_dist": LaunchConfiguration("actual_measured_dist"),
                "imu_topic": "/imu/data_transformed",
                "odom_topic": LaunchConfiguration("odom_topic"),
                "tag_pose_topic": "/chessboard/robot_pose",
                "cmd_vel_topic": "/cmd_vel_calib",
            }
        ],
    )

    return LaunchDescription(
        [
            calib_mode_arg,
            test_dist_arg,
            rot_cnt_arg,
            linear_vel_arg,
            angular_vel_arg,
            controller_name_arg,
            actual_measured_dist_arg,
            odom_topic_arg,
            calib_node,
        ]
    )
