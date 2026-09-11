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
    curr_wheel_radius_arg = DeclareLaunchArgument(
        "current_wheel_radius",
        default_value="0.065",
        description="Current configured wheel radius in meters.",
    )
    curr_robot_radius_arg = DeclareLaunchArgument(
        "current_robot_radius",
        default_value="0.1268",
        description="Current configured robot radius (wheelbase distance to center) in meters.",
    )
    actual_measured_dist_arg = DeclareLaunchArgument(
        "actual_measured_dist",
        default_value="0.0",
        description="Ground truth measured distance for rollout mode (0.0 uses target distance).",
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
                "current_wheel_radius": LaunchConfiguration("current_wheel_radius"),
                "current_robot_radius": LaunchConfiguration("current_robot_radius"),
                "actual_measured_dist": LaunchConfiguration("actual_measured_dist"),
                "imu_topic": "/imu/data_transformed",
                "odom_topic": "/omni_base_controller/odom",
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
            curr_wheel_radius_arg,
            curr_robot_radius_arg,
            actual_measured_dist_arg,
            calib_node,
        ]
    )
