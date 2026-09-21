# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Launch file for running chessboard AprilTag calibration with camera streamer and tag detector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Configures launch arguments, starts perception container, and runs calibrator node."""
    calib_share = FindPackageShare("lekiwi_calibration")
    bringup_share = FindPackageShare("lekiwi_bringup")

    default_config_path = PathJoinSubstitution(
        [calib_share, "config", "chessboard_calib_params.yaml"]
    )
    gscam_params_file = PathJoinSubstitution(
        [bringup_share, "config", "perception", "gscam_cameras.yaml"]
    )
    chessboard_params_file = PathJoinSubstitution(
        [bringup_share, "config", "localization", "chessboard_tags.yaml"]
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_path,
        description="Path to calibrator YAML config file.",
    )

    # 1. Intra-process ComposableNodeContainer for zero-copy camera streaming & AprilTag detection
    camera_streamer_component = ComposableNode(
        package="lekiwi_perception",
        plugin="lekiwi_perception::CameraStreamerComponent",
        name="gscam",
        namespace="cameras/stereo_left",
        parameters=[
            gscam_params_file,
            {"calib_mode": True},
        ],
        remappings=[
            ("camera/image_raw", "image_raw"),
            ("camera/image_raw/compressed", "image_raw/compressed"),
            ("camera/camera_info", "camera_info"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    chessboard_estimator_component = ComposableNode(
        package="lekiwi_perception",
        plugin="lekiwi_perception::ChessboardPoseEstimator",
        name="chessboard_pose_estimator",
        namespace="",
        remappings=[
            ("~/image_raw", "/cameras/stereo_left/image_raw"),
            ("~/camera_info", "/cameras/stereo_left/camera_info"),
            ("~/camera_mode", "/camera_mode"),
        ],
        parameters=[
            chessboard_params_file,
            {"calib": True},
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    perception_container = ComposableNodeContainer(
        name="chessboard_calib_perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            camera_streamer_component,
            chessboard_estimator_component,
        ],
        output="screen",
    )

    # 2. Main GUI and Bundle Adjustment calibration node
    calib_node = Node(
        package="lekiwi_calibration",
        executable="calibrate_chessboard",
        name="chessboard_tag_calibrator",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription(
        [
            config_file_arg,
            perception_container,
            calib_node,
        ]
    )
