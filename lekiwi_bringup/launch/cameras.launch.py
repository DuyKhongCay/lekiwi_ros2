# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _camera_streamer_component(namespace, params_file):
    """Build a CameraStreamerComponent that loads pipeline and camera config from YAML."""
    return ComposableNode(
        package="lekiwi_perception",
        plugin="lekiwi_perception::CameraStreamerComponent",
        name="gscam",
        namespace=namespace,
        parameters=[params_file],
        remappings=[
            ("camera/image_raw", "image_raw"),
            ("camera/camera_info", "camera_info"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def generate_launch_description():
    """Launch CameraStreamerComponent drivers and Hailo chess perception component as composed nodes."""
    container_name = LaunchConfiguration("container_name")
    create_container = LaunchConfiguration("create_container")
    gscam_params_file = LaunchConfiguration("gscam_params_file")

    camera_namespaces = [
        "cameras/stereo_left",
        "cameras/stereo_right",
        "cameras/usb_wrist",
        "cameras/usb_side",
    ]

    camera_components = [
        _camera_streamer_component(ns, gscam_params_file) for ns in camera_namespaces
    ]

    inference_component = ComposableNode(
        package="lekiwi_perception",
        plugin="lekiwi_perception::HailoChessInferenceComponent",
        name="hailo_chess_inference",
        parameters=[
            {
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    tag_localization_share = FindPackageShare("lekiwi_tag_localization")
    apriltag_params_file = LaunchConfiguration("apriltag_params_file")
    chessboard_params_file = LaunchConfiguration("chessboard_params_file")

    apriltag_component = ComposableNode(
        package="apriltag_ros",
        plugin="AprilTagNode",
        name="apriltag_detector",
        namespace="",
        remappings=[
            ("image_rect", "/cameras/stereo_left/image_raw"),
            ("camera_info", "/cameras/stereo_left/camera_info"),
            ("detections", "/tag_detections"),
        ],
        parameters=[apriltag_params_file],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    chessboard_estimator_component = ComposableNode(
        package="lekiwi_tag_localization",
        plugin="lekiwi_tag_localization::ChessboardPoseEstimator",
        name="chessboard_pose_estimator",
        namespace="",
        remappings=[
            ("/cameras/stereo_left/camera_info", "/cameras/stereo_left/camera_info"),
            ("/tag_detections", "/tag_detections"),
        ],
        parameters=[chessboard_params_file],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    all_components = [
        *camera_components,
        inference_component,
        apriltag_component,
        chessboard_estimator_component,
    ]

    container = ComposableNodeContainer(
        name=container_name,
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=all_components,
        output="screen",
        condition=IfCondition(create_container),
    )
    loader = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=all_components,
        condition=UnlessCondition(create_container),
    )

    bringup_share = FindPackageShare("lekiwi_bringup")

    declared_args_spec = [
        ("container_name", "lekiwi_perception_container"),
        ("create_container", "true"),
        ("publish_debug_image", "true"),
        (
            "gscam_params_file",
            PathJoinSubstitution(
                [bringup_share, "config", "perception", "gscam_cameras.yaml"]
            ),
        ),
        (
            "apriltag_params_file",
            PathJoinSubstitution(
                [tag_localization_share, "config", "apriltag_36h11.yaml"]
            ),
        ),
        (
            "chessboard_params_file",
            PathJoinSubstitution(
                [tag_localization_share, "config", "chessboard_tags.yaml"]
            ),
        ),
    ]

    all_declared_arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in declared_args_spec
    ]

    return LaunchDescription(
        [
            *all_declared_arguments,
            container,
            loader,
        ]
    )
