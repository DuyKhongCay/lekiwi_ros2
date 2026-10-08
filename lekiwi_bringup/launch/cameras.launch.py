# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Multi-camera GStreamer streaming and Hailo NPU perception bringup launch file.

Loads 4 camera streamer components (stereo pair, wrist, side) and Hailo-8
neural inference components into a single multi-threaded intra-process container.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _camera_streamer_component(namespace, params_file):
    """Build ComposableNode description for a GStreamer camera streamer instance.

    Args:
        namespace: ROS namespace prefix for camera topics (e.g. 'cameras/stereo_left').
        params_file: Path substitution to gscam_cameras.yaml parameter file.

    Returns:
        ComposableNode configured with intra-process communication.
    """
    return ComposableNode(
        package="lekiwi_perception",
        plugin="lekiwi_perception::CameraStreamerComponent",
        name="gscam",
        namespace=namespace,
        parameters=[params_file],
        remappings=[
            ("camera/image_raw", "image_raw"),
            ("camera/image_raw/compressed", "image_raw/compressed"),
            ("camera/camera_info", "camera_info"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def generate_launch_description():
    """Configure intra-process container holding 4 camera drivers and Hailo chess perception.

    Returns:
        LaunchDescription containing the multi-threaded perception container.
    """
    # 1. Resolve perception configuration parameter files
    bringup_share = FindPackageShare("lekiwi_bringup")

    gscam_params_file = PathJoinSubstitution(
        [bringup_share, "config", "perception", "gscam_cameras.yaml"]
    )
    perception_params_file = PathJoinSubstitution(
        [bringup_share, "config", "perception", "perception_config.yaml"]
    )

    # 2. Configure 4 GStreamer camera streaming components
    camera_namespaces = [
        "cameras/stereo_left",
        "cameras/stereo_right",
        "cameras/usb_wrist",
        "cameras/usb_side",
    ]

    camera_components = [
        _camera_streamer_component(ns, gscam_params_file) for ns in camera_namespaces
    ]

    # 3. Configure Hailo NPU chess inference and chessboard pose estimators
    inference_component = ComposableNode(
        package="lekiwi_perception",
        plugin="lekiwi_perception::HailoChessInferenceComponent",
        name="hailo_chess_inference",
        parameters=[perception_params_file],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    chess_overlay_visualizer = ComposableNode(
        package="lekiwi_perception",
        plugin="lekiwi_perception::ChessOverlayVisualizer",
        name="chess_overlay_visualizer",
        parameters=[perception_params_file],
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
            ("~/perception_context", "/perception_context"),
        ],
        parameters=[perception_params_file],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    all_components = [
        *camera_components,
        inference_component,
        chess_overlay_visualizer,
        chessboard_estimator_component,
    ]

    # 4. Assemble multi-threaded composable component container
    container = ComposableNodeContainer(
        name="lekiwi_perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=all_components,
        output="screen",
    )

    return LaunchDescription([container])
