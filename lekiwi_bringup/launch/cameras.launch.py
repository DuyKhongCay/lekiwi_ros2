# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
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
            ("camera/image_raw/compressed", "image_raw/compressed"),
            ("camera/camera_info", "camera_info"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def generate_launch_description():
    """Launch CameraStreamerComponent drivers and Hailo chess perception component as composed nodes."""
    bringup_share = FindPackageShare("lekiwi_bringup")

    gscam_params_file = PathJoinSubstitution(
        [bringup_share, "config", "perception", "gscam_cameras.yaml"]
    )
    chessboard_params_file = PathJoinSubstitution(
        [bringup_share, "config", "localization", "chessboard_tags.yaml"]
    )

    hailo_infer_params_file = PathJoinSubstitution(
        [bringup_share, "config", "perception", "hailo_chess_infer_config.yaml"]
    )

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
        parameters=[hailo_infer_params_file],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    chess_visualizer_component = ComposableNode(
        package="lekiwi_perception",
        plugin="lekiwi_perception::ChessVisualizerComponent",
        name="chess_visualizer",
        parameters=[
            {
                "camera_topic": "/cameras/stereo_left/image_raw",
                "fen_topic": "/chess/fen",
                "detections_topic": "/chess/detections_2d",
                "overlay_topic": "/chess/overlay_image/compressed",
                "board_2d_topic": "/chess/board_2d/compressed",
                "jpeg_quality": 85,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    chess_engine_component = ComposableNode(
        package="lekiwi_perception",
        plugin="lekiwi_perception::ChessEngineComponent",
        name="chess_engine",
        parameters=[
            {
                "stockfish_path": "/usr/games/stockfish",
                "think_time_ms": 1000,
                "robot_color": "black",
                "auto_play": True,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    chessboard_estimator_component = ComposableNode(
        package="apriltag_localizer",
        plugin="apriltag_localizer::ChessboardPoseEstimator",
        name="chessboard_pose_estimator",
        namespace="",
        remappings=[
            ("~/image_raw", "/cameras/stereo_left/image_raw"),
            ("~/camera_info", "/cameras/stereo_left/camera_info"),
            ("~/camera_mode", "/system/camera_mode"),
        ],
        parameters=[chessboard_params_file],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    all_components = [
        *camera_components,
        inference_component,
        chess_visualizer_component,
        chess_engine_component,
        chessboard_estimator_component,
    ]

    container = ComposableNodeContainer(
        name="lekiwi_perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=all_components,
        output="screen",
    )

    return LaunchDescription([container])
