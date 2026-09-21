# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Complete entry-point launch file for LeKiwi Hand-Eye calibration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("lekiwi_calibration")
    bringup_share = FindPackageShare("lekiwi_bringup")
    description_share = FindPackageShare("lekiwi_description")

    default_params_file = PathJoinSubstitution(
        [pkg_share, "config", "handeye_params.yaml"]
    )
    default_camera_config = PathJoinSubstitution(
        [bringup_share, "config", "perception", "gscam_cameras.yaml"]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to YAML parameters file for handeye calibration",
        ),
        DeclareLaunchArgument(
            "camera_config",
            default_value=default_camera_config,
            description="Path to gscam camera configuration YAML file",
        ),
        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            description="Spawn and configure stereo_left camera for calibration",
        ),
        DeclareLaunchArgument(
            "bringup_robot",
            default_value="true",
            description="Start robot state publisher and arm controllers if not already running",
        ),
        DeclareLaunchArgument(
            "hardware_type",
            default_value="real",
            description="Hardware interface type: real or mock",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
    ]

    # Optional robot description & controllers bringup
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([description_share, "launch", "description.launch.py"])
        ),
        launch_arguments={
            "hardware_type": LaunchConfiguration("hardware_type"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("bringup_robot")),
    )

    robot_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "controllers.launch.py"])
        ),
        launch_arguments={
            "hardware_type": LaunchConfiguration("hardware_type"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "arm_controller": "true",
            "base_controller": "false",
            "imu_broadcaster": "false",
            "use_mag": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("bringup_robot")),
    )

    # stereo_left camera component in calib_mode (bypasses mode gatekeeper)
    stereo_left_camera_container = ComposableNodeContainer(
        name="stereo_left_calib_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="lekiwi_perception",
                plugin="lekiwi_perception::CameraStreamerComponent",
                name="gscam",
                namespace="cameras/stereo_left",
                parameters=[
                    LaunchConfiguration("camera_config"),
                    {
                        "calib_mode": True,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
                remappings=[
                    ("camera/image_raw", "image_raw"),
                    ("camera/image_raw/compressed", "image_raw/compressed"),
                    ("camera/camera_info", "camera_info"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

    # Interactive Hand-Eye Calibration Node
    calib_node = Node(
        package="lekiwi_calibration",
        executable="calibrate_handeye",
        name="handeye_calibration_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(
        [
            *declared_arguments,
            robot_description,
            robot_controllers,
            stereo_left_camera_container,
            calib_node,
        ]
    )
