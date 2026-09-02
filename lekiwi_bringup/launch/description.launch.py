# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Publish the robot model and optionally manage its ros2_control stack."""
    bringup_share = FindPackageShare("lekiwi_bringup")
    description_share = FindPackageShare("lekiwi_description")

    xacro_file = PathJoinSubstitution(
        [description_share, "urdf", "lekiwi_robot.urdf.xacro"]
    )

    declared_args_spec = [
        ("hardware_type", "mock"),
        ("usb_port", "/dev/lekiwi_serial"),
        (
            "joint_config_file",
            PathJoinSubstitution(
                [bringup_share, "config", "servos", "lekiwi_arm_calib.yaml"]
            ),
        ),
        ("use_ros2_control", "true"),
        ("enable_imu", "true"),
        ("imu_hardware_type", "mock"),
        ("imu_i2c_bus", "1"),
        ("imu_i2c_address", "0x68"),
        ("imu_auto_calibrate_gyro", "true"),
        ("imu_gyro_calib_samples", "500"),
        (
            "controllers_file",
            PathJoinSubstitution(
                [bringup_share, "config", "controllers", "lekiwi_controllers.yaml"]
            ),
        ),
        ("start_controller_manager", "false"),
        ("activate_controllers", "false"),
        ("use_sim_time", "false"),
        ("frame_prefix", ""),
        ("joint_states_topic", "/joint_states"),
        ("ignore_timestamp", "false"),
    ]

    # Arguments passed directly to the xacro processor
    xacro_param_names = [
        "use_ros2_control",
        "hardware_type",
        "usb_port",
        "joint_config_file",
        "enable_imu",
        "imu_hardware_type",
        "imu_i2c_bus",
        "imu_i2c_address",
        "imu_auto_calibrate_gyro",
        "imu_gyro_calib_samples",
    ]

    xacro_command = ["xacro ", xacro_file]
    for param in xacro_param_names:
        xacro_command.extend([f" {param}:=", LaunchConfiguration(param)])

    robot_description = {
        "robot_description": ParameterValue(Command(xacro_command), value_type=str)
    }

    controller_config = LaunchConfiguration("controllers_file")

    controller_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("start_controller_manager"),
                "' == 'true' and '",
                LaunchConfiguration("activate_controllers"),
                "' == 'true'",
            ]
        )
    )
    imu_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("start_controller_manager"),
                "' == 'true' and '",
                LaunchConfiguration("activate_controllers"),
                "' == 'true' and '",
                LaunchConfiguration("enable_imu"),
                "' == 'true'",
            ]
        )
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_controller_manager")),
        parameters=[
            robot_description,
            controller_config,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            },
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=controller_condition,
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=controller_condition,
    )
    omni_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "omni_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=controller_condition,
    )
    imu_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "lekiwi_imu_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=imu_condition,
    )
    mag_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "lekiwi_magnetometer_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=imu_condition,
    )

    all_declared_arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in declared_args_spec
    ]

    return LaunchDescription(
        [
            *all_declared_arguments,
            Node(
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
                        "frame_prefix": LaunchConfiguration("frame_prefix"),
                        "ignore_timestamp": ParameterValue(
                            LaunchConfiguration("ignore_timestamp"), value_type=bool
                        ),
                        "publish_frequency": 50.0,
                    },
                ],
                remappings=[
                    ("joint_states", LaunchConfiguration("joint_states_topic"))
                ],
            ),
            controller_manager,
            joint_state_broadcaster,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[arm_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=arm_controller,
                    on_exit=[omni_base_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=omni_base_controller,
                    on_exit=[imu_broadcaster],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=imu_broadcaster,
                    on_exit=[mag_broadcaster],
                )
            ),
        ]
    )
