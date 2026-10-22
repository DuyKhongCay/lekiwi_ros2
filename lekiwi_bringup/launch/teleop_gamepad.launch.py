# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Gamepad teleoperation bringup launch file for LeKiwi mobile base and arm.

Starts joy_linux driver node for Linux joystick devices, joy_teleop
to map raw gamepad axes to base cmd_vel, manipulation_action_server,
cartesian_jog_server_node, and gamepad_arm_teleop_node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Configure joy_linux driver, joy_teleop base node, jog server, and arm teleop node.

    Returns:
        LaunchDescription containing gamepad teleoperation nodes.
    """
    # 1. Resolve package paths and configuration files
    bringup_share = FindPackageShare("lekiwi_bringup")
    calibration_share = FindPackageShare("lekiwi_calibration")

    teleop_config_file = PathJoinSubstitution(
        [bringup_share, "config", "control", "gamepad_teleop.yaml"]
    )

    jog_config_file = PathJoinSubstitution(
        [bringup_share, "config", "control", "orchestrator.yaml"]
    )

    # 2. Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock"
    )

    manipulation_arg = DeclareLaunchArgument(
        "manipulation",
        default_value="kinematics",
        choices=["kinematics", "policy"],
        description="Arm manipulation mode: 'kinematics' (arm_trajectory_controller) or 'policy' (arm_forward_controller)",
    )

    controller_mode = PythonExpression(
        ["'forward' if '", LaunchConfiguration("manipulation"), "' == 'policy' else 'trajectory'"]
    )

    # 3. Define joy driver and joy_teleop nodes
    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        parameters=[
            teleop_config_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    joy_teleop_node = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        parameters=[
            teleop_config_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    jog_server_node = Node(
        package="lekiwi_motion",
        executable="cartesian_jog_server_node",
        name="cartesian_jog_server_node",
        parameters=[
            jog_config_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "controller_mode": controller_mode,
            },
        ],
        output="screen",
        # condition=IfCondition(LaunchConfiguration("arm_teleop")),
    )

    # 4. Assemble LaunchDescription
    return LaunchDescription(
        [
            use_sim_time_arg,
            manipulation_arg,
            # arm_teleop_arg,
            joy_node,
            joy_teleop_node,
            jog_server_node,
        ]
    )
