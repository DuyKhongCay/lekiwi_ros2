# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Publish the robot model and optionally manage its ros2_control stack."""
    xacro_file = PathJoinSubstitution([
        FindPackageShare('lekiwi_description'), 'urdf', 'lekiwi_robot.urdf.xacro'
    ])
    controller_config = LaunchConfiguration('controllers_file')
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', xacro_file,
                ' use_ros2_control:=', LaunchConfiguration('use_ros2_control'),
                ' hardware_type:=', LaunchConfiguration('hardware_type'),
                ' usb_port:=', LaunchConfiguration('usb_port'),
                ' joint_config_file:=', LaunchConfiguration('joint_config_file'),
            ]),
            value_type=str,
        )
    }
    controller_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('start_controller_manager'),
        "' == 'true' and '", LaunchConfiguration('activate_controllers'),
        "' == 'true'",
    ]))
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_controller_manager')),
        parameters=[controller_config, {
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=controller_condition,
    )
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=controller_condition,
    )
    omni_base_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_base_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=controller_condition,
    )

    return LaunchDescription([
        DeclareLaunchArgument('hardware_type', default_value='mock'),
        DeclareLaunchArgument('usb_port', default_value='/dev/lekiwi_serial'),
        DeclareLaunchArgument(
            'joint_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lekiwi_bringup'), 'config', 'hardware', 'lekiwi_joints.yaml'
            ]),
        ),
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lekiwi_bringup'), 'config', 'controllers',
                'lekiwi_controllers.yaml'
            ]),
        ),
        DeclareLaunchArgument('start_controller_manager', default_value='false'),
        DeclareLaunchArgument('activate_controllers', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('ignore_timestamp', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
                'ignore_timestamp': ParameterValue(
                    LaunchConfiguration('ignore_timestamp'), value_type=bool),
                'publish_frequency': 50.0,
            }],
            remappings=[('joint_states', LaunchConfiguration('joint_states_topic'))],
        ),
        controller_manager,
        joint_state_broadcaster,
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=arm_controller,
            on_exit=[omni_base_controller],
        )),
    ])
