import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    xacro_path = os.path.join(
        get_package_share_directory("lekiwi_description"),
        "urdf",
        "lekiwi_robot.urdf.xacro",
    )

    srdf_path = os.path.join(
        get_package_share_directory("lekiwi_moveit_config"),
        "config",
        "lekiwi_robot.srdf",
    )

    controllers_yaml = os.path.join(
        get_package_share_directory("lekiwi_bringup"),
        "config",
        "control",
        "lekiwi_controllers.yaml",
    )

    # 1. Robot description for robot_state_publisher and ros2_control
    robot_description_content = ParameterValue(
        Command([
            "xacro ", xacro_path,
            " use_ros2_control:=true",
            " hardware_type:=mock",
            " enable_imu:=false",
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": use_sim_time,
        }],
    )

    # 2. ros2_control controller_manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            controllers_yaml,
        ],
        output="screen",
    )

    # 3. Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 4. MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("lekiwi", package_name="lekiwi_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={
                "use_ros2_control": "true",
                "hardware_type": "mock",
                "enable_imu": "false",
            },
        )
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )

    # 5. move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    # 6. RViz2 node (optional)
    rviz_config_file = os.path.join(
        get_package_share_directory("lekiwi_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node,
    ])
