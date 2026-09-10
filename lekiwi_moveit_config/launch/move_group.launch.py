import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
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

    moveit_config = (
        MoveItConfigsBuilder("lekiwi", package_name="lekiwi_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={
                "use_ros2_control": "true",
                "hardware_type": "mock",
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

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            move_group_node,
        ]
    )
