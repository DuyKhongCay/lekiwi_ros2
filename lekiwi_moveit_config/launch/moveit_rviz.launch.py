import os
import subprocess
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Xác định đường dẫn tương đối từ vị trí file launch này (hoàn toàn không phụ thuộc package share hay ament index)
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    moveit_pkg_dir = os.path.abspath(os.path.join(launch_dir, ".."))
    workspace_dir = os.path.abspath(os.path.join(moveit_pkg_dir, ".."))
    description_pkg_dir = os.path.join(workspace_dir, "lekiwi_description")

    # File paths
    xacro_file = os.path.join(description_pkg_dir, "urdf", "lekiwi_robot.urdf.xacro")
    srdf_file = os.path.join(moveit_pkg_dir, "config", "lekiwi_robot.srdf")
    rviz_config_file = os.path.join(moveit_pkg_dir, "config", "moveit.rviz")
    kinematics_file = os.path.join(moveit_pkg_dir, "config", "kinematics.yaml")
    ompl_file = os.path.join(moveit_pkg_dir, "config", "ompl_planning.yaml")
    joint_limits_file = os.path.join(moveit_pkg_dir, "config", "joint_limits.yaml")

    # 1. Sinh robot_description (URDF) từ xacro trực tiếp
    cmd = [
        "xacro",
        xacro_file,
        "hardware_type:=mock",
        "enable_imu:=false",
    ]
    try:
        robot_description_content = subprocess.check_output(cmd).decode("utf-8")
    except Exception as e:
        raise RuntimeError(f"Failed to generate URDF using xacro: {e}")

    # 2. Đọc trực tiếp nội dung SRDF
    with open(srdf_file, "r", encoding="utf-8") as f:
        robot_description_semantic_content = f.read()

    # 3. Đọc kinematics.yaml
    with open(kinematics_file, "r", encoding="utf-8") as f:
        kinematics_yaml = yaml.safe_load(f)

    # 4. Đọc ompl_planning.yaml
    with open(ompl_file, "r", encoding="utf-8") as f:
        ompl_yaml = yaml.safe_load(f)

    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "ompl": ompl_yaml,
    }

    # 5. Đọc joint_limits.yaml
    with open(joint_limits_file, "r", encoding="utf-8") as f:
        joint_limits_yaml = yaml.safe_load(f)

    # Cấu hình node rviz2 với toàn bộ parameters của MoveIt
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"robot_description": robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": joint_limits_yaml},
            planning_pipelines,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            rviz_node,
        ]
    )
